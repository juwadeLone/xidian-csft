`timescale 1ns/1ps

// GR16S variant of phase_compensator_buffered: the wrapper logic is VERBATIM;
// the inner compensator is phase_compensator_grp when GRP_EN != 0 (the
// product node carries the two replica domains and the product vote) and the
// plain W16R2 phase_compensator when GRP_EN == 0 (the pass's other,
// unprotected compensator slot).  rpr_sticky reports {product, coef, rec} of
// the inner instance.
module phase_compensator_grp_buffered #(
    parameter integer LOGN              = 12,
    parameter integer CORDIC_ITERATIONS = 25,
    parameter integer COEF_W            = 25,
    parameter integer IN_W              = 11,
    parameter integer OUT_W             = 11,
    parameter integer REC_TMR           = 0,
    parameter integer GRP_EN            = 0,
    parameter integer GRP_K             = 10,
    parameter integer GRP_T             = 4,
    parameter integer GRP_KC            = 12,
    parameter GRP_MEM                   = "chain/rtl/mem/grp17_coef_kc12.mem"
) (
    input  wire                         clk,
    input  wire                         rst,
    input  wire                         clear,

    input  wire                         seed_valid,
    output wire                         seed_ready,
    input  wire [31:0]                  seed_theta0,
    input  wire [31:0]                  seed_delta1,
    input  wire [31:0]                  seed_delta2,

    input  wire                         s_valid,
    output wire                         s_ready,
    input  wire                         s_sof,
    input  wire                         s_eof,
    input  wire signed [IN_W-1:0]       s_re,
    input  wire signed [IN_W-1:0]       s_im,

    output wire                         m_valid,
    input  wire                         m_ready,
    output wire                         m_sof,
    output wire                         m_eof,
    output wire signed [OUT_W-1:0]      m_re,
    output wire signed [OUT_W-1:0]      m_im,

    output wire                         busy,
    output wire                         protocol_error,
    output wire                         overflow_sticky,
    output wire [2:0]                   rpr_sticky
);
    localparam integer N = 1 << LOGN;

    initial begin
        if (LOGN < 1)
            $error("phase_compensator_buffered requires LOGN >= 1");
        if (IN_W < 2 || OUT_W < 2)
            $error("phase_compensator_buffered requires IN_W, OUT_W >= 2");
    end

    // Independent two-entry seed FIFO.  Entries are removed only when a new
    // input vector has both a valid SOF and an available vector credit.
    reg [31:0] seed_theta0_fifo [0:1];
    reg [31:0] seed_delta1_fifo [0:1];
    reg [31:0] seed_delta2_fifo [0:1];
    reg        seed_wr_ptr;
    reg        seed_rd_ptr;
    reg [1:0]  seed_count;

    reg                         input_active;
    reg [LOGN-1:0]              input_index;
    reg [1:0]                   inflight_count;
    reg                         armed_hold_q;
    reg signed [IN_W-1:0]       armed_re_q;
    reg signed [IN_W-1:0]       armed_im_q;
    reg                         armed_sof_q;
    reg                         armed_eof_q;

    reg                         protocol_error_q;
    reg                         buffer_overflow_q;
    reg                         fault_q;

    wire [31:0] comp_theta0 = seed_theta0_fifo[seed_rd_ptr];
    wire [31:0] comp_delta1 = seed_delta1_fifo[seed_rd_ptr];
    wire [31:0] comp_delta2 = seed_delta2_fifo[seed_rd_ptr];

    localparam integer DW_EL = 2 * OUT_W;
    wire [DW_EL-1:0] elastic_s_data;
    wire        elastic_s_valid;
    wire        elastic_s_ready;
    wire        elastic_s_sof;
    wire        elastic_s_eof;
    wire [DW_EL-1:0] elastic_m_data;
    wire        elastic_m_valid;
    wire        elastic_m_ready;
    wire        elastic_m_sof;
    wire        elastic_m_eof;
    wire        elastic_occupied;
    wire        elastic_protocol_error;
    wire        elastic_overflow_error;

    wire        comp_m_valid;
    wire        comp_m_sof;
    wire        comp_m_eof;
    wire signed [OUT_W-1:0] comp_m_re;
    wire signed [OUT_W-1:0] comp_m_im;
    wire        comp_sample_overflow;
    wire        comp_overflow_sticky;
    wire        comp_alignment_error;

    wire public_output_fire = elastic_m_valid && m_ready && !fault_q;
    wire public_output_eof_fire = public_output_fire && elastic_m_eof;
    wire vector_credit_available = (inflight_count < 2) ||
                                   public_output_eof_fire;
    wire first_tags_ok = s_sof && (s_eof == (N == 1));

    // Loading is deliberately performed while s_ready is low.  The producer
    // keeps its SOF beat stable, the seed is sampled at this edge, and the same
    // beat can first enter phase_compensator on the following edge.
    wire comp_seed_load = !fault_q && !input_active && s_valid &&
                          first_tags_ok && (seed_count != 0) &&
                          vector_credit_available;

    wire seed_fire = seed_valid && seed_ready;
    wire expected_sof = (input_index == 0);
    wire expected_eof = (input_index == N-1);
    wire active_tags_ok = (s_sof == expected_sof) &&
                          (s_eof == expected_eof);
    wire input_fire = s_valid && s_ready;
    wire malformed_active_beat = input_fire && !active_tags_ok;
    wire malformed_idle_beat = !fault_q && !input_active && s_valid &&
                               (!s_sof || (s_eof != (N == 1)));

    // The beat that requested seed loading was stalled intentionally.  It is
    // a ready/valid protocol violation if the producer withdraws or changes it
    // before the next-cycle handshake.
    wire armed_hold_violation = armed_hold_q &&
                                (!s_valid || s_re != armed_re_q ||
                                 s_im != armed_im_q ||
                                 s_sof != armed_sof_q ||
                                 s_eof != armed_eof_q);

    wire comp_s_valid = input_fire && active_tags_ok &&
                        !armed_hold_violation;

    // This event must be unreachable if vector credits and the elastic FIFO
    // agree.  Since phase_compensator cannot stall, observe it explicitly and
    // fail closed instead of silently dropping a changing output sample.
    wire elastic_input_overrun = comp_m_valid && !elastic_s_ready && !fault_q;
    wire protocol_event_now = malformed_active_beat || malformed_idle_beat ||
                              armed_hold_violation || comp_alignment_error ||
                              elastic_protocol_error;
    wire hard_fault_event = protocol_event_now || elastic_input_overrun ||
                            elastic_overflow_error;

    // Gate interfaces from the registered fail-closed state.  Do not feed a
    // same-cycle fault term back through elastic m_ready/s_ready: that would
    // create a combinational loop through EOF credit release.
    assign seed_ready = !fault_q && (seed_count < 2);
    assign s_ready = !fault_q && input_active;

    assign elastic_s_data  = {comp_m_im, comp_m_re};
    assign elastic_s_valid = comp_m_valid && !fault_q;
    assign elastic_s_sof   = comp_m_sof;
    assign elastic_s_eof   = comp_m_eof;
    assign elastic_m_ready = m_ready && !fault_q;

    assign m_valid = elastic_m_valid && !fault_q;
    assign m_sof   = elastic_m_sof;
    assign m_eof   = elastic_m_eof;
    assign m_re    = $signed(elastic_m_data[OUT_W-1:0]);
    assign m_im    = $signed(elastic_m_data[DW_EL-1:OUT_W]);

    assign protocol_error = protocol_error_q;
    assign overflow_sticky = comp_overflow_sticky || buffer_overflow_q ||
                             elastic_overflow_error;
    assign busy = fault_q || (seed_count != 0) || input_active ||
                  (inflight_count != 0) || elastic_occupied;

    generate
        if (GRP_EN != 0) begin : g_grp
            phase_compensator_grp #(
                .CORDIC_ITERATIONS(CORDIC_ITERATIONS), .COEF_W(COEF_W),
                .IN_W(IN_W), .OUT_W(OUT_W), .REC_TMR(REC_TMR),
                .GRP_K(GRP_K), .GRP_T(GRP_T), .GRP_KC(GRP_KC),
                .GRP_MEM(GRP_MEM)
            ) u_compensator (
                .clk(clk),
                .rst(rst),
                .clear(clear),
                .seed_load(comp_seed_load),
                .theta0(comp_theta0),
                .delta1(comp_delta1),
                .delta2(comp_delta2),
                .s_valid(comp_s_valid),
                .s_sof(s_sof),
                .s_eof(s_eof),
                .s_re(s_re),
                .s_im(s_im),
                .m_valid(comp_m_valid),
                .m_sof(comp_m_sof),
                .m_eof(comp_m_eof),
                .m_re(comp_m_re),
                .m_im(comp_m_im),
                .sample_overflow(comp_sample_overflow),
                .overflow_sticky(comp_overflow_sticky),
                .alignment_error(comp_alignment_error),
                .rpr_clr(1'b0),
                .rpr_sticky(rpr_sticky)
            );
        end else begin : g_plain
            phase_compensator #(
                .CORDIC_ITERATIONS(CORDIC_ITERATIONS), .COEF_W(COEF_W),
                .IN_W(IN_W), .OUT_W(OUT_W), .REC_TMR(REC_TMR)
            ) u_compensator (
                .clk(clk),
                .rst(rst),
                .clear(clear),
                .seed_load(comp_seed_load),
                .theta0(comp_theta0),
                .delta1(comp_delta1),
                .delta2(comp_delta2),
                .s_valid(comp_s_valid),
                .s_sof(s_sof),
                .s_eof(s_eof),
                .s_re(s_re),
                .s_im(s_im),
                .m_valid(comp_m_valid),
                .m_sof(comp_m_sof),
                .m_eof(comp_m_eof),
                .m_re(comp_m_re),
                .m_im(comp_m_im),
                .sample_overflow(comp_sample_overflow),
                .overflow_sticky(comp_overflow_sticky),
                .alignment_error(comp_alignment_error),
                .rpr_clr(1'b0),
                .rpr_sticky()
            );
            assign rpr_sticky = 3'b000;
        end
    endgenerate

    vector_elastic_pingpong #(
        .LOGN(LOGN),
        .DATA_W(DW_EL)
    ) u_elastic (
        .clk(clk),
        .rst(rst),
        .clear(clear),
        .s_data(elastic_s_data),
        .s_valid(elastic_s_valid),
        .s_ready(elastic_s_ready),
        .s_sof(elastic_s_sof),
        .s_eof(elastic_s_eof),
        .m_data(elastic_m_data),
        .m_valid(elastic_m_valid),
        .m_ready(elastic_m_ready),
        .m_sof(elastic_m_sof),
        .m_eof(elastic_m_eof),
        .occupied(elastic_occupied),
        .protocol_error(elastic_protocol_error),
        .overflow_error(elastic_overflow_error)
    );

    always @(posedge clk) begin
        if (rst || clear) begin
            seed_wr_ptr       <= 1'b0;
            seed_rd_ptr       <= 1'b0;
            seed_count        <= 2'd0;
            input_active      <= 1'b0;
            input_index       <= {LOGN{1'b0}};
            inflight_count    <= 2'd0;
            armed_hold_q      <= 1'b0;
            armed_re_q        <= '0;
            armed_im_q        <= '0;
            armed_sof_q       <= 1'b0;
            armed_eof_q       <= 1'b0;
            protocol_error_q  <= 1'b0;
            buffer_overflow_q <= 1'b0;
            fault_q           <= 1'b0;
        end else begin
            if (seed_fire) begin
                seed_theta0_fifo[seed_wr_ptr] <= seed_theta0;
                seed_delta1_fifo[seed_wr_ptr] <= seed_delta1;
                seed_delta2_fifo[seed_wr_ptr] <= seed_delta2;
                seed_wr_ptr <= ~seed_wr_ptr;
            end
            if (comp_seed_load)
                seed_rd_ptr <= ~seed_rd_ptr;

            case ({seed_fire, comp_seed_load})
                2'b10: seed_count <= seed_count + 1'b1;
                2'b01: seed_count <= seed_count - 1'b1;
                default: seed_count <= seed_count;
            endcase

            if (comp_seed_load) begin
                input_active <= 1'b1;
                input_index  <= {LOGN{1'b0}};
                armed_hold_q <= 1'b1;
                armed_re_q   <= s_re;
                armed_im_q   <= s_im;
                armed_sof_q  <= s_sof;
                armed_eof_q  <= s_eof;
            end

            if (input_fire && active_tags_ok && !armed_hold_violation) begin
                armed_hold_q <= 1'b0;
                if (input_index == N-1) begin
                    input_active <= 1'b0;
                    input_index  <= {LOGN{1'b0}};
                end else begin
                    input_index <= input_index + 1'b1;
                end
            end

            case ({comp_seed_load, public_output_eof_fire})
                2'b10: begin
                    if (inflight_count == 2) begin
                        protocol_error_q <= 1'b1;
                        fault_q <= 1'b1;
                    end else begin
                        inflight_count <= inflight_count + 1'b1;
                    end
                end
                2'b01: begin
                    if (inflight_count == 0) begin
                        protocol_error_q <= 1'b1;
                        fault_q <= 1'b1;
                    end else begin
                        inflight_count <= inflight_count - 1'b1;
                    end
                end
                default: inflight_count <= inflight_count;
            endcase

            if (protocol_event_now) begin
                protocol_error_q <= 1'b1;
                fault_q <= 1'b1;
            end
            if (elastic_input_overrun || elastic_overflow_error) begin
                protocol_error_q  <= 1'b1;
                buffer_overflow_q <= 1'b1;
                fault_q           <= 1'b1;
            end
        end
    end
endmodule
