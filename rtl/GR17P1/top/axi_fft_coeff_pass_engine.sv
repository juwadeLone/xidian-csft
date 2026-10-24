`timescale 1ns/1ps

// Parameterized DDR -> optional pre-compensation -> natural-order transform
// lane -> optional post-compensation -> DDR pass.
//
// The lane is either the fixed-point SDF lane (BFP_LANE = 0) or the
// block-floating-point IFFT lane (BFP_LANE = 1).  DDR words are 64-bit
// containers (re and im sign-extended to 32 bits); the compensators and the
// lane narrow them to their own component widths and sign-extend back.
// overflow_sticky reports saturation anywhere in the pass and is informational:
// the model saturates, so a saturating vector is a legitimate result.

module axi_fft_coeff_pass_engine #(
    parameter integer LOGN = 12,
    parameter integer ADDR_W = 36,
    parameter integer ID_W = 4,
    parameter integer INVERSE = 0,
    parameter [11:0] SCALE_MASK = 12'hfff,
    parameter [3:0] CORE_ID = 0,
    parameter integer CORDIC_ITERATIONS = 25,
    // coefficient component width shared by both compensators (CORDIC output width)
    parameter integer COEF_W = 25,
    parameter integer BFP_LANE = 0,
    // lane widths: W = core component (SDF) / mantissa (BFP) width,
    // LANE_IN_W = component width the input container carries,
    // LANE_OUT_W = component width of the output: the BFP lane produces it; the SDF lane (W bits)
    // saturates to it at the exit when LANE_OUT_W < LANE_W
    parameter integer LANE_W = 11,
    parameter integer LANE_IN_W = 11,
    parameter integer LANE_OUT_W = 11,
    // SDF lane only: 0 < LANE_ENTRY_W < LANE_W limits the lane input to LANE_ENTRY_W bits (sign-extended to LANE_W)
    parameter integer LANE_ENTRY_W = 0,
    parameter integer BFP_OUT_SHIFT = 0,
    parameter integer BFP_NORM1_MAX = 10,
    parameter integer BFP_NORM2_MAX = 3,
    // compensator component widths (in -> out)
    parameter integer PRE_IN_W = 10,
    parameter integer PRE_OUT_W = 10,
    parameter integer POST_IN_W = 11,
    parameter integer POST_OUT_W = 11,
    // rpr_noroll: spectrum centring by addressing (sdf_fft_lane_natural)
    parameter integer FFT_OUT_SHIFT = 0,
    parameter integer FFT_IN_SHIFT  = 0
) (
    input  wire                         clk,
    input  wire                         rst,
    input  wire                         clear,
    input  wire                         start,
    input  wire [ADDR_W-1:0]            source_addr,
    input  wire [ADDR_W-1:0]            destination_addr,
    input  wire [31:0]                  vector_count,
    input  wire                         pre_coeff_enable,
    input  wire                         post_coeff_enable,

    input  wire                         pre_seed_valid,
    output wire                         pre_seed_ready,
    input  wire [31:0]                  pre_seed_theta0,
    input  wire [31:0]                  pre_seed_delta1,
    input  wire [31:0]                  pre_seed_delta2,

    input  wire                         post_seed_valid,
    output wire                         post_seed_ready,
    input  wire [31:0]                  post_seed_theta0,
    input  wire [31:0]                  post_seed_delta1,
    input  wire [31:0]                  post_seed_delta2,

    output wire                         busy,
    output reg                          done,
    output reg                          error,
    output wire                         protocol_error,
    output wire                         overflow_sticky,

    output wire [ID_W-1:0]              m_axi_awid,
    output wire [ADDR_W-1:0]            m_axi_awaddr,
    output wire [7:0]                   m_axi_awlen,
    output wire [2:0]                   m_axi_awsize,
    output wire [1:0]                   m_axi_awburst,
    output wire                         m_axi_awvalid,
    input  wire                         m_axi_awready,
    output wire [255:0]                 m_axi_wdata,
    output wire [31:0]                  m_axi_wstrb,
    output wire                         m_axi_wlast,
    output wire                         m_axi_wvalid,
    input  wire                         m_axi_wready,
    input  wire [ID_W-1:0]              m_axi_bid,
    input  wire [1:0]                   m_axi_bresp,
    input  wire                         m_axi_bvalid,
    output wire                         m_axi_bready,

    output wire [ID_W-1:0]              m_axi_arid,
    output wire [ADDR_W-1:0]            m_axi_araddr,
    output wire [7:0]                   m_axi_arlen,
    output wire [2:0]                   m_axi_arsize,
    output wire [1:0]                   m_axi_arburst,
    output wire                         m_axi_arvalid,
    input  wire                         m_axi_arready,
    input  wire [ID_W-1:0]              m_axi_rid,
    input  wire [255:0]                 m_axi_rdata,
    input  wire [1:0]                   m_axi_rresp,
    input  wire                         m_axi_rlast,
    input  wire                         m_axi_rvalid,
    output wire                         m_axi_rready
);
    localparam integer N = 1 << LOGN;

    initial begin
        if (LOGN < 6 || LOGN > 12)
            $error("axi_fft_coeff_pass_engine requires 6 <= LOGN <= 12");
    end

    wire [63:0] start_total_beats_ext =
        {32'd0, vector_count} << (LOGN-2);
    wire invalid_start = (vector_count == 0) ||
                         (start_total_beats_ext[63:32] != 0) ||
                         (start_total_beats_ext[3:0] != 0) ||
                         (source_addr[8:0] != 0) ||
                         (destination_addr[8:0] != 0);

    reg active;
    reg launched;
    reg child_start;
    reg [ADDR_W-1:0] source_addr_q;
    reg [ADDR_W-1:0] destination_addr_q;
    reg [31:0] vector_count_q;
    reg [31:0] total_beats_q;
    reg pre_enable_q;
    reg post_enable_q;
    reg reader_seen;
    reg writer_seen;

    // Every accepted launch gets one full-cycle local reset before either
    // child can emit AXI VALID.  This clears sticky state without dropping an
    // already-issued AXI transaction.
    wire child_clear = clear || (active && !launched);
    wire run_accept_enable = active && launched && !child_clear;

    wire reader_busy;
    wire reader_done;
    wire reader_error;
    wire writer_busy;
    wire writer_done;
    wire writer_error;
    wire [255:0] read_data;
    wire read_valid;
    wire read_ready;
    wire [255:0] packed_data;
    wire packed_valid;
    wire packed_ready;

    axi_burst_reader_256 #(
        .ADDR_W(ADDR_W), .ID_W(ID_W), .AXI_ID(1)
    ) u_reader (
        .clk(clk), .rst(rst), .clear(child_clear), .start(child_start),
        .base_addr(source_addr_q), .total_beats(total_beats_q),
        .busy(reader_busy), .done(reader_done), .error(reader_error),
        .m_data(read_data), .m_valid(read_valid), .m_ready(read_ready),
        .m_axi_arid(m_axi_arid), .m_axi_araddr(m_axi_araddr),
        .m_axi_arlen(m_axi_arlen), .m_axi_arsize(m_axi_arsize),
        .m_axi_arburst(m_axi_arburst), .m_axi_arvalid(m_axi_arvalid),
        .m_axi_arready(m_axi_arready), .m_axi_rid(m_axi_rid),
        .m_axi_rdata(m_axi_rdata), .m_axi_rresp(m_axi_rresp),
        .m_axi_rlast(m_axi_rlast), .m_axi_rvalid(m_axi_rvalid),
        .m_axi_rready(m_axi_rready)
    );

    axi_burst_writer_256 #(
        .ADDR_W(ADDR_W), .ID_W(ID_W), .AXI_ID(2)
    ) u_writer (
        .clk(clk), .rst(rst), .clear(child_clear), .start(child_start),
        .base_addr(destination_addr_q), .total_beats(total_beats_q),
        .busy(writer_busy), .done(writer_done), .error(writer_error),
        .s_data(packed_data), .s_valid(packed_valid), .s_ready(packed_ready),
        .m_axi_awid(m_axi_awid), .m_axi_awaddr(m_axi_awaddr),
        .m_axi_awlen(m_axi_awlen), .m_axi_awsize(m_axi_awsize),
        .m_axi_awburst(m_axi_awburst), .m_axi_awvalid(m_axi_awvalid),
        .m_axi_awready(m_axi_awready), .m_axi_wdata(m_axi_wdata),
        .m_axi_wstrb(m_axi_wstrb), .m_axi_wlast(m_axi_wlast),
        .m_axi_wvalid(m_axi_wvalid), .m_axi_wready(m_axi_wready),
        .m_axi_bid(m_axi_bid), .m_axi_bresp(m_axi_bresp),
        .m_axi_bvalid(m_axi_bvalid), .m_axi_bready(m_axi_bready)
    );

    // AXI beat serialization and source frame tags.
    wire [63:0] source_data;
    wire source_valid;
    wire source_ready;
    reg [LOGN-1:0] source_index;
    wire source_sof = source_index == 0;
    wire source_eof = source_index == N-1;
    wire source_fire = source_valid && source_ready;

    stream_256_to_64 u_unpack (
        .clk(clk), .rst(rst), .clear(child_clear),
        .s_data(read_data), .s_valid(read_valid), .s_ready(read_ready),
        .m_data(source_data), .m_valid(source_valid), .m_ready(source_ready)
    );

    // Optional pre-FFT phase compensation.  The external seed interface is
    // limited to exactly vector_count accepted seeds for an enabled pass.
    reg [31:0] pre_seed_count;
    wire pre_seed_ready_internal;
    wire pre_phase_s_ready;
    wire pre_phase_m_valid;
    wire pre_phase_m_ready;
    wire pre_phase_m_sof;
    wire pre_phase_m_eof;
    wire signed [PRE_OUT_W-1:0] pre_phase_m_re;
    wire signed [PRE_OUT_W-1:0] pre_phase_m_im;
    wire [63:0] pre_phase_container;
    wire pre_phase_busy;
    wire pre_phase_protocol;
    wire pre_phase_overflow;
    wire pre_seed_allowed = run_accept_enable && pre_enable_q &&
                            (pre_seed_count < vector_count_q);
    wire pre_seed_fire = pre_seed_valid && pre_seed_ready;

    assign pre_seed_ready = pre_seed_allowed && pre_seed_ready_internal;

    phase_compensator_buffered #(
        .LOGN(LOGN), .CORDIC_ITERATIONS(CORDIC_ITERATIONS),
        .IN_W(PRE_IN_W), .OUT_W(PRE_OUT_W), .COEF_W(COEF_W)
    ) u_pre_compensation (
        .clk(clk), .rst(rst), .clear(child_clear),
        .seed_valid(pre_seed_valid && pre_seed_allowed),
        .seed_ready(pre_seed_ready_internal),
        .seed_theta0(pre_seed_theta0), .seed_delta1(pre_seed_delta1),
        .seed_delta2(pre_seed_delta2),
        .s_valid(source_valid && pre_enable_q), .s_ready(pre_phase_s_ready),
        .s_sof(source_sof), .s_eof(source_eof),
        .s_re(source_data[PRE_IN_W-1:0]), .s_im(source_data[32+PRE_IN_W-1:32]),
        .m_valid(pre_phase_m_valid), .m_ready(pre_phase_m_ready),
        .m_sof(pre_phase_m_sof), .m_eof(pre_phase_m_eof),
        .m_re(pre_phase_m_re), .m_im(pre_phase_m_im),
        .busy(pre_phase_busy), .protocol_error(pre_phase_protocol),
        .overflow_sticky(pre_phase_overflow)
    );
    pair_sext #(.W(PRE_OUT_W)) u_pre_sext (
        .din({pre_phase_m_im, pre_phase_m_re}), .dout(pre_phase_container));

    wire [63:0] pre_stage_data = pre_enable_q ?
                                 pre_phase_container : source_data;
    wire pre_stage_valid = pre_enable_q ? pre_phase_m_valid : source_valid;
    wire pre_stage_sof = pre_enable_q ? pre_phase_m_sof : source_sof;
    wire pre_stage_eof = pre_enable_q ? pre_phase_m_eof : source_eof;
    wire pre_stage_ready;
    wire fft_input_ready;
    assign source_ready = pre_enable_q ? pre_phase_s_ready : pre_stage_ready;
    assign pre_phase_m_ready = pre_enable_q ? pre_stage_ready : 1'b0;

    assign pre_stage_ready = fft_input_ready;

    wire [63:0] fft_input_data = pre_stage_data;
    wire fft_input_valid = pre_stage_valid;
    wire fft_input_sof = pre_stage_sof;
    wire fft_input_eof = pre_stage_eof;

    wire [63:0] fft_output_data;
    wire fft_output_valid;
    wire fft_output_ready;
    wire fft_output_sof;
    wire fft_output_eof;
    wire lane_busy;
    wire lane_overflow;
    wire lane_protocol;

    generate
        if (BFP_LANE == 0) begin : g_sdf
            wire [LOGN-1:0] unused_stage_overflow;
            wire [LOGN*12-1:0] unused_delay_addr;
            wire [LOGN*12-1:0] unused_twiddle_addr;
            wire [LOGN-1:0] unused_stage_valid;
            wire [3:0] unused_core_id;
            wire unused_sat_re, unused_sat_im;
            wire [63:0] lane_m_data;

            sdf_fft_lane_natural #(
                .LOGN(LOGN), .INVERSE(INVERSE), .SCALE_MASK(SCALE_MASK),
                .CORE_ID(CORE_ID), .W(LANE_W), .IN_W(LANE_IN_W),
                .ENTRY_W(LANE_ENTRY_W),
                .OUT_SHIFT(FFT_OUT_SHIFT), .IN_SHIFT(FFT_IN_SHIFT)
            ) u_fft (
                .clk(clk), .rst(rst), .clear(child_clear),
                .s_data(fft_input_data), .s_valid(fft_input_valid),
                .s_ready(fft_input_ready), .s_sof(fft_input_sof),
                .s_eof(fft_input_eof), .m_data(lane_m_data),
                .m_valid(fft_output_valid), .m_ready(fft_output_ready),
                .m_sof(fft_output_sof), .m_eof(fft_output_eof),
                .busy(lane_busy), .overflow_sticky(lane_overflow),
                .stage_overflow(unused_stage_overflow),
                .debug_delay_addr(unused_delay_addr),
                .debug_twiddle_addr(unused_twiddle_addr),
                .debug_stage_valid(unused_stage_valid), .core_id(unused_core_id),
                .input_sat_re(unused_sat_re), .input_sat_im(unused_sat_im),
                .protocol_error(lane_protocol)
            );

            // Exit limit (the model's _limit_pair to a narrower memory or output width); the lane output is a
            // LANE_W-bit sign-extended container, so the narrowing is the SRC_W > W clamp.
            if (LANE_OUT_W < LANE_W) begin : g_exit_limit
                wire [2*LANE_OUT_W-1:0] limited;
                wire exit_sat_re, exit_sat_im;
                pair_narrow #(.SRC_W(LANE_W), .W(LANE_OUT_W)) u_exit_narrow (
                    .clk(clk), .valid(fft_output_valid), .din(lane_m_data), .dout(limited),
                    .sat_re(exit_sat_re), .sat_im(exit_sat_im));
                pair_sext #(.W(LANE_OUT_W)) u_exit_sext (.din(limited), .dout(fft_output_data));
            end else begin : g_exit_pass
                assign fft_output_data = lane_m_data;
            end
        end else begin : g_bfp
            bfp_ifft_lane_natural #(
                .LOGN(LOGN), .W(LANE_W), .IN_W(LANE_IN_W), .OUT_W(LANE_OUT_W),
                .CORE_OUT_SHIFT(BFP_OUT_SHIFT),
                .NORM1_MAX(BFP_NORM1_MAX), .NORM2_MAX(BFP_NORM2_MAX),
                .OUT_SHIFT(FFT_OUT_SHIFT), .IN_SHIFT(FFT_IN_SHIFT)
            ) u_fft (
                .clk(clk), .rst(rst), .clear(child_clear),
                .s_data(fft_input_data), .s_valid(fft_input_valid),
                .s_ready(fft_input_ready), .s_sof(fft_input_sof),
                .s_eof(fft_input_eof), .m_data(fft_output_data),
                .m_valid(fft_output_valid), .m_ready(fft_output_ready),
                .m_sof(fft_output_sof), .m_eof(fft_output_eof),
                .busy(lane_busy), .overflow_sticky(lane_overflow),
                .protocol_error(lane_protocol)
            );
        end
    endgenerate

    // Optional post-FFT phase compensation with its own independent seed
    // stream and vector-credit wrapper.
    reg [31:0] post_seed_count;
    wire post_seed_ready_internal;
    wire post_phase_s_ready;
    wire post_phase_m_valid;
    wire post_phase_m_ready;
    wire post_phase_m_sof;
    wire post_phase_m_eof;
    wire signed [POST_OUT_W-1:0] post_phase_m_re;
    wire signed [POST_OUT_W-1:0] post_phase_m_im;
    wire [63:0] post_phase_container;
    wire post_phase_busy;
    wire post_phase_protocol;
    wire post_phase_overflow;
    wire post_seed_allowed = run_accept_enable && post_enable_q &&
                             (post_seed_count < vector_count_q);
    wire post_seed_fire = post_seed_valid && post_seed_ready;

    assign post_seed_ready = post_seed_allowed && post_seed_ready_internal;

    phase_compensator_buffered #(
        .LOGN(LOGN), .CORDIC_ITERATIONS(CORDIC_ITERATIONS),
        .IN_W(POST_IN_W), .OUT_W(POST_OUT_W), .COEF_W(COEF_W)
    ) u_post_compensation (
        .clk(clk), .rst(rst), .clear(child_clear),
        .seed_valid(post_seed_valid && post_seed_allowed),
        .seed_ready(post_seed_ready_internal),
        .seed_theta0(post_seed_theta0), .seed_delta1(post_seed_delta1),
        .seed_delta2(post_seed_delta2),
        .s_valid(fft_output_valid && post_enable_q),
        .s_ready(post_phase_s_ready), .s_sof(fft_output_sof),
        .s_eof(fft_output_eof), .s_re(fft_output_data[POST_IN_W-1:0]),
        .s_im(fft_output_data[32+POST_IN_W-1:32]),
        .m_valid(post_phase_m_valid), .m_ready(post_phase_m_ready),
        .m_sof(post_phase_m_sof), .m_eof(post_phase_m_eof),
        .m_re(post_phase_m_re), .m_im(post_phase_m_im),
        .busy(post_phase_busy), .protocol_error(post_phase_protocol),
        .overflow_sticky(post_phase_overflow)
    );
    pair_sext #(.W(POST_OUT_W)) u_post_sext (
        .din({post_phase_m_im, post_phase_m_re}), .dout(post_phase_container));

    wire [63:0] final_sample_data = post_enable_q ?
                                    post_phase_container :
                                    fft_output_data;
    wire final_sample_valid = post_enable_q ?
                              post_phase_m_valid : fft_output_valid;
    wire final_sample_ready;
    assign fft_output_ready = post_enable_q ?
                              post_phase_s_ready : final_sample_ready;
    assign post_phase_m_ready = post_enable_q ? final_sample_ready : 1'b0;

    stream_64_to_256 u_pack (
        .clk(clk), .rst(rst), .clear(child_clear),
        .s_data(final_sample_data), .s_valid(final_sample_valid),
        .s_ready(final_sample_ready), .m_data(packed_data),
        .m_valid(packed_valid), .m_ready(packed_ready)
    );

    assign protocol_error = pre_phase_protocol || post_phase_protocol ||
                            lane_protocol;
    assign overflow_sticky = pre_phase_overflow || post_phase_overflow ||
                             lane_overflow;

    assign busy = active || reader_busy || writer_busy || lane_busy ||
                  pre_phase_busy || post_phase_busy;

    always @(posedge clk) begin
        if (rst || clear) begin
            active <= 1'b0;
            launched <= 1'b0;
            child_start <= 1'b0;
            source_addr_q <= '0;
            destination_addr_q <= '0;
            vector_count_q <= '0;
            total_beats_q <= '0;
            pre_enable_q <= 1'b0;
            post_enable_q <= 1'b0;
            reader_seen <= 1'b0;
            writer_seen <= 1'b0;
            source_index <= '0;
            pre_seed_count <= '0;
            post_seed_count <= '0;
            done <= 1'b0;
            error <= 1'b0;
        end else begin
            done <= 1'b0;
            child_start <= 1'b0;

            if (start) begin
                if (busy) begin
                    // The current transaction and all latched controls remain
                    // untouched; no second child_start or AXI request occurs.
                    error <= 1'b1;
                end else if (invalid_start) begin
                    done <= 1'b1;
                    error <= 1'b1;
                end else begin
                    active <= 1'b1;
                    launched <= 1'b0;
                    source_addr_q <= source_addr;
                    destination_addr_q <= destination_addr;
                    vector_count_q <= vector_count;
                    total_beats_q <= start_total_beats_ext[31:0];
                    pre_enable_q <= pre_coeff_enable;
                    post_enable_q <= post_coeff_enable;
                    reader_seen <= 1'b0;
                    writer_seen <= 1'b0;
                    source_index <= '0;
                    pre_seed_count <= '0;
                    post_seed_count <= '0;
                    error <= 1'b0;
                end
            end

            if (active && !launched) begin
                launched <= 1'b1;
                child_start <= 1'b1;
            end

            if (source_fire) begin
                if (source_index == N-1)
                    source_index <= '0;
                else
                    source_index <= source_index + 1'b1;
            end

            if (pre_seed_fire)
                pre_seed_count <= pre_seed_count + 1'b1;
            if (post_seed_fire)
                post_seed_count <= post_seed_count + 1'b1;

            if (reader_done)
                reader_seen <= 1'b1;
            if (writer_done)
                writer_seen <= 1'b1;

            if (active && launched && !child_start &&
                (reader_error || writer_error || protocol_error))
                error <= 1'b1;

            if (active && launched &&
                (reader_seen || reader_done) &&
                (writer_seen || writer_done)) begin
                active <= 1'b0;
                done <= 1'b1;
            end
        end
    end
endmodule
