`timescale 1ns/1ps

// Complete physical transform lane: two-vector prefetch, streaming SDF core,
// and two-bank bit-reversal reorder.  AXI jitter is confined to the prefetch
// side; the SDF core sees gap-free vectors.
//
// The lane is the channel: s_data / m_data are 64-bit DDR containers (re and
// im each sign-extended to 32 bits).  The input is narrowed to W bits per
// component at the entry (pair_narrow; IN_W > W clamps), both banks and the
// core run at DATA_W = 2*W / W, and the output is sign-extended back into the
// container at the exit.  0 < ENTRY_W < W clamps the input to ENTRY_W bits
// instead and sign-extends the result to W bits (the chain entry limit of P1,
// the model's _limit_pair to input.width in front of the AZ FFT).
module sdf_fft_lane_natural #(
    parameter integer LOGN = 12,
    parameter integer INVERSE = 0,
    parameter [11:0] SCALE_MASK = 12'hfff,
    parameter [3:0] CORE_ID = 0,
    // 0: every launched vector owns one of the two reorder banks until its
    //    natural-order EOF retires.  This mode is safe for arbitrary m_ready
    //    stalls, but it cannot promise gap-free launches indefinitely.
    // 1: the downstream wrapper has reserved a complete, non-stalling output
    //    window.  m_ready must remain asserted whenever m_valid is asserted;
    //    violating that contract is reported through protocol_error.
    parameter integer OUTPUT_VECTOR_RESERVED = 0,
    // component width inside the lane (core DATA_W, bank DATA_W = 2*W)
    parameter integer W = 11,
    // component width the input container may carry; IN_W > W clamps the input to W bits
    parameter integer IN_W = 11,
    // entry limit width; 0 (or >= W) = no limit beyond W
    parameter integer ENTRY_W = 0,
    // rpr_noroll: spectrum centring by addressing instead of (-1)^n rolls.
    // OUT_SHIFT = 1: the output is the fftshift of the transform (P1, P3);
    // IN_SHIFT  = 1: the core is fed the ifftshift of each input vector (P6).
    parameter integer OUT_SHIFT = 0,
    parameter integer IN_SHIFT  = 0
) (
    input  wire                         clk,
    input  wire                         rst,
    input  wire                         clear,
    input  wire [63:0]                  s_data,
    input  wire                         s_valid,
    output wire                         s_ready,
    input  wire                         s_sof,
    input  wire                         s_eof,
    output wire [63:0]                  m_data,
    output wire                         m_valid,
    input  wire                         m_ready,
    output wire                         m_sof,
    output wire                         m_eof,
    output wire                         busy,
    output wire                         overflow_sticky,
    output wire [LOGN-1:0]              stage_overflow,
    output wire [LOGN*12-1:0]           debug_delay_addr,
    output wire [LOGN*12-1:0]           debug_twiddle_addr,
    output wire [LOGN-1:0]              debug_stage_valid,
    output wire [3:0]                   core_id,
    output wire                         input_sat_re,      // accepted input word whose re / im was clamped
    output wire                         input_sat_im,
    output wire                         protocol_error
);
    localparam integer DW = 2 * W;
    localparam integer EW = (ENTRY_W > 0 && ENTRY_W < W) ? ENTRY_W : W;

    wire [DW-1:0] pf_in;
    wire [DW-1:0] prefetched_data;
    wire prefetched_valid,prefetched_sof,prefetched_eof;
    wire [1:0] prefetch_banks;
    wire prefetch_occupied;
    wire prefetch_protocol,prefetch_overflow;
    wire core_ready;
    wire core_valid,core_sof,core_eof;
    wire signed [W-1:0] core_re,core_im;
    wire core_busy,core_protocol;
    wire reorder_ready,reorder_protocol,reorder_overflow;
    wire reorder_occupied;
    reg stream_contract_error;
    reg [1:0] reorder_credits_used;
    reg restart_blocked;

    wire reorder_retire = m_valid && m_ready && m_eof;
    wire reorder_credit_available = (reorder_credits_used < 2) || reorder_retire;
    wire output_window_available = (OUTPUT_VECTOR_RESERVED != 0) ||
                                   reorder_credit_available;
    wire launch_enable = core_ready && output_window_available &&
                         !restart_blocked;
    wire vector_launch = prefetched_valid && prefetched_sof && core_ready;

    wire in_fire = s_valid && s_ready;
    wire sat_re_w, sat_im_w;
    wire [2*EW-1:0] entry_pair;
    pair_narrow #(.SRC_W(IN_W), .W(EW)) u_narrow (
        .clk(clk), .valid(in_fire), .din(s_data), .dout(entry_pair),
        .sat_re(sat_re_w), .sat_im(sat_im_w));
    generate
        if (EW < W) begin : g_entry_sext
            assign pf_in = {{(W-EW){entry_pair[2*EW-1]}}, entry_pair[2*EW-1:EW],
                            {(W-EW){entry_pair[EW-1]}}, entry_pair[EW-1:0]};
        end else begin : g_entry_pass
            assign pf_in = entry_pair;
        end
    endgenerate
    assign input_sat_re = in_fire && sat_re_w;
    assign input_sat_im = in_fire && sat_im_w;

    vector_prefetch_pingpong #(.LOGN(LOGN),.DATA_W(DW),.IN_SHIFT(IN_SHIFT)) u_prefetch(
        .clk(clk),.rst(rst),.clear(clear),.s_data(pf_in),.s_valid(s_valid),
        .s_ready(s_ready),.s_sof(s_sof),.s_eof(s_eof),
        .launch_enable(launch_enable),.m_data(prefetched_data),
        .m_valid(prefetched_valid),.m_sof(prefetched_sof),.m_eof(prefetched_eof),
        .bank_full(prefetch_banks),.occupied(prefetch_occupied),
        .protocol_error(prefetch_protocol),
        .overflow_error(prefetch_overflow));

    sdf_fft_core #(.LOGN(LOGN),.DATA_W(W),.INVERSE(INVERSE),
        .SCALE_MASK(SCALE_MASK),.CORE_ID(CORE_ID)) u_core(
        .clk(clk),.rst(rst),.clear(clear),.s_valid(prefetched_valid),
        .s_ready(core_ready),.s_sof(prefetched_sof),.s_eof(prefetched_eof),
        .s_re(prefetched_data[W-1:0]),.s_im(prefetched_data[DW-1:W]),
        .m_valid(core_valid),.m_sof(core_sof),.m_eof(core_eof),
        .m_re(core_re),.m_im(core_im),.busy(core_busy),
        .protocol_error(core_protocol),.overflow_sticky(overflow_sticky),
        .stage_overflow(stage_overflow),.debug_delay_addr(debug_delay_addr),
        .debug_twiddle_addr(debug_twiddle_addr),
        .debug_stage_valid(debug_stage_valid),.core_id(core_id));

    wire [DW-1:0] reorder_data;
    bitrev_pingpong #(.LOGN(LOGN),.DATA_W(DW),.OUT_SHIFT(OUT_SHIFT)) u_reorder(
        .clk(clk),.rst(rst),.clear(clear),.s_data({core_im,core_re}),
        .s_valid(core_valid),.s_ready(reorder_ready),.s_sof(core_sof),
        .s_eof(core_eof),.m_data(reorder_data),.m_valid(m_valid),.m_ready(m_ready),
        .m_sof(m_sof),.m_eof(m_eof),.protocol_error(reorder_protocol),
        .overflow_error(reorder_overflow),.occupied(reorder_occupied));
    pair_sext #(.W(W)) u_sext (.din(reorder_data), .dout(m_data));

    always @(posedge clk) begin
        if (rst || clear) begin
            stream_contract_error <= 1'b0;
            reorder_credits_used <= 2'd0;
            restart_blocked <= 1'b0;
        end else begin
            if ((prefetched_valid && !core_ready) ||
                (core_valid && !reorder_ready) ||
                ((OUTPUT_VECTOR_RESERVED != 0) && m_valid && !m_ready))
                stream_contract_error <= 1'b1;

            // Once an input EOF is accepted without an already-streaming next
            // vector, the SDF core will enter its flush sequence on the next
            // clock.  Do not start a bank during that one-cycle grace window.
            if (!core_ready)
                restart_blocked <= 1'b0;
            else if (vector_launch)
                restart_blocked <= 1'b0;
            else if (prefetched_valid && prefetched_eof)
                restart_blocked <= 1'b1;

            if (OUTPUT_VECTOR_RESERVED != 0) begin
                // Capacity is guaranteed by the external vector reservation,
                // not inferred optimistically from the instantaneous ready.
                reorder_credits_used <= 2'd0;
            end else begin
                case ({vector_launch, reorder_retire})
                    2'b10: begin
                        if (reorder_credits_used < 2)
                            reorder_credits_used <= reorder_credits_used + 1'b1;
                        else
                            stream_contract_error <= 1'b1;
                    end
                    2'b01: begin
                        if (reorder_credits_used != 0)
                            reorder_credits_used <= reorder_credits_used - 1'b1;
                        else
                            stream_contract_error <= 1'b1;
                    end
                    default: begin
                        reorder_credits_used <= reorder_credits_used;
                    end
                endcase
            end
        end
    end

    assign busy = prefetch_occupied || core_busy || reorder_occupied ||
                  (reorder_credits_used != 0);
    assign protocol_error = prefetch_protocol || prefetch_overflow ||
                            core_protocol || reorder_protocol ||
                            reorder_overflow || stream_contract_error;
endmodule
