`timescale 1ns/1ps

// Twiddle format is set once by the generated header (scripts/gen_twiddles.py).
`include "sdf_twiddle12_params.vh"

// GR16S control-estimation variant: the W16R2 sdf_stage datapath (input
// register, feedback delay line, twiddle ROM, operand registers, rotator)
// with ALL control state pulled out into grp_ctl.  Every input below that
// names a control signal (ce, phase_addr, first_half_d, addr_d,
// ctrl_valid_d, do_tw, unity, quarter, *_tw_d) is produced by a control
// copy; this module contains no timing state of its own beyond the
// datapath registers and the rotator's internal pipeline (which belongs to
// the datapath: it only replays the control taps it is fed).
//
// The body is a byte-for-byte extract of sdf_stage_grp's datapath section so
// the arithmetic stays identical to GR16S / W16R2.
module sdf_stage_dp #(
    parameter integer LOGN = 11,
    parameter integer STAGE = 0,
    parameter integer MAX_LOGN = 11,
    parameter integer DATA_W = 16,
    parameter integer INVERSE = 0,
    parameter integer SCALE = 0
) (
    input  wire                         clk,
    input  wire                         rst,
    input  wire                         clear,
    input  wire                         ce,        // local_ce from grp_ctl
    input  wire signed [DATA_W-1:0]     i_re,
    input  wire signed [DATA_W-1:0]     i_im,
    // control bundle from grp_ctl
    input  wire [10:0]                  phase_addr,
    input  wire                         first_half_d,
    input  wire [10:0]                  addr_d,
    input  wire                         ctrl_valid_d,
    input  wire                         do_tw,
    input  wire                         unity,
    input  wire                         quarter,
    input  wire                         valid_tw_d,
    input  wire                         sof_tw_d,
    input  wire                         eof_tw_d,
    output wire                         o_valid,
    output wire                         o_sof,
    output wire                         o_eof,
    output wire signed [DATA_W-1:0]     o_re,
    output wire signed [DATA_W-1:0]     o_im,
    output wire                         o_sample_overflow,
    output wire                         o_overflow_re,
    output wire                         o_overflow_im,
    output wire                         o_overflow_sticky,
    output wire [11:0]                  o_debug_delay_addr,
    output wire [11:0]                  o_debug_twiddle_addr
);
    localparam integer DEPTH = 1 << (LOGN-STAGE-1);
    localparam integer ADDR_W = (DEPTH <= 1) ? 1 : $clog2(DEPTH);
    localparam integer FB_W = DATA_W + 1;
    localparam integer TWIDDLE_STAGE_ID = STAGE + (MAX_LOGN-LOGN);
    wire local_ce = ce;

    reg signed [DATA_W-1:0] x_re_d, x_im_d;
    always @(posedge clk) begin
        if (rst || clear) begin
            x_re_d <= '0;
            x_im_d <= '0;
        end else if (local_ce) begin
            x_re_d <= i_re;
            x_im_d <= i_im;
        end
    end

    wire signed [FB_W-1:0] x_re_d_ext = {x_re_d[DATA_W-1], x_re_d};
    wire signed [FB_W-1:0] x_im_d_ext = {x_im_d[DATA_W-1], x_im_d};

    wire signed [FB_W-1:0] feedback_re;
    wire signed [FB_W-1:0] feedback_im;
    wire signed [FB_W-1:0] write_re = first_half_d ? x_re_d_ext : feedback_re - x_re_d_ext;
    wire signed [FB_W-1:0] write_im = first_half_d ? x_im_d_ext : feedback_im - x_im_d_ext;
    wire signed [FB_W-1:0] pre_re = first_half_d ? feedback_re : feedback_re + x_re_d_ext;
    wire signed [FB_W-1:0] pre_im = first_half_d ? feedback_im : feedback_im + x_im_d_ext;

    generate
        if (DEPTH == 1) begin : g_feedback_register
            reg signed [FB_W-1:0] feedback_re_q;
            reg signed [FB_W-1:0] feedback_im_q;
            assign feedback_re = feedback_re_q;
            assign feedback_im = feedback_im_q;
            always @(posedge clk) begin
                if (local_ce && ctrl_valid_d) begin
                    feedback_re_q <= write_re;
                    feedback_im_q <= write_im;
                end
            end
        end else if (DEPTH <= 32) begin : g_feedback_srl
            reg signed [FB_W-1:0] sr_re [0:DEPTH-1];
            reg signed [FB_W-1:0] sr_im [0:DEPTH-1];
            integer k;
            always @(posedge clk) begin
                if (local_ce) begin
                    sr_re[0] <= write_re;
                    sr_im[0] <= write_im;
                    for (k = 1; k < DEPTH; k = k + 1) begin
                        sr_re[k] <= sr_re[k-1];
                        sr_im[k] <= sr_im[k-1];
                    end
                end
            end
            assign feedback_re = sr_re[DEPTH-1];
            assign feedback_im = sr_im[DEPTH-1];
        end else begin : g_feedback_ram
            reg [2*FB_W-1:0] mem [0:DEPTH-1];
            reg [2*FB_W-1:0] rd_q;
            always @(posedge clk) begin
                if (local_ce) begin
                    if (ctrl_valid_d)
                        mem[addr_d[ADDR_W-1:0]] <= {write_im, write_re};
                    rd_q <= mem[phase_addr[ADDR_W-1:0]];
                end
            end
            assign feedback_re = $signed(rd_q[FB_W-1:0]);
            assign feedback_im = $signed(rd_q[2*FB_W-1:FB_W]);
        end
    endgenerate

    wire signed [`SDF_TW_W-1:0] tw_wr;
    wire signed [`SDF_TW_W-1:0] tw_wi;
    sdf_twiddle_rom #(
        .STAGE_ID(TWIDDLE_STAGE_ID),
        .DEPTH(DEPTH),
        .ADDR_W(ADDR_W),
        .INVERSE(INVERSE)
    ) u_twiddle_rom (
        .clk(clk),
        .ce(local_ce),
        .addr(addr_d[ADDR_W-1:0]),
        .wr(tw_wr),
        .wi(tw_wi)
    );

    reg signed [FB_W-1:0] pre_re_tw_d, pre_im_tw_d;
    always @(posedge clk) begin
        if (rst || clear) begin
            pre_re_tw_d <= '0;
            pre_im_tw_d <= '0;
        end else if (local_ce) begin
            pre_re_tw_d <= pre_re;
            pre_im_tw_d <= pre_im;
        end
    end

    // The rotator output is the stage output.  Its tick ports are driven by
    // the control copy's *_tw_d taps; its own tick pipeline is datapath
    // state replaying those taps.
    generate
        if (DEPTH <= 2) begin : g_trivial_rotator
            sdf_trivial_rotator #(
                .IN_W(FB_W),
                .OUT_W(DATA_W)
            ) u_rotator (
                .clk(clk),.rst(rst),.clear(clear),.ce(local_ce),
                .i_valid(valid_tw_d),.i_sof(sof_tw_d),.i_eof(eof_tw_d),
                .i_do_twiddle(do_tw),.i_scale(SCALE != 0),
                .i_re(pre_re_tw_d),.i_im(pre_im_tw_d),.i_wi(tw_wi),
                .o_valid(o_valid),.o_sof(o_sof),.o_eof(o_eof),
                .o_re(o_re),.o_im(o_im),
                .o_sample_overflow(o_sample_overflow),
                .o_overflow_re(o_overflow_re),
                .o_overflow_im(o_overflow_im),
                .o_overflow_sticky(o_overflow_sticky));
        end else begin : g_general_rotator
            sdf_complex_rotator #(
                .IN_W(FB_W),
                .OUT_W(DATA_W),
                .TW_W(`SDF_TW_W),
                .TW_FRAC(`SDF_TW_FRAC),
                .ROUND(1)
            ) u_rotator (
                .clk(clk),.rst(rst),.clear(clear),.ce(local_ce),
                .i_valid(valid_tw_d),.i_sof(sof_tw_d),.i_eof(eof_tw_d),
                .i_do_twiddle(do_tw && !unity),
                .i_exact_quarter(do_tw && quarter),
                .i_scale(SCALE != 0),
                .i_re(pre_re_tw_d),.i_im(pre_im_tw_d),
                .i_wr(tw_wr),.i_wi(tw_wi),
                .o_valid(o_valid),.o_sof(o_sof),.o_eof(o_eof),
                .o_re(o_re),.o_im(o_im),
                .o_sample_overflow(o_sample_overflow),
                .o_overflow_re(o_overflow_re),
                .o_overflow_im(o_overflow_im),
                .o_overflow_sticky(o_overflow_sticky));
        end
    endgenerate

    assign o_debug_delay_addr = addr_d;
    assign o_debug_twiddle_addr = addr_d;
endmodule
