`timescale 1ns/1ps

// Twiddle format is set once by the generated header (scripts/gen_twiddles.py).
`include "sdf_twiddle12_params.vh"

module sdf_stage #(
    parameter integer LOGN = 11,
    parameter integer STAGE = 0,
    parameter integer MAX_LOGN = 11,
    parameter integer DATA_W = 11,
    parameter integer INVERSE = 0,
    parameter integer SCALE = 0
) (
    input  wire                         clk,
    input  wire                         rst,
    input  wire                         clear,
    input  wire                         ce,
    input  wire                         i_valid,
    input  wire                         i_sof,
    input  wire                         i_eof,
    input  wire signed [DATA_W-1:0]     i_re,
    input  wire signed [DATA_W-1:0]     i_im,
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
    localparam integer PHASE_W = LOGN-STAGE;
    localparam integer FB_W = DATA_W + 1;
    localparam integer TWIDDLE_STAGE_ID = STAGE + (MAX_LOGN-LOGN);

    reg [PHASE_W-1:0] phase;
    reg active_q;
    wire local_ce = ce && (active_q || i_valid);
    wire first_half = !phase[PHASE_W-1];
    wire [ADDR_W-1:0] phase_addr = (DEPTH <= 1) ? {ADDR_W{1'b0}} : phase[ADDR_W-1:0];

    reg signed [DATA_W-1:0] x_re_d, x_im_d;
    reg first_half_d;
    reg [ADDR_W-1:0] addr_d;
    reg ctrl_valid_d;

    reg [DEPTH:0] valid_delay;
    reg [DEPTH:0] sof_delay;
    reg [DEPTH:0] eof_delay;

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
            // A short delay line is data in flight: a shift register of
            // DEPTH taps gives the same DEPTH-beat delay as the addressed
            // RAM (feedback on the beat after edge t is write(t+1-DEPTH)).
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
            // One inferred RAM holds re and im side by side, 2*FB_W bits per
            // word.  The word read at phase_addr is on feedback one ce edge
            // later; the write goes to addr_d on the same edge.
            reg [2*FB_W-1:0] mem [0:DEPTH-1];
            reg [2*FB_W-1:0] rd_q;
            always @(posedge clk) begin
                if (local_ce) begin
                    if (ctrl_valid_d)
                        mem[addr_d] <= {write_im, write_re};
                    rd_q <= mem[phase_addr];
                end
            end
            assign feedback_re = $signed(rd_q[FB_W-1:0]);
            assign feedback_im = $signed(rd_q[2*FB_W-1:FB_W]);
        end
    endgenerate

    always @(posedge clk) begin
        if (rst || clear) begin
            phase <= '0;
            active_q <= 1'b0;
            x_re_d <= '0;
            x_im_d <= '0;
            first_half_d <= 1'b0;
            addr_d <= '0;
            ctrl_valid_d <= 1'b0;
            valid_delay <= '0;
            sof_delay <= '0;
            eof_delay <= '0;
        end else if (local_ce) begin
            active_q <= 1'b1;
            phase <= phase + {{(PHASE_W-1){1'b0}}, 1'b1};
            x_re_d <= i_re;
            x_im_d <= i_im;
            first_half_d <= first_half;
            addr_d <= phase_addr;
            ctrl_valid_d <= 1'b1;
            valid_delay <= {valid_delay[DEPTH-1:0], i_valid};
            sof_delay <= {sof_delay[DEPTH-1:0], i_sof};
            eof_delay <= {eof_delay[DEPTH-1:0], i_eof};
        end
    end

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
        .addr(addr_d),
        .wr(tw_wr),
        .wi(tw_wi)
    );

    reg signed [FB_W-1:0] pre_re_tw_d, pre_im_tw_d;
    reg do_twiddle_tw_d, unity_tw_d, quarter_tw_d;
    reg valid_tw_d, sof_tw_d, eof_tw_d;
    always @(posedge clk) begin
        if (rst || clear) begin
            pre_re_tw_d <= '0;
            pre_im_tw_d <= '0;
            do_twiddle_tw_d <= 1'b0;
            unity_tw_d <= 1'b0;
            quarter_tw_d <= 1'b0;
            valid_tw_d <= 1'b0;
            sof_tw_d <= 1'b0;
            eof_tw_d <= 1'b0;
        end else if (local_ce) begin
            pre_re_tw_d <= pre_re;
            pre_im_tw_d <= pre_im;
            do_twiddle_tw_d <= first_half_d;
            unity_tw_d <= (addr_d == 0);
            quarter_tw_d <= (DEPTH > 2) && (addr_d == (DEPTH >> 1));
            valid_tw_d <= valid_delay[DEPTH];
            sof_tw_d <= sof_delay[DEPTH];
            eof_tw_d <= eof_delay[DEPTH];
        end
    end

    // The rotator output is the stage output.  A sample is presented from one
    // ce edge to the next and consumed on the ce-high cycle, so beat = valid
    // && ce counts it once.
    generate
        if (DEPTH <= 2) begin : g_trivial_rotator
            sdf_trivial_rotator #(
                .IN_W(FB_W),
                .OUT_W(DATA_W)
            ) u_rotator (
                .clk(clk),.rst(rst),.clear(clear),.ce(local_ce),
                .i_valid(valid_tw_d),.i_sof(sof_tw_d),.i_eof(eof_tw_d),
                .i_do_twiddle(do_twiddle_tw_d),.i_scale(SCALE != 0),
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
                .i_do_twiddle(do_twiddle_tw_d && !unity_tw_d),
                .i_exact_quarter(do_twiddle_tw_d && quarter_tw_d),
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

    assign o_debug_delay_addr = {{(12-ADDR_W){1'b0}}, addr_d};
    assign o_debug_twiddle_addr = {{(12-ADDR_W){1'b0}}, addr_d};

endmodule
