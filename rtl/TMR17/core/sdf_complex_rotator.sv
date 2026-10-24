`timescale 1ns/1ps

// Twiddle format is set once by the generated header (scripts/gen_twiddles.py).
`include "sdf_twiddle12_params.vh"

// Output = saturate_OUT_W(((acc + HALF) >>> TW_FRAC) >>> scale) per component, with HALF = 2^(TW_FRAC-1) for
// ROUND = 1 (half_up, ties toward +infinity) and 0 for ROUND = 0 (floor).  The overflow flags are per component.
module sdf_complex_rotator #(
    parameter integer IN_W = 12,
    parameter integer OUT_W = 11,
    parameter integer TW_W = `SDF_TW_W,
    parameter integer TW_FRAC = `SDF_TW_FRAC,
    parameter integer ROUND = 1
) (
    input  wire                         clk,
    input  wire                         rst,
    input  wire                         clear,
    input  wire                         ce,
    input  wire                         i_valid,
    input  wire                         i_sof,
    input  wire                         i_eof,
    input  wire                         i_do_twiddle,
    input  wire                         i_exact_quarter,
    input  wire                         i_scale,
    input  wire signed [IN_W-1:0]       i_re,
    input  wire signed [IN_W-1:0]       i_im,
    input  wire signed [TW_W-1:0]       i_wr,
    input  wire signed [TW_W-1:0]       i_wi,
    output reg                          o_valid,
    output reg                          o_sof,
    output reg                          o_eof,
    output reg signed [OUT_W-1:0]       o_re,
    output reg signed [OUT_W-1:0]       o_im,
    output reg                          o_sample_overflow,
    output reg                          o_overflow_re,
    output reg                          o_overflow_im,
    output reg                          o_overflow_sticky
);

    localparam integer PROD_W = IN_W + TW_W;
    localparam integer ACC_W = PROD_W + 1;
    localparam integer RND_W = ACC_W + 1;
    localparam [RND_W-1:0] HALF = (ROUND != 0) ? ({{(RND_W-1){1'b0}}, 1'b1} << (TW_FRAC - 1)) : {RND_W{1'b0}};

    reg signed [IN_W-1:0] x_re_0, x_im_0;
    reg signed [TW_W-1:0] w_re_0, w_im_0;
    reg do_twiddle_0, exact_quarter_0, quarter_negative_0;
    reg scale_0, valid_0, sof_0, eof_0;

    reg signed [PROD_W-1:0] rr_1, ii_1, ri_1, ir_1;
    reg signed [ACC_W-1:0] bypass_re_1, bypass_im_1;
    reg signed [ACC_W-1:0] quarter_re_1, quarter_im_1;
    reg do_twiddle_1, exact_quarter_1, scale_1, valid_1, sof_1, eof_1;

    reg signed [ACC_W-1:0] acc_re_2, acc_im_2;
    reg scale_2, valid_2, sof_2, eof_2;

    wire signed [ACC_W-1:0] rr_ext = {{(ACC_W-PROD_W){rr_1[PROD_W-1]}}, rr_1};
    wire signed [ACC_W-1:0] ii_ext = {{(ACC_W-PROD_W){ii_1[PROD_W-1]}}, ii_1};
    wire signed [ACC_W-1:0] ri_ext = {{(ACC_W-PROD_W){ri_1[PROD_W-1]}}, ri_1};
    wire signed [ACC_W-1:0] ir_ext = {{(ACC_W-PROD_W){ir_1[PROD_W-1]}}, ir_1};

    wire signed [RND_W-1:0] rounded_re = {acc_re_2[ACC_W-1], acc_re_2} + HALF;
    wire signed [RND_W-1:0] rounded_im = {acc_im_2[ACC_W-1], acc_im_2} + HALF;
    wire signed [RND_W-1:0] shifted_re = rounded_re >>> (TW_FRAC + (scale_2 ? 1 : 0));
    wire signed [RND_W-1:0] shifted_im = rounded_im >>> (TW_FRAC + (scale_2 ? 1 : 0));
    wire overflow_re = shifted_re[RND_W-1:OUT_W-1] != {(RND_W-OUT_W+1){shifted_re[OUT_W-1]}};
    wire overflow_im = shifted_im[RND_W-1:OUT_W-1] != {(RND_W-OUT_W+1){shifted_im[OUT_W-1]}};
    wire sample_overflow_re = valid_2 && overflow_re;
    wire sample_overflow_im = valid_2 && overflow_im;
    wire signed [OUT_W-1:0] sat_re = !overflow_re ? shifted_re[OUT_W-1:0] :
        (shifted_re[RND_W-1] ? {1'b1, {(OUT_W-1){1'b0}}} : {1'b0, {(OUT_W-1){1'b1}}});
    wire signed [OUT_W-1:0] sat_im = !overflow_im ? shifted_im[OUT_W-1:0] :
        (shifted_im[RND_W-1] ? {1'b1, {(OUT_W-1){1'b0}}} : {1'b0, {(OUT_W-1){1'b1}}});

    wire signed [ACC_W-1:0] x_re_ext = {{(ACC_W-IN_W){x_re_0[IN_W-1]}}, x_re_0};
    wire signed [ACC_W-1:0] x_im_ext = {{(ACC_W-IN_W){x_im_0[IN_W-1]}}, x_im_0};

    always @(posedge clk) begin
        if (rst || clear) begin
            x_re_0 <= '0;
            x_im_0 <= '0;
            w_re_0 <= '0;
            w_im_0 <= '0;
            do_twiddle_0 <= 1'b0;
            exact_quarter_0 <= 1'b0;
            quarter_negative_0 <= 1'b0;
            scale_0 <= 1'b0;
            valid_0 <= 1'b0;
            sof_0 <= 1'b0;
            eof_0 <= 1'b0;
            rr_1 <= '0;
            ii_1 <= '0;
            ri_1 <= '0;
            ir_1 <= '0;
            bypass_re_1 <= '0;
            bypass_im_1 <= '0;
            quarter_re_1 <= '0;
            quarter_im_1 <= '0;
            do_twiddle_1 <= 1'b0;
            exact_quarter_1 <= 1'b0;
            scale_1 <= 1'b0;
            valid_1 <= 1'b0;
            sof_1 <= 1'b0;
            eof_1 <= 1'b0;
            acc_re_2 <= '0;
            acc_im_2 <= '0;
            scale_2 <= 1'b0;
            valid_2 <= 1'b0;
            sof_2 <= 1'b0;
            eof_2 <= 1'b0;
            o_valid <= 1'b0;
            o_sof <= 1'b0;
            o_eof <= 1'b0;
            o_re <= '0;
            o_im <= '0;
            o_sample_overflow <= 1'b0;
            o_overflow_re <= 1'b0;
            o_overflow_im <= 1'b0;
            o_overflow_sticky <= 1'b0;
        end else if (ce) begin
            x_re_0 <= i_re;
            x_im_0 <= i_im;
            w_re_0 <= i_wr;
            w_im_0 <= i_wi;
            do_twiddle_0 <= i_do_twiddle;
            exact_quarter_0 <= i_exact_quarter;
            quarter_negative_0 <= i_wi[TW_W-1];
            scale_0 <= i_scale;
            valid_0 <= i_valid;
            sof_0 <= i_sof;
            eof_0 <= i_eof;

            rr_1 <= x_re_0 * w_re_0;
            ii_1 <= x_im_0 * w_im_0;
            ri_1 <= x_re_0 * w_im_0;
            ir_1 <= x_im_0 * w_re_0;
            bypass_re_1 <= x_re_ext <<< TW_FRAC;
            bypass_im_1 <= x_im_ext <<< TW_FRAC;
            if (quarter_negative_0) begin
                quarter_re_1 <= x_im_ext <<< TW_FRAC;
                quarter_im_1 <= (-x_re_ext) <<< TW_FRAC;
            end else begin
                quarter_re_1 <= (-x_im_ext) <<< TW_FRAC;
                quarter_im_1 <= x_re_ext <<< TW_FRAC;
            end
            do_twiddle_1 <= do_twiddle_0;
            exact_quarter_1 <= exact_quarter_0;
            scale_1 <= scale_0;
            valid_1 <= valid_0;
            sof_1 <= sof_0;
            eof_1 <= eof_0;

            acc_re_2 <= exact_quarter_1 ? quarter_re_1 :
                        (do_twiddle_1 ? rr_ext - ii_ext : bypass_re_1);
            acc_im_2 <= exact_quarter_1 ? quarter_im_1 :
                        (do_twiddle_1 ? ri_ext + ir_ext : bypass_im_1);
            scale_2 <= scale_1;
            valid_2 <= valid_1;
            sof_2 <= sof_1;
            eof_2 <= eof_1;

            o_re <= sat_re;
            o_im <= sat_im;
            o_valid <= valid_2;
            o_sof <= sof_2;
            o_eof <= eof_2;
            o_sample_overflow <= sample_overflow_re || sample_overflow_im;
            o_overflow_re <= sample_overflow_re;
            o_overflow_im <= sample_overflow_im;
            if (sample_overflow_re || sample_overflow_im)
                o_overflow_sticky <= 1'b1;
        end
    end

endmodule
