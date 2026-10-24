`timescale 1ns/1ps

// Twiddle width comes from the generated header; i_wi's sign bit selects the
// quarter-turn direction, so it must track TW_W rather than assume 30.
`include "sdf_twiddle12_params.vh"

// Multiplier-free unit and quarter-turn rotations for the final two Radix-2
// DIF stages.  Latency matches sdf_complex_rotator so stage composition and
// frame tags remain identical.  The scaled value saturates to OUT_W per
// component, like the complex rotator.
module sdf_trivial_rotator #(
    parameter integer IN_W = 12,
    parameter integer OUT_W = 11,
    parameter integer TW_W = `SDF_TW_W
) (
    input  wire                         clk,
    input  wire                         rst,
    input  wire                         clear,
    input  wire                         ce,
    input  wire                         i_valid,
    input  wire                         i_sof,
    input  wire                         i_eof,
    input  wire                         i_do_twiddle,
    input  wire                         i_scale,
    input  wire signed [IN_W-1:0]       i_re,
    input  wire signed [IN_W-1:0]       i_im,
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
    localparam integer EXT_W = IN_W+1;
    wire signed [EXT_W-1:0] i_re_ext={{(EXT_W-IN_W){i_re[IN_W-1]}},i_re};
    wire signed [EXT_W-1:0] i_im_ext={{(EXT_W-IN_W){i_im[IN_W-1]}},i_im};
    reg signed [EXT_W-1:0] selected_re_0,selected_im_0;
    reg scale_0,valid_0,sof_0,eof_0;
    reg signed [EXT_W-1:0] selected_re_1,selected_im_1;
    reg scale_1,valid_1,sof_1,eof_1;
    reg signed [OUT_W-1:0] narrowed_re_2,narrowed_im_2;
    reg overflow_re_2,overflow_im_2,valid_2,sof_2,eof_2;

    wire signed [EXT_W-1:0] rotated_re =
        (!i_do_twiddle || i_wi == 0) ? i_re_ext :
        i_wi[TW_W-1] ? i_im_ext : -i_im_ext;
    wire signed [EXT_W-1:0] rotated_im =
        (!i_do_twiddle || i_wi == 0) ? i_im_ext :
        i_wi[TW_W-1] ? -i_re_ext : i_re_ext;
    wire signed [EXT_W-1:0] shifted_re = selected_re_1 >>> (scale_1 ? 1 : 0);
    wire signed [EXT_W-1:0] shifted_im = selected_im_1 >>> (scale_1 ? 1 : 0);
    wire overflow_re = shifted_re[EXT_W-1:OUT_W-1] !=
                       {(EXT_W-OUT_W+1){shifted_re[OUT_W-1]}};
    wire overflow_im = shifted_im[EXT_W-1:OUT_W-1] !=
                       {(EXT_W-OUT_W+1){shifted_im[OUT_W-1]}};
    wire signed [OUT_W-1:0] sat_re = !overflow_re ? shifted_re[OUT_W-1:0] :
        (shifted_re[EXT_W-1] ? {1'b1, {(OUT_W-1){1'b0}}} : {1'b0, {(OUT_W-1){1'b1}}});
    wire signed [OUT_W-1:0] sat_im = !overflow_im ? shifted_im[OUT_W-1:0] :
        (shifted_im[EXT_W-1] ? {1'b1, {(OUT_W-1){1'b0}}} : {1'b0, {(OUT_W-1){1'b1}}});

    always @(posedge clk) begin
        if (rst || clear) begin
            selected_re_0 <= '0;
            selected_im_0 <= '0;
            scale_0 <= 1'b0;
            valid_0 <= 1'b0;
            sof_0 <= 1'b0;
            eof_0 <= 1'b0;
            selected_re_1 <= '0;
            selected_im_1 <= '0;
            scale_1 <= 1'b0;
            valid_1 <= 1'b0;
            sof_1 <= 1'b0;
            eof_1 <= 1'b0;
            narrowed_re_2 <= '0;
            narrowed_im_2 <= '0;
            overflow_re_2 <= 1'b0;
            overflow_im_2 <= 1'b0;
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
            selected_re_0 <= rotated_re;
            selected_im_0 <= rotated_im;
            scale_0 <= i_scale;
            valid_0 <= i_valid;
            sof_0 <= i_sof;
            eof_0 <= i_eof;

            selected_re_1 <= selected_re_0;
            selected_im_1 <= selected_im_0;
            scale_1 <= scale_0;
            valid_1 <= valid_0;
            sof_1 <= sof_0;
            eof_1 <= eof_0;

            narrowed_re_2 <= sat_re;
            narrowed_im_2 <= sat_im;
            overflow_re_2 <= valid_1 && overflow_re;
            overflow_im_2 <= valid_1 && overflow_im;
            valid_2 <= valid_1;
            sof_2 <= sof_1;
            eof_2 <= eof_1;

            o_re <= narrowed_re_2;
            o_im <= narrowed_im_2;
            o_valid <= valid_2;
            o_sof <= sof_2;
            o_eof <= eof_2;
            o_sample_overflow <= overflow_re_2 || overflow_im_2;
            o_overflow_re <= overflow_re_2;
            o_overflow_im <= overflow_im_2;
            if (overflow_re_2 || overflow_im_2)
                o_overflow_sticky <= 1'b1;
        end
    end
endmodule
