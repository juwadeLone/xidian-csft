`timescale 1ns/1ps

// GR17 reduced product replica: the GR16S rp_phase_product16 structure at
// 17 bits and a per-product coefficient width KC (Q1.(KC-1), the table of
// gr17_model.rep_coef_table).  The data operand is the top KW = K bits of
// the aligned 17-bit main word (x_t = floor(x / 2^(17-K)), trunc).
// The product is LUT-only (rp_cmul_lut, use_dsp = "no" throughout).
// Output word: sat_K((x_t*c -/+ y_t*c + 2^(KC-2)) >>> (KC-1)) on a K+2-bit
// container -- gr17_model.rep_product.
module rp_phase_product17 #(
    parameter integer K  = 10,          // product replica width
    parameter integer KC = 12           // coefficient width (Q1.(KC-1))
) (
    input wire clk,
    input wire ce,
    input wire signed [16:0] x_re,
    input wire signed [16:0] x_im,
    input wire signed [KC-1:0] c_re,
    input wire signed [KC-1:0] c_im,
    output wire signed [K+1:0] r_re,
    output wire signed [K+1:0] r_im,
    output wire [3*(K+2)-1:0] r_re_d,
    output wire [3*(K+2)-1:0] r_im_d
);
`include "grp17_defs.vh"
    localparam integer KW = grp_pkw17(K);
    localparam integer OW = KW + KC + 1;
    initial if (K < 4 || K > 16 || KC < 8 || KC > 24)
        $fatal(1, "unsupported phase-product width K=%0d KC=%0d", K, KC);
    wire signed [OW-1:0] prod_re, prod_im;
    (* keep_hierarchy = "yes", dont_touch = "true" *) rp_cmul_lut #(
        .KA(KW), .KT(KC), .MODE(grp_pmode17(K, KC)), .OW(OW)
    ) u_mul (
        .clk(clk), .ce(ce), .pr(x_re[16 -: KW]), .pi(x_im[16 -: KW]),
        .wr(c_re), .wi(c_im), .re(prod_re), .im(prod_im));
    genvar dest;
    generate
        for (dest = 0; dest < 3; dest = dest + 1) begin : g_copy
            (* keep_hierarchy = "yes", dont_touch = "true" *) iso_phase_product_output17 #(
                .K(K), .KC(KC), .OW(OW)
            ) u_out (
                .clk(clk), .ce(ce), .prod_re(prod_re), .prod_im(prod_im),
                .r_re(r_re_d[dest*(K+2) +: K+2]), .r_im(r_im_d[dest*(K+2) +: K+2]));
        end
    endgenerate
    assign r_re = r_re_d[0 +: K+2];
    assign r_im = r_im_d[0 +: K+2];
endmodule

module iso_phase_product_output17 #(
    parameter integer K  = 10,
    parameter integer KC = 12,
    parameter integer OW = K + KC + 1
) (
    input wire clk,
    input wire ce,
    input wire signed [OW-1:0] prod_re,
    input wire signed [OW-1:0] prod_im,
    output reg signed [K+1:0] r_re,
    output reg signed [K+1:0] r_im
);
    localparam signed [OW-1:0] HALF = 1 << (KC-2);
    localparam signed [OW-1:0] HI = (1 << (K-1))-1;
    localparam signed [OW-1:0] LO = -(1 << (K-1));
    wire signed [OW-1:0] shifted_re = (prod_re + HALF) >>> (KC-1);
    wire signed [OW-1:0] shifted_im = (prod_im + HALF) >>> (KC-1);
    always @(posedge clk) begin
        if (ce) begin
            r_re <= (shifted_re > HI) ? HI : (shifted_re < LO) ? LO : shifted_re;
            r_im <= (shifted_im > HI) ? HI : (shifted_im < LO) ? LO : shifted_im;
        end
    end
endmodule
