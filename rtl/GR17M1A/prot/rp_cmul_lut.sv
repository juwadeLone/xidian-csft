`timescale 1ns/1ps

// One reduced-precision replica of a complex multiply in DSP48E1:
//   re = pr*wr - pi*wi,   im = pr*wi + pi*wr
// on KA-bit data and KT-bit twiddle operands (fb16 port of rpr_base's
// rp_cmul_dsp; unchanged logic, new KA/KT ranges of the 16-bit chain:
// KA <= 18, KT <= 12).  Three structures (MODE):
//   0  packed, 2 DSP: two products sharing a multiplicand in one DSP through
//      the A/D pre-adder, (pr*2^S + pi)*wr = pr*wr*2^S + pi*wr, and
//      (pi*2^S + pr)*wi.  Needs 2*KA + KT <= 24 (the pre-adder output is 25
//      bits and keeps one guard bit).  The borrow of a negative low product
//      from the high one is not corrected; rpr_params.vh counts it in T.
//   1  3 DSP: k1 = wr*(pr+pi) (A/D pre-adder), k2 = pr*(wi-wr), k3 = pi*(wr+wi);
//      re = k1 - k3, im = k1 + k2.  The twiddle-side sums are in LUT (the B
//      port has no pre-adder) and must fit its 18 bits: KT <= 17.
//   2  4 DSP: the four products, KA <= 25, KT <= 18.
// The final additions are in LUT carry chains (use_dsp = "no").
//
// Timing: operands registered by the caller on edge a; this module registers
// on b (operands / pre-add), c (products), d (re, im).  All registers move
// on ce, like the FP pipeline it shadows.  No reset: the data are qualified
// by the FP pipeline's own valid.
//
// Instantiate every replica with DONT_TOUCH: two replicas with the same
// inputs are otherwise merged by synthesis.  RP1 and RP2 are separate
// instances, so they never share a DSP.
module rp_cmul_lut #(
    parameter integer KA   = 18,
    parameter integer KT   = 12,
    parameter integer MODE = 1,
    parameter integer OW   = KA + KT + 1
) (
    input  wire                 clk,
    input  wire                 ce,
    input  wire signed [KA-1:0] pr,
    input  wire signed [KA-1:0] pi,
    input  wire signed [KT-1:0] wr,
    input  wire signed [KT-1:0] wi,
    output wire signed [OW-1:0] re,
    output wire signed [OW-1:0] im
);
    // The final sums stay in LUT carry chains.  Without the attribute Vivado
    // put the packed replica's hi1 - hi2 into a third DSP's post-adder
    // (rpr_base OOC 2026-09-29: 3 DSP instead of 2).
    (* use_dsp = "no" *) reg signed [OW-1:0] re_q, im_q;
    assign re = re_q;
    assign im = im_q;
    generate
        if (MODE == 0) begin : g_pack
            localparam integer S  = KA + KT;
            localparam integer AW = KA + S + 1;
            localparam integer PW = AW + KT;
            (* use_dsp = "no" *) reg signed [AW-1:0] a1, a2;
            (* use_dsp = "no" *) reg signed [PW-1:0] m1, m2;
            reg signed [KT-1:0] wr1, wi1;
            always @(posedge clk) begin
                if (ce) begin
                    a1 <= ($signed({{(AW-KA){pr[KA-1]}}, pr}) <<< S) + pi;
                    a2 <= ($signed({{(AW-KA){pi[KA-1]}}, pi}) <<< S) + pr;
                    wr1 <= wr; wi1 <= wi;
                    m1 <= a1 * wr1;     // pr*wr (high) | pi*wr (low)
                    m2 <= a2 * wi1;     // pi*wi (high) | pr*wi (low)
                    re_q <= $signed(m1[PW-1:S]) - $signed(m2[PW-1:S]);
                    im_q <= $signed(m2[S-1:0]) + $signed(m1[S-1:0]);
                end
            end
            initial if (2 * KA + KT > 24 || KT > 18) $error("rp_cmul_dsp pack: 2*KA+KT=%0d", 2*KA+KT);
        end else if (MODE == 1) begin : g_3dsp
            (* use_dsp = "no" *) reg signed [KA:0] s1;
            (* use_dsp = "no" *)  reg signed [KT:0] s2, s3;
            reg signed [KA-1:0] pr1, pi1;
            reg signed [KT-1:0] wr1;
            (* use_dsp = "no" *) reg signed [KA+KT:0] k1, k2, k3;
            always @(posedge clk) begin
                if (ce) begin
                    s1 <= pr + pi;  s2 <= wi - wr;  s3 <= wr + wi;
                    pr1 <= pr; pi1 <= pi; wr1 <= wr;
                    k1 <= s1 * wr1;  k2 <= pr1 * s2;  k3 <= pi1 * s3;
                    re_q <= k1 - k3;
                    im_q <= k1 + k2;
                end
            end
            initial if (KA > 24 || KT > 17) $error("rp_cmul_dsp 3dsp: KA=%0d KT=%0d", KA, KT);
        end else begin : g_4dsp
            reg signed [KA-1:0] pr1, pi1;
            reg signed [KT-1:0] wr1, wi1;
            (* use_dsp = "no" *) reg signed [KA+KT-1:0] rr, ii, ri, ir;
            always @(posedge clk) begin
                if (ce) begin
                    pr1 <= pr; pi1 <= pi; wr1 <= wr; wi1 <= wi;
                    rr <= pr1 * wr1;  ii <= pi1 * wi1;
                    ri <= pr1 * wi1;  ir <= pi1 * wr1;
                    re_q <= rr - ii;
                    im_q <= ri + ir;
                end
            end
            initial if (KA > 25 || KT > 18) $error("rp_cmul_dsp 4dsp: KA=%0d KT=%0d", KA, KT);
        end
    endgenerate
endmodule
