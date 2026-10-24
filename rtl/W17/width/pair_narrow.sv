`timescale 1ns/1ps

// Channel entry of a 64-bit DDR container word {im[31:0], re[31:0]} (each component sign-extended to 32 bits):
// keep the low W bits of each component.
//   SRC_W : component width the container is allowed to carry.  Bits [31:SRC_W-1] of each component must be
//           copies of its sign bit.
//   W     : component width inside the channel (SRC_W >= W).
//   SRC_W = W : the bits dropped are sign copies, narrowing loses nothing.
//   SRC_W > W : the SRC_W-bit value is clamped to [-2^(W-1), 2^(W-1)-1], the same limit as the model's
//               _limit_pair (p0: 12-bit input -> 11 bits at the AZ FFT channel entry).
// sat_re / sat_im are high on a word whose component was clamped.  In simulation every valid word is checked for the
// sign-copy condition on the dropped container bits; `violations` counts words that break it and `accepted` counts
// the valid words checked.  CHECK_MSG = 0 keeps the counting but drops the $error text.
module pair_narrow #(
    parameter integer SRC_W = 11,
    parameter integer W = 11,
    parameter integer CHECK_MSG = 1
) (
    input  wire             clk,
    input  wire             valid,
    input  wire [63:0]      din,
    output wire [2*W-1:0]   dout,
    output wire             sat_re,
    output wire             sat_im
);
    wire [31:0] re = din[31:0];
    wire [31:0] im = din[63:32];

    // bits [hi:lo] are all equal
    function automatic same_bits;
        input [31:0] v;
        input integer hi;
        input integer lo;
        reg [31:0] m;
        begin
            m = (32'hffffffff >> (31 - hi)) & (32'hffffffff << lo);
            same_bits = ((v & m) == 32'd0) || ((v & m) == m);
        end
    endfunction

    wire ovf_re = !same_bits(re, SRC_W - 1, W - 1);
    wire ovf_im = !same_bits(im, SRC_W - 1, W - 1);
    wire [W-1:0] re_o = ovf_re ? (re[SRC_W-1] ? {1'b1, {(W-1){1'b0}}} : {1'b0, {(W-1){1'b1}}}) : re[W-1:0];
    wire [W-1:0] im_o = ovf_im ? (im[SRC_W-1] ? {1'b1, {(W-1){1'b0}}} : {1'b0, {(W-1){1'b1}}}) : im[W-1:0];
    assign dout = {im_o, re_o};
    assign sat_re = ovf_re;
    assign sat_im = ovf_im;

`ifndef SYNTHESIS
    integer violations = 0;
    integer accepted = 0;
    initial begin
        if (W < 2 || SRC_W < W || SRC_W > 32)
            $error("pair_narrow: need 2 <= W <= SRC_W <= 32");
    end
    always @(posedge clk) begin
        if (valid) begin
            accepted = accepted + 1;
            if (!same_bits(re, 31, SRC_W - 1) || !same_bits(im, 31, SRC_W - 1)) begin
                violations = violations + 1;
                if (CHECK_MSG != 0)
                    $error("pair_narrow: container bits above %0d are not sign copies: re=%h im=%h", SRC_W, re, im);
            end
        end
    end
`endif
endmodule
