`timescale 1ns/1ps

// Channel exit: a packed {im[W-1:0], re[W-1:0]} word back into the 64-bit DDR container, each component
// sign-extended to 32 bits.
module pair_sext #(
    parameter integer W = 11
) (
    input  wire [2*W-1:0]   din,
    output wire [63:0]      dout
);
    assign dout = {{(32-W){din[2*W-1]}}, din[2*W-1:W], {(32-W){din[W-1]}}, din[W-1:0]};
endmodule
