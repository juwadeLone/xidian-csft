`timescale 1ns/1ps

// Combinational bitwise three-way majority with per-replica sticky
// mismatch flags.
//
// `voted` is a pure majority of the three input buses -- no register on
// the data path, so a healthy triad's waveform is cycle-identical to a
// single chain's.  That is what allows it on handshake lines (the
// baseline tmr_vote.sv registers its output and cannot be used there).
//
// flag_x is set the first clock in_x differs from `voted` and stays set
// until clr (asserted once per frame start) or rst.  In a healthy triad
// all three flags stay 0.
module tmr_voter #(
    parameter integer W = 1
) (
    input  wire         clk,
    input  wire         rst,
    input  wire         clr,        // frame start: clear the flags
    input  wire [W-1:0] in_a,
    input  wire [W-1:0] in_b,
    input  wire [W-1:0] in_c,
    output wire [W-1:0] voted,
    output reg          flag_a,
    output reg          flag_b,
    output reg          flag_c
);
    assign voted = (in_a & in_b) | (in_a & in_c) | (in_b & in_c);

    always @(posedge clk) begin
        if (rst || clr) begin
            flag_a <= 1'b0;
            flag_b <= 1'b0;
            flag_c <= 1'b0;
        end else begin
            // !==: a replica whose bundle carries X where the voted
            // bundle is defined (e.g. an idle X bus next to a driven one)
            // IS a divergence and must flag; logical != would return X
            // and the if() would swallow it.
            if (in_a !== voted) flag_a <= 1'b1;
            if (in_b !== voted) flag_b <= 1'b1;
            if (in_c !== voted) flag_c <= 1'b1;
        end
    end
endmodule
