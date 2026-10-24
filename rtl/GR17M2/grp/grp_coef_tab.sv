`timescale 1ns/1ps

// Full-circle midpoint coefficient table for a GR17 product replica
// (gr17_model.rep_coef_table(12, KC)): the index is the top P = 12 bits of
// the domain's own 32-bit phase accumulator; the table holds 4096 midpoint
// cos/sin entries, each a KC-bit signed Q1.(KC-1) pair.  Each memory word is
// 2*KC bits, {sin[2*KC-1:KC], cos[KC-1:0]}.
//
// GR16SC3A's single-quadrant fold and its 13-bit midtab/exctab are gone:
// the full-circle table needs no fold logic and is bit-exact by
// construction (the fold's clamped boundary cells could not reproduce the
// model's saturated entries).
//
// Pipeline: phase -> index reg -> table reg -> CORDIC_ITERATIONS-2 delay
// taps -> output register = CORDIC_ITERATIONS + 2 ce edges, matching
// cordic_phase_to_unit's i_valid -> o_valid, so the replica coefficient is
// presented on the same beat as the main path's CORDIC coefficient.
module grp_coef_tab #(
    parameter integer CORDIC_ITERATIONS = 25,
    parameter integer KC = 12,          // coefficient bits (Q1.(KC-1))
    parameter MEM_FILE = "chain/rtl/mem/grp17_coef_kc12.mem"
) (
    input  wire             clk,
    input  wire             rst,
    input  wire             clear,
    input  wire             ce,
    input  wire             i_valid,     // domain recurrence 'valid'
    input  wire [11:0]      phase,       // phase[31:20]
    output wire             o_valid,
    output wire signed [KC-1:0] c_re,
    output wire signed [KC-1:0] c_im
);
    (* rom_style = "block" *) reg [2*KC-1:0] tab [0:4095];
    initial $readmemh(MEM_FILE, tab);

    // pipeline: idx reg (1) -> table reg (2) -> dc[0..IT-2] (IT-1 regs)
    //           -> c_re_q (IT+2 edges total, IT = CORDIC_ITERATIONS)
    reg [11:0] idx_q;
    reg [2*KC-1:0] cs_q;
    reg [CORDIC_ITERATIONS:0] valid_pipe;
    always @(posedge clk) begin
        if (ce) begin
            idx_q <= phase;
            cs_q <= tab[idx_q];
        end
        if (rst || clear)
            valid_pipe <= '0;
        else if (ce)
            valid_pipe <= {valid_pipe[CORDIC_ITERATIONS-1:0], i_valid};
    end
    reg signed [KC-1:0] dc [0:CORDIC_ITERATIONS-2];
    reg signed [KC-1:0] ds [0:CORDIC_ITERATIONS-2];
    integer i;
    always @(posedge clk) begin
        if (ce) begin
            dc[0] <= $signed(cs_q[KC-1:0]);
            ds[0] <= $signed(cs_q[2*KC-1:KC]);
            for (i = 1; i <= CORDIC_ITERATIONS-2; i = i + 1) begin
                dc[i] <= dc[i-1];
                ds[i] <= ds[i-1];
            end
        end
    end
    reg signed [KC-1:0] c_re_q, c_im_q;
    reg o_valid_q;
    always @(posedge clk) begin
        if (rst || clear) begin
            c_re_q <= '0;
            c_im_q <= '0;
            o_valid_q <= 1'b0;
        end else if (ce) begin
            c_re_q <= dc[CORDIC_ITERATIONS-2];
            c_im_q <= ds[CORDIC_ITERATIONS-2];
            o_valid_q <= valid_pipe[CORDIC_ITERATIONS];
        end
    end
    assign c_re = c_re_q;
    assign c_im = c_im_q;
    assign o_valid = o_valid_q;
endmodule
