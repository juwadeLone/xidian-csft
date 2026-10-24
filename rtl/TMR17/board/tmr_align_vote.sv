`timescale 1ns/1ps

// TMR16 alignment voter, one instance per replica (csa_cs_app_tmr g_rep[g]).
//
// sched_v is the bitwise majority of the three replicas' scheduling vectors;
// align is 1 when replica OWN's own vector differs from that majority.  Each
// replica has its own copy, so a wrong alignment voter disturbs only the
// replica it feeds; the other two keep aligning to a correct majority.  The
// three copies have the same majority logic: the wrapper instantiates them
// with DONT_TOUCH and keep_hierarchy so synthesis cannot merge them.
// Combinational, no register: the timing of the shared voter it replaces is
// unchanged.
module tmr_align_vote #(
    parameter integer W   = 10,
    parameter integer OWN = 0     // which input is this replica's own vector
) (
    input  wire [W-1:0] s0,
    input  wire [W-1:0] s1,
    input  wire [W-1:0] s2,
    output wire [W-1:0] sched_v,
    output wire         align
);
    wire [W-1:0] own = (OWN == 0) ? s0 : (OWN == 1) ? s1 : s2;
    // internal nets, so a testbench can force the voter's result (xsim
    // ignores force on ports that cross a generate boundary)
    wire [W-1:0] maj_w = (s0 & s1) | (s1 & s2) | (s0 & s2);
    // !==: an X sched bit means divergence too, and align must come out as a
    // clean 0/1 (it selects the replica's state_c/pass_c muxes -- an X align
    // would let the X lock in place).
    wire         align_w = (own !== maj_w);
    assign sched_v = maj_w;
    assign align   = align_w;
endmodule
