`timescale 1ns/1ps

// RPR decision for one component (rpr_base, ported to the 16-bit chain).
//
//   y      : the full-precision (FP) result, W bits, after any injection point
//   r1, r2 : the two replicas of y's top K bits, aligned with y, as K+2-bit
//            two's-complement values that do not wrap (saturated replicas stay
//            inside the K-bit range)
//   o      : y, unless r1 == r2 and the top K bits of y differ from r1 by more
//            than T; then sat_K(r1) extended back to W bits
//
// The 16-bit chain's outputs saturate (they do not wrap): the compare is the
// non-modular one (MOD = 0) for node outputs, and the modular one (MOD = 1)
// only where the operand genuinely wraps (the butterfly write-back word, V_fb
// -- the 17-bit sum/difference wraps at its own width).
//
// Endpoint K = W (S = 0): the compare covers the whole word; the substitute
// is the saturated replica word itself (there are no low bits to pad).  T = 0
// then means exact equality, which is what a full-width replica gives.
//
// T is the largest fault-free |top(y) - r| of the node (set by its replica
// structure, rpr_params.vh).  Fault-free, the output is always y and the
// sticky flag stays 0.  One register after the decision, enabled with the
// pipeline's ce; REG = 0 makes the output combinational (used where the voted
// value feeds a pipeline whose alignment must not change).  The sticky flag
// is always registered; it sets on a valid beat whose value was replaced and
// clears only on clr or rst.
module rpr_vote #(
    parameter integer W = 16,
    parameter integer K = 16,
    parameter integer T = 0,
    parameter integer REG = 1,
    // number of top output bits this voter drives (the rest are 0); a replica
    // domain's voter feeds only the top bits its next node reads
    parameter integer OB = W,
    // MOD = 0 compares without wrap (saturating outputs: node data, CORDIC
    // coefficients); MOD = 1 modulo 2^K (operands that wrap at W bits: the
    // butterfly write-back word)
    parameter integer MOD = 0
) (
    input  wire                clk,
    input  wire                rst,
    input  wire                ce,
    input  wire                clr,
    input  wire                vld,
    input  wire [W-1:0]        y,
    input  wire [K+1:0]        r1,
    input  wire [K+1:0]        r2,
    output wire [W-1:0]        o,
    output reg                 sticky
);
    localparam integer S = W - K;
    wire [K-1:0] yh = y[W-1 -: K];
    wire signed [K-1:0] d = yh - r1[K-1:0];    // modulo 2^K
    wire signed [K+1:0] da = $signed({{2{yh[K-1]}}, yh}) - $signed(r1);   // no wrap
    wire bad = (MOD != 0) ? ((d > $signed(T[K-1:0])) || (d < -$signed(T[K-1:0])))
                          : ((da > $signed(T[K+1:0])) || (da < -$signed(T[K+1:0])));
    wire sel = (r1 == r2) && bad;
    wire signed [K+1:0] r1s = r1;
    // sat_K(r1): the replica word clamped to the K-bit signed range
    wire [K-1:0] sub = (r1s > $signed({3'b000, {(K-1){1'b1}}})) ? {1'b0, {(K-1){1'b1}}} :
                       (r1s < $signed({3'b111, {(K-1){1'b0}}})) ? {1'b1, {(K-1){1'b0}}} :
                       r1[K-1:0];
    wire [W-1:0] o_d;
    generate
        if (S == 0) begin : g_kw
            // K = W endpoint: the substitute is the replica word itself
            assign o_d = sel ? sub : y;
        end else begin : g_gen
            assign o_d = sel ? {sub, 1'b1, {(S-1){1'b0}}} : y;
        end
    endgenerate
    generate
        if (REG != 0) begin : g_reg
            reg [OB-1:0] o_q;
            always @(posedge clk) if (ce) o_q <= o_d[W-1 -: OB];
            if (OB < W) begin : g_lo
                assign o = {o_q, {(W-OB){1'b0}}};
            end else begin : g_full
                assign o = o_q;
            end
        end else begin : g_comb
            if (OB < W) begin : g_lo
                assign o = {o_d[W-1 -: OB], {(W-OB){1'b0}}};
            end else begin : g_full
                assign o = o_d;
            end
        end
    endgenerate
    always @(posedge clk) begin
        if (rst || clr)
            sticky <= 1'b0;
        else if (ce && vld && sel)
            sticky <= 1'b1;
    end
    initial begin
        if (K < 4 || S < 0 || T < 0 || T >= (1 << (K-2)))
            $error("rpr_vote: unsupported W=%0d K=%0d T=%0d", W, K, T);
        if (OB < 1 || OB > W)
            $error("rpr_vote: OB=%0d outside 1..W=%0d", OB, W);
    end
endmodule

// The same decision with two more ports: a forced substitution and the
// "replaced on this beat" output.  Used by the three-domain modules.
module rpr_vote_t #(
    parameter integer W = 16,
    parameter integer K = 16,
    parameter integer T = 0,
    parameter integer REG = 1,
    parameter integer OB = W,
    parameter integer MOD = 0
) (
    input  wire                clk,
    input  wire                rst,
    input  wire                ce,
    input  wire                clr,
    input  wire                vld,
    input  wire [W-1:0]        y,
    input  wire [K+1:0]        r1,
    input  wire [K+1:0]        r2,
    output wire [W-1:0]        o,
    output reg                 sticky,
    // fsub = replace whatever y is (the main path's tick disagrees with the
    // two replicas'); sel_o = this beat is replaced
    input  wire                fsub,
    output wire                sel_o
);
    localparam integer S = W - K;
    wire [K-1:0] yh = y[W-1 -: K];
    wire signed [K-1:0] d = yh - r1[K-1:0];    // modulo 2^K
    wire signed [K+1:0] da = $signed({{2{yh[K-1]}}, yh}) - $signed(r1);   // no wrap
    wire bad = (MOD != 0) ? ((d > $signed(T[K-1:0])) || (d < -$signed(T[K-1:0])))
                          : ((da > $signed(T[K+1:0])) || (da < -$signed(T[K+1:0])));
    wire sel = (r1 == r2) && (bad || fsub);
    assign sel_o = sel;
    wire signed [K+1:0] r1s = r1;
    wire [K-1:0] sub = (r1s > $signed({3'b000, {(K-1){1'b1}}})) ? {1'b0, {(K-1){1'b1}}} :
                       (r1s < $signed({3'b111, {(K-1){1'b0}}})) ? {1'b1, {(K-1){1'b0}}} :
                       r1[K-1:0];
    wire [W-1:0] o_d;
    generate
        if (S == 0) begin : g_kw
            assign o_d = sel ? sub : y;
        end else begin : g_gen
            assign o_d = sel ? {sub, 1'b1, {(S-1){1'b0}}} : y;
        end
    endgenerate
    generate
        if (REG != 0) begin : g_reg
            reg [OB-1:0] o_q;
            always @(posedge clk) if (ce) o_q <= o_d[W-1 -: OB];
            if (OB < W) begin : g_lo
                assign o = {o_q, {(W-OB){1'b0}}};
            end else begin : g_full
                assign o = o_q;
            end
        end else begin : g_comb
            if (OB < W) begin : g_lo
                assign o = {o_d[W-1 -: OB], {(W-OB){1'b0}}};
            end else begin : g_full
                assign o = o_d;
            end
        end
    endgenerate
    always @(posedge clk) begin
        if (rst || clr)
            sticky <= 1'b0;
        else if (ce && vld && sel)
            sticky <= 1'b1;
    end
    initial begin
        if (K < 4 || S < 0 || T < 0 || T >= (1 << (K-2)))
            $error("rpr_vote: unsupported W=%0d K=%0d T=%0d", W, K, T);
        if (OB < 1 || OB > W)
            $error("rpr_vote: OB=%0d outside 1..W=%0d", OB, W);
    end
endmodule
