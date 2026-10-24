// ============================================================================
//  k7_fp_alu -- one shared, sequential arithmetic unit for the K7 CS seed
//  builder, shaped like the a7 baseline's f32_alu_xil + f32_divsqrt pair:
//  start/op/a/b -> y/done, ONE operation at a time, no per-call-site hardware.
//
//  WHY (root cause, 2026-09-11):
//    cs_seed_builder.sv was already micro-sequenced (one operator call per `st`
//    step) -- the synthesis explosion came from the operators being tasks:
//    every call site materialised its own hardware (33 copies of a 128-step
//    restoring divider, 66 copies of a 64x64 multiplier, ...).  Millions of
//    LUTs on a 203k-LUT part; RTL Optimization Phase 2 degraded >250x.
//    a7 never had this: ONE ALU (add + mul IP + sequential div/sqrt) driven by
//    a step machine, plus ONE 48-bit integer restoring divider + int64 isqrt
//    for the phase-critical terms (a7 source: "f32 product drops phase").
//
//  NUMERIC CONTRACT -- bit-exact with cs_seed_builder.sv.  The 10 cheap
//  operators are called straight from cs_seed_ops.svh (one instance each).
//  The 4 heavy ones (sf_div/rhe_div, sf_sqrt/isqrt128) are clocked iterations
//  of the same loops; sim/tb/tb_k7_fp_alu.sv checks all 14 against the
//  reference over thousands of random + corner vectors.
//
//  a7 note: a7's Xilinx FP32 IP path is NOT usable here -- a7 quantises its
//  table to 2*pi/2^16, ours to 2*pi/2^32, which needs ~2^-62 relative
//  accuracy; f32 (2^-24) is ~7 orders short (measured: one f32 word of lam
//  alone costs c4 theta0 5.0e6 LSB).
// ============================================================================
`timescale 1ns / 1ps

module k7_fp_alu (
    input  logic                 clk,
    input  logic                 rst,          // sync reset, active high
    input  logic                 start,        // 1-cycle pulse: begin op
    input  logic [4:0]           op,
    input  logic signed [127:0]  a,
    input  logic signed [127:0]  b,
    input  logic signed [31:0]   ae,
    input  logic signed [31:0]   be,
    input  logic signed [31:0]   aux,          // shift amount / k / S
    output logic signed [127:0]  y,            // float ops: mantissa in y[63:0]
    output logic signed [31:0]   ye,           // float ops: exponent
    output logic                 done,
    output logic [4:0]           busy_op
);

    // ------------------------------------------------------------------ ops
    localparam logic [4:0] OP_NORM      = 5'd0;   // a[127:0]        -> (m,e)
    localparam logic [4:0] OP_ADD       = 5'd1;   // (a,ae)+(b,be)   -> (m,e)
    localparam logic [4:0] OP_SUB       = 5'd2;   // (a,ae)-(b,be)   -> (m,e)
    localparam logic [4:0] OP_MUL       = 5'd3;   // (a,ae)*(b,be)   -> (m,e)
    localparam logic [4:0] OP_DIV       = 5'd4;   // (a,ae)/(b,be)   -> (m,e)
    localparam logic [4:0] OP_SQRT      = 5'd5;   // isqrt(a,ae)     -> (m,e)
    localparam logic [4:0] OP_INT       = 5'd6;   // integer a       -> (m,e)
    localparam logic [4:0] OP_F32       = 5'd7;   // binary32 a      -> (m,e)
    localparam logic [4:0] OP_Q64       = 5'd8;   // (a,ae)          -> Q64 int
    localparam logic [4:0] OP_RHE_POW2  = 5'd9;   // RHE(a / 2^aux), 128 bit
    localparam logic [4:0] OP_RHE_DIV   = 5'd10;  // RHE(a / b[63:0])
    localparam logic [4:0] OP_ISQRT128  = 5'd11;  // floor(sqrt(a[127:0]))
    localparam logic [4:0] OP_F32_TO_FX = 5'd12;  // binary32 a * 2^aux -> fx64
    localparam logic [4:0] OP_COUNT32   = 5'd13;  // count(a[127:0], aux)

    // ---------- the 14 reference operators: exactly one instance each -------
    `include "cs_seed_ops.svh"

    localparam logic [1:0] S_IDLE = 2'd0, S_RUN = 2'd1, S_FIN = 2'd2;
    reg [1:0] cs;
    reg [7:0] cnt;
    reg [4:0] op_r;
    reg signed [127:0] a_r, b_r;
    reg signed [31:0]  ae_r, be_r, aux_r;

    reg [127:0] dv_a, dv_r, dv_q, dv_den;   // restoring divider (60% of a7's here)
    reg         dv_neg;
    reg [127:0] sq_a;                        // radix-4 restoring sqrt
    reg [67:0]  sq_r;
    reg [63:0]  sq_root;
    reg signed [127:0] mul_p;                // one 64x64 multiplier, shared
    reg [127:0]        ml_lo, ml_hi;

    logic signed [127:0] tnum, tq, tp;
    logic [127:0] rr128, qq128, tt128, fm128;
    logic [67:0]  rr68, tst68;
    logic [63:0]  tmp64;
    logic signed [63:0] mo;
    integer eo, ii, sh;
    logic rb, st;

    always @(posedge clk) begin
        if (rst) begin
            cs <= S_IDLE; done <= 1'b0; busy_op <= 5'd0; cnt <= 8'd0;
            y <= 128'sd0; ye <= 32'sd0;
            op_r <= 5'd0; a_r <= 128'sd0; b_r <= 128'sd0;
            ae_r <= 32'sd0; be_r <= 32'sd0; aux_r <= 32'sd0;
            dv_a <= 128'd0; dv_r <= 128'd0; dv_q <= 128'd0; dv_den <= 128'd0; dv_neg <= 1'b0;
            sq_a <= 128'd0; sq_r <= 68'd0; sq_root <= 64'd0;
            mul_p <= 128'sd0; ml_lo <= 128'd0; ml_hi <= 128'd0;
        end else begin
            done <= 1'b0;
            case (cs)
            // --------------------------------------------------------- issue
            S_IDLE: begin
                if (start) begin
                    op_r <= op; a_r <= a; b_r <= b;
                    ae_r <= ae; be_r <= be; aux_r <= aux; busy_op <= op;
                    case (op)
                    OP_DIV: begin
                        if (b[63:0] == 64'sd0) begin          // sf_div: bm == 0 -> (0,0)
                            y <= 128'sd0; ye <= 32'sd0; cs <= S_FIN;
                        end else begin
                            tnum   = {{64{a[63]}}, a[63:0]} <<< 62;
                            dv_a   <= tnum[127] ? (~tnum + 128'sd1) : tnum;
                            dv_den <= {64'd0, (b[63] ? (~b[63:0] + 64'd1) : b[63:0])};
                            dv_neg <= a[63] ^ b[63];
                            dv_r <= 128'd0; dv_q <= 128'd0; cnt <= 8'd129; cs <= S_RUN;
                        end
                    end
                    OP_RHE_DIV: begin
                        dv_a   <= a[127] ? (~a + 128'sd1) : a;
                        dv_den <= {64'd0, b[63:0]};
                        dv_neg <= a[127];
                        dv_r <= 128'd0; dv_q <= 128'd0; cnt <= 8'd129; cs <= S_RUN;
                    end
                    OP_SQRT: begin
                        if ($signed(a[63:0]) <= 64'sd0) begin   // sf_sqrt: am <= 0 -> (0,0)
                            y <= 128'sd0; ye <= 32'sd0; cs <= S_FIN;
                        end else begin
                            sq_a <= ({64'd0, $unsigned(a[63:0])}) <<< (ae[0] ? 63 : 62);
                            sq_r <= 68'd0; sq_root <= 64'd0; cnt <= 8'd65; cs <= S_RUN;
                        end
                    end
                    OP_ISQRT128: begin
                        sq_a <= a; sq_r <= 68'd0; sq_root <= 64'd0; cnt <= 8'd65; cs <= S_RUN;
                    end
                    OP_MUL:     begin cnt <= 8'd2; cs <= S_RUN; end
                    OP_COUNT32: begin cnt <= 8'd3; cs <= S_RUN; end
                    default:    begin cnt <= 8'd1; cs <= S_RUN; end
                    endcase
                end
            end
            // ---------------------------------------------------------- work
            S_RUN: begin
                case (op_r)
                // 128 restoring steps then trailing round-half-even + sign
                OP_DIV, OP_RHE_DIV: begin
                    if (cnt >= 8'd2) begin
                        ii    = int'(cnt) - 2;
                        rr128 = (dv_r << 1) | {{127{1'b0}}, dv_a[ii]};
                        qq128 = dv_q << 1;
                        if (rr128 >= dv_den) begin dv_r <= rr128 - dv_den; dv_q <= qq128 | 128'd1; end
                        else                 begin dv_r <= rr128;          dv_q <= qq128;         end
                        cnt <= cnt - 8'd1;
                    end else begin
                        tt128 = dv_r << 1;
                        tq    = dv_q;
                        if ((tt128 > dv_den) || ((tt128 == dv_den) && dv_q[0])) tq = tq + 128'sd1;
                        if (dv_neg) tq = -tq;
                        if (op_r == OP_DIV) begin
                            sf_norm(tq, int'(ae_r) - int'(be_r) - 62, mo, eo);
                            y <= {{64{mo[63]}}, mo}; ye <= eo;
                        end else begin
                            y <= tq; ye <= 32'sd0;
                        end
                        cs <= S_FIN;
                    end
                end
                // 64 radix-4 restoring steps (floor) then normalise
                OP_SQRT, OP_ISQRT128: begin
                    if (cnt >= 8'd2) begin
                        ii    = int'(cnt) - 2;
                        rr68  = (sq_r << 2) | {66'd0, sq_a[2*ii +: 2]};
                        tst68 = {sq_root, 2'b01};
                        if (rr68 >= tst68) begin sq_r <= rr68 - tst68; sq_root <= (sq_root << 1) | 64'd1; end
                        else               begin sq_r <= rr68;         sq_root <= (sq_root << 1);        end
                        cnt <= cnt - 8'd1;
                    end else begin
                        if (op_r == OP_ISQRT128) begin
                            y <= {64'd0, sq_root}; ye <= 32'sd0;
                        end else begin
                            sf_norm({124'd0, sq_root},
                                    ae_r[0] ? (((int'(ae_r) - 1) / 2) - 31) : ((int'(ae_r) / 2) - 31),
                                    mo, eo);
                            y <= {{64{mo[63]}}, mo}; ye <= eo;
                        end
                        cs <= S_FIN;
                    end
                end
                // one 64x64 multiplier, split over two cycles
                OP_MUL: begin
                    if (cnt == 8'd2) begin
                        mul_p <= $signed(a_r[63:0]) * $signed(b_r[63:0]);
                        cnt   <= 8'd1;
                    end else begin
                        sf_norm(mul_p, int'(ae_r) + int'(be_r), mo, eo);
                        y <= {{64{mo[63]}}, mo}; ye <= eo; cs <= S_FIN;
                    end
                end
                // same multiplier reused for the two half products
                OP_COUNT32: begin
                    if (cnt == 8'd3) begin
                        ml_lo <= $unsigned(a_r[63:0]) * $unsigned(K_CONST);
                        cnt   <= 8'd2;
                    end else if (cnt == 8'd2) begin
                        ml_hi <= $unsigned($signed(a_r[127:64]) * $signed(K_CONST));
                        cnt   <= 8'd1;
                    end else begin
                        tp  = ml_lo + (ml_hi <<< 64);
                        sh  = int'(aux_r) + 32;
                        if (sh >= 128) begin
                            y <= 128'sd0; ye <= 32'sd0;
                        end else begin
                            qq128 = tp >>> sh;
                            rb    = tp[sh-1];
                            fm128 = (128'd1 << (sh - 1)) - 128'd1;
                            st    = |(tp & fm128);
                            if (rb && (st || qq128[0])) qq128 = qq128 + 128'd1;
                            y <= {{96{1'b0}}, qq128[31:0]}; ye <= 32'sd0;
                        end
                        cs <= S_FIN;
                    end
                end
                // ---- single-cycle operators: call the reference directly ----
                OP_NORM: begin
                    sf_norm(a_r, int'(ae_r), mo, eo);
                    y <= {{64{mo[63]}}, mo}; ye <= eo; cs <= S_FIN;
                end
                OP_ADD: begin
                    sf_add(a_r[63:0], int'(ae_r), b_r[63:0], int'(be_r), mo, eo);
                    y <= {{64{mo[63]}}, mo}; ye <= eo; cs <= S_FIN;
                end
                OP_SUB: begin
                    sf_sub(a_r[63:0], int'(ae_r), b_r[63:0], int'(be_r), mo, eo);
                    y <= {{64{mo[63]}}, mo}; ye <= eo; cs <= S_FIN;
                end
                OP_INT: begin
                    sf_int(a_r[31:0], mo, eo);
                    y <= {{64{mo[63]}}, mo}; ye <= eo; cs <= S_FIN;
                end
                OP_F32: begin
                    sf_f32(a_r[31:0], mo, eo);
                    y <= {{64{mo[63]}}, mo}; ye <= eo; cs <= S_FIN;
                end
                OP_Q64: begin
                    sf_q64(a_r[63:0], int'(ae_r), tq);
                    y <= tq; ye <= 32'sd0; cs <= S_FIN;
                end
                OP_RHE_POW2: begin
                    y <= rhe_pow2(a_r, int'(aux_r)); ye <= 32'sd0; cs <= S_FIN;
                end
                OP_F32_TO_FX: begin
                    tmp64 = f32_to_fx(a_r[31:0], int'(aux_r));
                    y <= {{64{tmp64[63]}}, tmp64}; ye <= 32'sd0; cs <= S_FIN;
                end
                default: begin
                    y <= a_r; ye <= 32'sd0; cs <= S_FIN;
                end
                endcase
            end
            // ---------------------------------------------------------- done
            S_FIN: begin done <= 1'b1; cs <= S_IDLE; end
            default: cs <= S_IDLE;
            endcase
        end
    end
endmodule

