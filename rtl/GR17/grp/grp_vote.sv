`timescale 1ns/1ps

// GR16S group-exit voter (gr16s_model.py group_transform, D359 comp):
//
//   cls    = leading-zero count of pos[10:2]                    (0..9)
//   comp2  = COMP_TAB[cls]          (signed 8-bit, = 2*COMP[class])
//   err2_r = 2*((re_main >> S8) - (vr1 >> S8)) - comp2   (per component)
//   fire   = (vr1 == vr2 on both components) AND
//            (|err2_r| > 2T OR |err2_i| > 2T)
//   emit   = vr1 + (comp2+1) << (S8-1)   for S8 >= 1
//            vr1 + ((comp2+1) >>> 1)    for S8 = 0 (K = 16 arm)
//   out    = fire ? emit : main_word     (BOTH components on any fire)
//   emit is saturated to the 16-bit word (the host model keeps unclamped
//   integers; a clamped word is the closest representable value and the
//   next stage saturates anyway).
//
// The operands are the REGISTERED stage-8 outputs (main o_re/o_im and the
// replicas' grid words) presented during the exit beat; the decision
// registers one ce edge later -- one added beat over W16R2 between the
// stage-8 output register and stage 9's input register.  valid/sof/eof pass
// through the same register; pos tracks the operand beat.
module grp_vote #(
    parameter integer W = 17,
    parameter integer K = 16,
    parameter integer T = 0,
    // 10 x signed 8-bit comp2 table, class c at COMP2[8*c +: 8]
    parameter [79:0] COMP2 = {80{1'b0}}
) (
    input  wire             clk,
    input  wire             rst,
    input  wire             clr,        // datapath re-arm (stage_clear)
    input  wire             clr_sticky, // explicit clear only -- sticky is
                                        // the fault record; need_clear's
                                        // re-arm must NOT erase it
    input  wire             ce,
    input  wire             i_valid,
    input  wire             i_sof,
    input  wire             i_eof,
    input  wire [10:0]      i_pos,
    input  wire signed [W-1:0] y_re,
    input  wire signed [W-1:0] y_im,
    input  wire signed [W-1:0] r1_re,
    input  wire signed [W-1:0] r1_im,
    input  wire signed [W-1:0] r2_re,
    input  wire signed [W-1:0] r2_im,
    output reg              o_valid,
    output reg              o_sof,
    output reg              o_eof,
    output reg  signed [W-1:0] o_re,
    output reg  signed [W-1:0] o_im,
    output reg              sticky,
    output wire             fire_mon
);
    localparam integer S8 = W - K;

    // class: consecutive zeros of pos[10:2] counted from bit 2 upward --
    // gr16s_model.pos_class: stage 8's branch bit is pos[2], so p[2]=1 is
    // class 0 and an all-zero window is class 9.
    wire [3:0] cls = i_pos[2]  ? 4'd0 : i_pos[3] ? 4'd1 : i_pos[4] ? 4'd2 :
                     i_pos[5]  ? 4'd3 : i_pos[6] ? 4'd4 : i_pos[7] ? 4'd5 :
                     i_pos[8]  ? 4'd6 : i_pos[9] ? 4'd7 : i_pos[10] ? 4'd8
                                                                  : 4'd9;
    wire signed [7:0] comp2 = COMP2[cls*8 +: 8];

    // compensated compare, in half-LSB units of the top-K grid
    wire signed [K:0] d_re = $signed(y_re[W-1 -: K]) - $signed(r1_re[W-1 -: K]);
    wire signed [K:0] d_im = $signed(y_im[W-1 -: K]) - $signed(r1_im[W-1 -: K]);
    wire signed [K+2:0] e2r = ($signed({d_re[K], d_re}) <<< 1) - comp2;
    wire signed [K+2:0] e2i = ($signed({d_im[K], d_im}) <<< 1) - comp2;
    wire signed [K+2:0] t2 = 2 * T;
    wire fire = (r1_re == r2_re) && (r1_im == r2_im) &&
                ((e2r > t2) || (e2r < -t2) || (e2i > t2) || (e2i < -t2));
    assign fire_mon = fire;

    // emit = vr + comp + half-LSB, saturated to the W-bit word
    wire signed [15:0] comp2p1 = $signed({{8{comp2[7]}}, comp2}) + 16'sd1;
    wire signed [31:0] add = (S8 >= 1) ? ({{16{comp2p1[15]}}, comp2p1} <<< (S8-1))
                                       : ({{16{comp2p1[15]}}, comp2p1} >>> 1);
    wire signed [31:0] er = $signed({{(32-W){r1_re[W-1]}}, r1_re}) + add;
    wire signed [31:0] ei = $signed({{(32-W){r1_im[W-1]}}, r1_im}) + add;
    localparam signed [31:0] WMAX = (32'sd1 <<< (W-1)) - 1;
    localparam signed [31:0] WMIN = -(32'sd1 <<< (W-1));
    wire signed [W-1:0] esat_re = (er > WMAX) ? WMAX[W-1:0] :
                                  (er < WMIN) ? WMIN[W-1:0] : er[W-1:0];
    wire signed [W-1:0] esat_im = (ei > WMAX) ? WMAX[W-1:0] :
                                  (ei < WMIN) ? WMIN[W-1:0] : ei[W-1:0];

    always @(posedge clk) begin
        if (rst || clr) begin
            o_valid <= 1'b0;
            o_sof   <= 1'b0;
            o_eof   <= 1'b0;
            o_re    <= '0;
            o_im    <= '0;
        end else if (ce) begin
            o_valid <= i_valid;
            o_sof   <= i_sof;
            o_eof   <= i_eof;
            o_re    <= fire ? esat_re : y_re;
            o_im    <= fire ? esat_im : y_im;
        end
    end

    // sticky is the fault record: it survives the inter-frame re-arm
    // (need_clear) and clears only on rst or the explicit chain clear.
    always @(posedge clk) begin
        if (rst || clr_sticky)
            sticky <= 1'b0;
        else if (ce && i_valid && fire)
            sticky <= 1'b1;
    end

    initial begin
        if (K < 4 || K > W || T < 0)
            $error("grp_vote: unsupported K=%0d T=%0d", K, T);
    end
endmodule
