`timescale 1ns/1ps

// V3 of the GR16S control-estimation task: ONE control copy per data domain.
// Domain 0 = the main datapath (ctl copy 0), domains 1/2 = the two replica
// domains (ctl copies 1/2).  Nothing is voted inside the stage: each domain
// runs on its own control bundle end to end, and the three control outputs
// are voted only where the stream crosses downstream boundaries (stage-8
// group exit / stages 9-10 / core output / core control), in the core.
//
// ctl copy d takes its input ticks from the previous stage's copy d only
// (VOTE_IN=0 with all three input pins tied to that copy), so a fault in
// one control copy can only move its own domain off the stream.
module sdf_stage_grp_v3 #(
    parameter integer LOGN = 11,
    parameter integer STAGE = 0,
    parameter integer MAX_LOGN = 11,
    parameter integer DATA_W = 16,
    parameter integer INVERSE = 0,
    parameter integer SCALE = 0,
    parameter integer K = 16,
    parameter integer KT = 13,
    parameter MEM_BASE = "chain/rtl/mem/grp17_twtab_p1_s"
) (
    input  wire                         clk,
    input  wire                         rst,
    input  wire [2:0]                   clear,   // per-copy stage_clear
    input  wire [2:0]                   ce,      // per-copy engine_ce
    input  wire [2:0]                   i_valid, // bit d = prev stage's ctl d
    input  wire [2:0]                   i_sof,
    input  wire [2:0]                   i_eof,
    input  wire signed [DATA_W-1:0]     i_re,
    input  wire signed [DATA_W-1:0]     i_im,
    output wire [2:0]                   o_valid, // per-copy control ticks
    output wire [2:0]                   o_sof,
    output wire [2:0]                   o_eof,
    output wire signed [DATA_W-1:0]     o_re,    // main datapath (domain 0)
    output wire signed [DATA_W-1:0]     o_im,
    output wire                         m_valid, // domain 0 datapath ticks
    output wire                         m_sof,
    output wire                         m_eof,
    output wire                         o_sample_overflow,
    output wire                         o_overflow_re,
    output wire                         o_overflow_im,
    output wire                         o_overflow_sticky,
    output wire [11:0]                  o_debug_delay_addr,
    output wire [11:0]                  o_debug_twiddle_addr,
    input  wire signed [DATA_W-1:0]     rep1_i_re,
    input  wire signed [DATA_W-1:0]     rep1_i_im,
    input  wire signed [DATA_W-1:0]     rep2_i_re,
    input  wire signed [DATA_W-1:0]     rep2_i_im,
    output wire signed [DATA_W-1:0]     rep1_o_re,
    output wire signed [DATA_W-1:0]     rep1_o_im,
    output wire signed [DATA_W-1:0]     rep2_o_re,
    output wire signed [DATA_W-1:0]     rep2_o_im,
    output wire [2:0]                   rep_o_valid
);
    // ---- three control copies, one per domain ------------------------------
    wire [2:0] lce, fh_d, cv_d, do_tw, un, qt, v_tw, s_tw, e_tw, rep_v;
    wire [10:0] p_a [0:2];
    wire [10:0] a_d [0:2];
    genvar d;
    generate
        for (d = 0; d < 3; d = d + 1) begin : g_ctl
            (* dont_touch = "true" *) grp_ctl #(
                .LOGN(LOGN), .STAGE(STAGE), .VOTE_IN(0)
            ) u_ctl (
                .clk(clk), .rst(rst), .clear(clear[d]), .ce(ce[d]),
                .i_valid3({3{i_valid[d]}}), .i_sof3({3{i_sof[d]}}),
                .i_eof3({3{i_eof[d]}}),
                .local_ce(lce[d]), .phase_addr(p_a[d]), .first_half_d(fh_d[d]),
                .addr_d(a_d[d]), .ctrl_valid_d(cv_d[d]),
                .do_tw(do_tw[d]), .unity(un[d]), .quarter(qt[d]),
                .valid_tw_d(v_tw[d]), .sof_tw_d(s_tw[d]), .eof_tw_d(e_tw[d]),
                .rep_i_valid(rep_v[d]),
                .o_valid(o_valid[d]), .o_sof(o_sof[d]), .o_eof(o_eof[d]));
        end
    endgenerate

    // ---- main datapath = domain 0 -------------------------------------------
    sdf_stage_dp #(
        .LOGN(LOGN), .STAGE(STAGE), .MAX_LOGN(MAX_LOGN), .DATA_W(DATA_W),
        .INVERSE(INVERSE), .SCALE(SCALE)
    ) u_dp (
        .clk(clk), .rst(rst), .clear(clear[0]), .ce(lce[0]),
        .i_re(i_re), .i_im(i_im),
        .phase_addr(p_a[0]), .first_half_d(fh_d[0]), .addr_d(a_d[0]),
        .ctrl_valid_d(cv_d[0]), .do_tw(do_tw[0]), .unity(un[0]), .quarter(qt[0]),
        .valid_tw_d(v_tw[0]), .sof_tw_d(s_tw[0]), .eof_tw_d(e_tw[0]),
        .o_valid(m_valid), .o_sof(m_sof), .o_eof(m_eof),
        .o_re(o_re), .o_im(o_im),
        .o_sample_overflow(o_sample_overflow),
        .o_overflow_re(o_overflow_re), .o_overflow_im(o_overflow_im),
        .o_overflow_sticky(o_overflow_sticky),
        .o_debug_delay_addr(o_debug_delay_addr),
        .o_debug_twiddle_addr(o_debug_twiddle_addr));

    // ---- replica domains 1 and 2 on their own copies ------------------------
    (* dont_touch = "true" *) grp_rep_dom #(
        .LOGN(LOGN), .STAGE(STAGE), .MAX_LOGN(MAX_LOGN), .DATA_W(DATA_W),
        .INVERSE(INVERSE), .K(K), .SC(SCALE), .KT(KT), .MEM_BASE(MEM_BASE)
    ) u_rep1 (
        .clk(clk), .rst(rst || clear[1]), .ce(lce[1]),
        .first_half(fh_d[1]), .phase_addr(p_a[1]),
        .addr_d(a_d[1]), .ctrl_valid(cv_d[1]),
        .do_twiddle(do_tw[1]), .unity(un[1]), .quarter(qt[1]),
        .i_valid(rep_v[1]),
        .i_re(rep1_i_re), .i_im(rep1_i_im),
        .o_valid(rep_o_valid[1]), .o_re(rep1_o_re), .o_im(rep1_o_im));

    (* dont_touch = "true" *) grp_rep_dom #(
        .LOGN(LOGN), .STAGE(STAGE), .MAX_LOGN(MAX_LOGN), .DATA_W(DATA_W),
        .INVERSE(INVERSE), .K(K), .SC(SCALE), .KT(KT), .MEM_BASE(MEM_BASE)
    ) u_rep2 (
        .clk(clk), .rst(rst || clear[2]), .ce(lce[2]),
        .first_half(fh_d[2]), .phase_addr(p_a[2]),
        .addr_d(a_d[2]), .ctrl_valid(cv_d[2]),
        .do_twiddle(do_tw[2]), .unity(un[2]), .quarter(qt[2]),
        .i_valid(rep_v[2]),
        .i_re(rep2_i_re), .i_im(rep2_i_im),
        .o_valid(rep_o_valid[2]), .o_re(rep2_o_re), .o_im(rep2_o_im));

    assign rep_o_valid[0] = m_valid;
endmodule
