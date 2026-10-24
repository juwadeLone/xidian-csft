`timescale 1ns/1ps

// GR16S-ctl-est V2/V3 core (CTL_MODE = 2 or 3):
//
//   * core control is triplicated: three fft_iso_ctrl copies under
//     dont_touch, exactly the sdf_fft_core / sdf_fft_core_grp logic split
//     out; s_ready / busy / protocol_error / the datapath-facing ce and
//     stage_clear are per-signal majorities of the three copies.
//   * inside the group (stages 0..8) tick plumbing is a three-copy vector:
//     stage g's ctl copies take the three previous-stage ctl outputs (V2:
//     each copy votes internally; V3: copy d follows copy d only).
//   * at the stage-8 group exit the stream becomes single again: the
//     valid/SOF/EOF feeding pos_q, grp_vote and stages 9-10 is the
//     majority of stage 8's three ctl outputs.  Stages 9-10 stay plain
//     sdf_stage on the voted ce / stage_clear.  The core output ticks and
//     the ticks each core-ctl copy watches are this single stream.
//   * datapath input mux (re_p[0]) is gated by the voted accept.
//
//   CTL_MODE = 4 (GR16SC3A): fully per-domain control.  Stage ctl copies
//   already take per-copy ce/clear/valid; the only remaining signal that
//   fed all three domains from one vote is the shared input-data gate.
//   Mode 4 makes the input gate per-domain too: domain d's data word is
//   gated by acc[d] (domain 0 = main path, 1/2 = replica paths), and each
//   domain's tick vector starts from its own copy's accept.  The single
//   input stream s_re/s_im is shared by construction; gating it with the
//   domain's own accept keeps a core-ctl fault contained to its domain.
//   The voted lines that stay single are exactly the boundary/engine
//   ones: s_ready, busy, protocol_error, ece_m/sclr_m at the voter,
//   pos_q, tail stages 9-10 and the m_* output stream.
module sdf_fft_core_grp_v23 #(
    parameter integer LOGN = 11,
    parameter integer DATA_W = 16,
    parameter integer INVERSE = 0,
    parameter [11:0] SCALE_MASK = 12'h000,
    parameter [3:0] CORE_ID = 4'd0,
    parameter [71:0] GRP_KV = 72'd0,
    parameter [71:0] GRP_KTV = 72'd0,     // replica twiddle width KT_g at [8g +: 8]
    parameter GRP_TWMEM = "chain/rtl/mem/grp17_twtab_p1_s",  // table family prefix
    parameter integer GRP_T = 0,
    parameter [79:0] GRP_COMP2 = 80'd0,
    parameter integer CTL_MODE = 2        // 2: voted shared control
                                          // 3: one control copy per domain
                                          // 4: per-domain incl. input gate
) (
    input  wire                         clk,
    input  wire                         rst,
    input  wire                         clear,
    input  wire                         s_valid,
    output wire                         s_ready,
    input  wire                         s_sof,
    input  wire                         s_eof,
    input  wire signed [DATA_W-1:0]     s_re,
    input  wire signed [DATA_W-1:0]     s_im,
    output wire                         m_valid,
    output wire                         m_sof,
    output wire                         m_eof,
    output wire signed [DATA_W-1:0]     m_re,
    output wire signed [DATA_W-1:0]     m_im,
    output wire                         busy,
    output reg                          protocol_error,
    output wire                         overflow_sticky,
    output wire [LOGN-1:0]              stage_overflow,
    output wire [LOGN*12-1:0]           debug_delay_addr,
    output wire [LOGN*12-1:0]           debug_twiddle_addr,
    output wire [LOGN-1:0]              debug_stage_valid,
    output wire [3:0]                   core_id,
    output wire                         rpr_sticky
);
    localparam integer N = 1 << LOGN;
    localparam integer GLAST = 8;
    assign core_id = CORE_ID;

    // ---- core control, three copies (fft_iso_ctrl = sdf_fft_core ctl) -------
    wire [2:0] rdy, acc, ece, sclr, bsy, perr;
    genvar d;
    generate
        for (d = 0; d < 3; d = d + 1) begin : g_cctl
            (* dont_touch = "true" *) fft_iso_ctrl #(.LOGN(LOGN)) u_cctl (
                .clk(clk), .rst(rst), .clear(clear),
                .s_valid(s_valid), .s_sof(s_sof), .s_eof(s_eof), .rdy(rdy),
                .m_valid(m_valid), .m_eof(m_eof),
                .s_ready_own(rdy[d]), .ready_maj(), .accept(acc[d]),
                .engine_ce(ece[d]), .stage_clear(sclr[d]),
                .busy(bsy[d]), .protocol_error(perr[d]));
        end
    endgenerate
    wire ece_m  = (ece[0]  & ece[1])  | (ece[0]  & ece[2])  | (ece[1]  & ece[2]);
    wire sclr_m = (sclr[0] & sclr[1]) | (sclr[0] & sclr[2]) | (sclr[1] & sclr[2]);
    wire acc_m  = (acc[0]  & acc[1])  | (acc[0]  & acc[2])  | (acc[1]  & acc[2]);
    assign s_ready        = (rdy[0]  & rdy[1])  | (rdy[0]  & rdy[2])  | (rdy[1]  & rdy[2]);
    assign busy           = (bsy[0]  & bsy[1])  | (bsy[0]  & bsy[2])  | (bsy[1]  & bsy[2]);
    always @(*) protocol_error = (perr[0] & perr[1]) | (perr[0] & perr[2]) | (perr[1] & perr[2]);

    // ---- stage pipes ---------------------------------------------------------
    // group ticks are three-copy vectors; data pipes stay single
    wire [2:0] v_p [0:GLAST+1];
    wire [2:0] s_p [0:GLAST+1];
    wire [2:0] e_p [0:GLAST+1];
    wire signed [DATA_W-1:0] re_pipe [0:LOGN];
    wire signed [DATA_W-1:0] im_pipe [0:LOGN];
    wire signed [DATA_W-1:0] r1_re [0:GLAST+1];
    wire signed [DATA_W-1:0] r1_im [0:GLAST+1];
    wire signed [DATA_W-1:0] r2_re [0:GLAST+1];
    wire signed [DATA_W-1:0] r2_im [0:GLAST+1];
    // single-copy ticks downstream of the group
    wire [LOGN:GLAST+1] valid_pipe;
    wire [LOGN:GLAST+1] sof_pipe;
    wire [LOGN:GLAST+1] eof_pipe;
    wire [LOGN-1:0] sample_overflow_pipe;
    wire [LOGN-1:0] sticky_overflow_pipe;
    reg  [LOGN-1:0] stage_overflow_sticky_q;

    always @(posedge clk) begin
        if (rst || clear)
            stage_overflow_sticky_q <= '0;
        else
            stage_overflow_sticky_q <= stage_overflow_sticky_q |
                                       sample_overflow_pipe;
    end

    assign v_p[0] = acc;
    assign s_p[0] = acc & {3{s_sof}};
    assign e_p[0] = acc & {3{s_eof}};
    if (CTL_MODE == 4) begin : g_gate_pd
        // per-domain input gate: domain d's data word is qualified by its
        // own ctl copy's accept; a fault in one copy cannot push wrong
        // (or wrongly absent) data into the other two domains.
        assign re_pipe[0] = acc[0] ? s_re : '0;
        assign im_pipe[0] = acc[0] ? s_im : '0;
        assign r1_re[0]   = acc[1] ? s_re : '0;
        assign r1_im[0]   = acc[1] ? s_im : '0;
        assign r2_re[0]   = acc[2] ? s_re : '0;
        assign r2_im[0]   = acc[2] ? s_im : '0;
    end else begin : g_gate_v
        assign re_pipe[0] = acc_m ? s_re : '0;
        assign im_pipe[0] = acc_m ? s_im : '0;
        assign r1_re[0]   = re_pipe[0];
        assign r1_im[0]   = im_pipe[0];
        assign r2_re[0]   = re_pipe[0];
        assign r2_im[0]   = im_pipe[0];
    end

    // raw outputs of group stage 8 (pre-voter)
    wire                     g8_valid, g8_sof, g8_eof;
    wire signed [DATA_W-1:0] g8_re, g8_im;

    genvar g;
    generate
        for (g = 0; g <= GLAST; g = g + 1) begin : g_sdf_stage
            localparam integer KG = (GRP_KV[8*g +: 8] < 4) ? DATA_W
                                                         : GRP_KV[8*g +: 8];
            localparam integer KTG = (GRP_KTV[8*g +: 8] < 8) ? 13
                                                           : GRP_KTV[8*g +: 8];
            wire signed [DATA_W-1:0] ore, oim;
            if (CTL_MODE == 2) begin : g_v2
                (* dont_touch = "true" *) sdf_stage_grp_v2 #(
                    .LOGN(LOGN), .STAGE(g), .MAX_LOGN(11), .DATA_W(DATA_W),
                    .INVERSE(INVERSE), .SCALE(SCALE_MASK[g]), .K(KG)
                ) u_stage (
                    .clk(clk), .rst(rst), .clear(sclr), .ce(ece),
                    .i_valid(v_p[g]), .i_sof(s_p[g]), .i_eof(e_p[g]),
                    .i_re(re_pipe[g]), .i_im(im_pipe[g]),
                    .o_valid(v_p[g+1]), .o_sof(s_p[g+1]), .o_eof(e_p[g+1]),
                    .o_re(ore), .o_im(oim),
                    .m_valid(), .m_sof(), .m_eof(),
                    .o_sample_overflow(sample_overflow_pipe[g]),
                    .o_overflow_re(), .o_overflow_im(),
                    .o_overflow_sticky(sticky_overflow_pipe[g]),
                    .o_debug_delay_addr(debug_delay_addr[g*12 +: 12]),
                    .o_debug_twiddle_addr(debug_twiddle_addr[g*12 +: 12]),
                    .rep1_i_re(r1_re[g]), .rep1_i_im(r1_im[g]),
                    .rep2_i_re(r2_re[g]), .rep2_i_im(r2_im[g]),
                    .rep1_o_re(r1_re[g+1]), .rep1_o_im(r1_im[g+1]),
                    .rep2_o_re(r2_re[g+1]), .rep2_o_im(r2_im[g+1]),
                    .rep_o_valid());
            end else begin : g_v3
                (* dont_touch = "true" *) sdf_stage_grp_v3 #(
                    .LOGN(LOGN), .STAGE(g), .MAX_LOGN(11), .DATA_W(DATA_W),
                    .INVERSE(INVERSE), .SCALE(SCALE_MASK[g]), .K(KG),
                    .KT(KTG), .MEM_BASE(GRP_TWMEM)
                ) u_stage (
                    .clk(clk), .rst(rst), .clear(sclr), .ce(ece),
                    .i_valid(v_p[g]), .i_sof(s_p[g]), .i_eof(e_p[g]),
                    .i_re(re_pipe[g]), .i_im(im_pipe[g]),
                    .o_valid(v_p[g+1]), .o_sof(s_p[g+1]), .o_eof(e_p[g+1]),
                    .o_re(ore), .o_im(oim),
                    .m_valid(), .m_sof(), .m_eof(),
                    .o_sample_overflow(sample_overflow_pipe[g]),
                    .o_overflow_re(), .o_overflow_im(),
                    .o_overflow_sticky(sticky_overflow_pipe[g]),
                    .o_debug_delay_addr(debug_delay_addr[g*12 +: 12]),
                    .o_debug_twiddle_addr(debug_twiddle_addr[g*12 +: 12]),
                    .rep1_i_re(r1_re[g]), .rep1_i_im(r1_im[g]),
                    .rep2_i_re(r2_re[g]), .rep2_i_im(r2_im[g]),
                    .rep1_o_re(r1_re[g+1]), .rep1_o_im(r1_im[g+1]),
                    .rep2_o_re(r2_re[g+1]), .rep2_o_im(r2_im[g+1]),
                    .rep_o_valid());
            end
            if (g < GLAST) begin : g_repass
                assign re_pipe[g+1] = ore;
                assign im_pipe[g+1] = oim;
            end
            assign debug_stage_valid[g] = v_p[g][0];
        end
    endgenerate

    // ---- group exit: the downstream stream is the majority of the three
    //      control copies (V2: identical to the voted datapath; V3: the
    //      prescribed boundary vote) ------------------------------------------
    assign g8_valid = (v_p[GLAST+1][0] & v_p[GLAST+1][1]) |
                      (v_p[GLAST+1][0] & v_p[GLAST+1][2]) |
                      (v_p[GLAST+1][1] & v_p[GLAST+1][2]);
    assign g8_sof   = (s_p[GLAST+1][0] & s_p[GLAST+1][1]) |
                      (s_p[GLAST+1][0] & s_p[GLAST+1][2]) |
                      (s_p[GLAST+1][1] & s_p[GLAST+1][2]);
    assign g8_eof   = (e_p[GLAST+1][0] & e_p[GLAST+1][1]) |
                      (e_p[GLAST+1][0] & e_p[GLAST+1][2]) |
                      (e_p[GLAST+1][1] & e_p[GLAST+1][2]);
    assign g8_re    = g_sdf_stage[GLAST].ore;
    assign g8_im    = g_sdf_stage[GLAST].oim;

    // ---- tail stages 9-10 on the voted ce / stage_clear ----------------------
    generate
        for (g = GLAST + 1; g < LOGN; g = g + 1) begin : g_tail_stage
            sdf_stage #(
                .LOGN(LOGN), .STAGE(g), .MAX_LOGN(11), .DATA_W(DATA_W),
                .INVERSE(INVERSE), .SCALE(SCALE_MASK[g])
            ) u_stage (
                .clk(clk), .rst(rst), .clear(sclr_m), .ce(ece_m),
                .i_valid(valid_pipe[g]), .i_sof(sof_pipe[g]), .i_eof(eof_pipe[g]),
                .i_re(re_pipe[g]), .i_im(im_pipe[g]),
                .o_valid(valid_pipe[g+1]), .o_sof(sof_pipe[g+1]),
                .o_eof(eof_pipe[g+1]),
                .o_re(re_pipe[g+1]), .o_im(im_pipe[g+1]),
                .o_sample_overflow(sample_overflow_pipe[g]),
                .o_overflow_re(), .o_overflow_im(),
                .o_overflow_sticky(sticky_overflow_pipe[g]),
                .o_debug_delay_addr(debug_delay_addr[g*12 +: 12]),
                .o_debug_twiddle_addr(debug_twiddle_addr[g*12 +: 12])
            );
            assign debug_stage_valid[g] = valid_pipe[g];
        end
    endgenerate
    // ---- exit position counter (single, on the voted stream) -----------------
    reg [10:0] pos_q;
    always @(posedge clk) begin
        if (rst || sclr_m)
            pos_q <= '0;
        else if (ece_m) begin
            if (g8_valid && g8_sof)
                pos_q <= 11'd1;
            else if (g8_valid && g8_eof)
                pos_q <= '0;
            else if (g8_valid)
                pos_q <= pos_q + 11'd1;
        end
    end

    // ---- the one group voter --------------------------------------------------
    wire vsticky;
    localparam integer KVOTE = (GRP_KV[8*GLAST +: 8] < 4) ? DATA_W
                                                        : GRP_KV[8*GLAST +: 8];
    (* dont_touch = "true" *) grp_vote #(
        .W(DATA_W), .K(KVOTE), .T(GRP_T), .COMP2(GRP_COMP2)
    ) u_vote (
        .clk(clk), .rst(rst), .clr(sclr_m), .clr_sticky(clear),
        .ce(ece_m),
        .i_valid(g8_valid), .i_sof(g8_sof), .i_eof(g8_eof), .i_pos(pos_q),
        .y_re(g8_re), .y_im(g8_im),
        .r1_re(r1_re[GLAST+1]), .r1_im(r1_im[GLAST+1]),
        .r2_re(r2_re[GLAST+1]), .r2_im(r2_im[GLAST+1]),
        .o_valid(valid_pipe[GLAST+1]), .o_sof(sof_pipe[GLAST+1]),
        .o_eof(eof_pipe[GLAST+1]),
        .o_re(re_pipe[GLAST+1]), .o_im(im_pipe[GLAST+1]),
        .sticky(vsticky), .fire_mon());

    assign rpr_sticky = vsticky;

    assign m_valid = valid_pipe[LOGN];
    assign m_sof = sof_pipe[LOGN];
    assign m_eof = eof_pipe[LOGN];
    assign m_re = re_pipe[LOGN];
    assign m_im = im_pipe[LOGN];
    assign stage_overflow = stage_overflow_sticky_q;
    assign overflow_sticky = |stage_overflow_sticky_q;

endmodule
