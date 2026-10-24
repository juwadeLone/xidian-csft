`timescale 1ns/1ps
`include "prot_vec.vh"

// TMR16: three identical copies of the CS application (chain, app FSM,
// scoreboard, seed tables) with one shared seed engine, one physical
// DDR port and a combinational majority vote on every outward signal.
// TASK_tmr_build.md sections 4-6; design page MODULE_RECORDS.md
// "TMR16 / step 0".
//
//   * replica (x3): csa_cs_app with SEED_EXT=1 -- everything inside the
//     application except the seed engine domain.
//   * engine domain (x1): k7_seed_engine_f64 plus par_dirty and the
//     eng_own write mux, exactly as csa_cs_app drives them in W16; the
//     _rb scan/repair flags are absent because the tables have no _rb
//     copy in this build (seed_window_driver RB_COPY, step 2).
//   * voter: tmr_voter on the concatenated outbound bundle -- pure
//     combinational majority, no register on handshake lines.
//   * inbound AXI signals and the muxed seed-write bus are broadcast to
//     all three replicas; clock, reset and control are shared.
//
// The port list matches csa_cs_app plus tmr_mismatch, so this module is
// a drop-in replacement at u_cs_app (the K7_MOD_KIND pattern) when the
// TMR_CS_APP define selects it.
module csa_cs_app_tmr #(
    parameter integer SIDE_LOG2 = 8,
    parameter [11:0] AZ_SCALE_MASK = 12'h680,
    parameter [11:0] RG_SCALE_MASK = 12'h6aa,
    parameter [11:0] RG_IFFT_SCALE_MASK = 12'h7ac,
    parameter [11:0] AZ_IFFT_SCALE_MASK = 12'h7fa,
    parameter integer ADDR_W = 36,
    parameter integer ID_W = 4,
    parameter [ADDR_W-1:0] BASE_B0 = {ADDR_W{1'b0}},
    parameter [ADDR_W-1:0] BASE_B1 = {ADDR_W{1'b0}},
    parameter [ADDR_W-1:0] BASE_B2 = {ADDR_W{1'b0}},
    parameter [ADDR_W-1:0] BASE_B3 = {ADDR_W{1'b0}},
    parameter [ADDR_W-1:0] BASE_OUT = {ADDR_W{1'b0}},
    parameter [63:0] EXPECTED_FOLD = 64'd0,
    parameter H1_MEM = "cs256_h1_seeds.mem",
    parameter H2_MEM = "cs256_h2_seeds.mem",
    parameter H3_MEM = "cs256_h3_seeds.mem",
    parameter C4_MEM = "cs256_c4_seeds.mem",
    parameter integer SYNC_SEED_ROM = 0
) (
    input  wire                     clk,
    input  wire                     rst,
    input  wire                     init_calib_complete,
    input  wire                     start,
    input  wire                     clear,

    // ---- per-frame CS parameter load (engine domain, single) ----
    input  wire                     cs_param_we,
    input  wire [3:0]               cs_param_idx,
    input  wire [31:0]              cs_param_w0,
    input  wire [31:0]              cs_param_w1,
    output wire                     cs_build_busy,
    output wire                     cs_build_done,
    output wire                     cs_build_error,
    output wire                     params_dirty,

    // ---- seed table write / read-back (engine domain, single) ----
    input  wire                     seed_wr_en,
    input  wire [1:0]               seed_wr_sel,
    input  wire [SIDE_LOG2-1:0]     seed_wr_idx,
    input  wire [95:0]              seed_wr_data,
    input  wire [1:0]               seed_wr_tgt,
    input  wire [1:0]               seed_rb_sel,
    input  wire [SIDE_LOG2-1:0]     seed_rb_idx,
    input  wire                     seed_rb_dbg,
    output wire [95:0]              seed_rb_data,

    // ---- replica status (voted) ----
    output wire                     calib_sync,
    output wire                     preload_done,
    output wire                     busy,
    output wire                     done,
    output wire                     error,
    output wire                     protocol_error,
    output wire                     overflow_sticky,
    output wire [2:0]               active_pass_id,
    output wire                     hash_ok,
    output wire [63:0]              fold,
    output wire                     image_done,
    output wire [2:0]               fsm_state,

    // ---- the TMR sticky mismatch flags (per replica, sim-visible) ----
    output wire [2:0]               tmr_mismatch,

    output wire [ID_W-1:0]          m_axi_awid,
    output wire [ADDR_W-1:0]        m_axi_awaddr,
    output wire [7:0]               m_axi_awlen,
    output wire [2:0]               m_axi_awsize,
    output wire [1:0]               m_axi_awburst,
    output wire                     m_axi_awvalid,
    input  wire                     m_axi_awready,
    output wire [255:0]             m_axi_wdata,
    output wire [31:0]              m_axi_wstrb,
    output wire                     m_axi_wlast,
    output wire                     m_axi_wvalid,
    input  wire                     m_axi_wready,
    input  wire [ID_W-1:0]          m_axi_bid,
    input  wire [1:0]               m_axi_bresp,
    input  wire                     m_axi_bvalid,
    output wire                     m_axi_bready,
    output wire [ID_W-1:0]          m_axi_arid,
    output wire [ADDR_W-1:0]        m_axi_araddr,
    output wire [7:0]               m_axi_arlen,
    output wire [2:0]               m_axi_arsize,
    output wire [1:0]               m_axi_arburst,
    output wire                     m_axi_arvalid,
    input  wire                     m_axi_arready,
    input  wire [ID_W-1:0]          m_axi_rid,
    input  wire [255:0]             m_axi_rdata,
    input  wire [1:0]               m_axi_rresp,
    input  wire                     m_axi_rlast,
    input  wire                     m_axi_rvalid,
    output wire                     m_axi_rready
);

    // ------------------------------------------------------------------
    // Engine domain (single): the binary64 seed engine, par_dirty and
    // the eng_own write mux -- the same logic csa_cs_app carries in
    // g_seed_int, minus the _rb scan/repair flags (no _rb copy exists in
    // this build, so the scan can never run and repair never fires).
    // ------------------------------------------------------------------
    wire                 eng_wr_en;
    wire [1:0]           eng_wr_sel;
    wire [SIDE_LOG2-1:0] eng_wr_idx;
    wire [95:0]          eng_wr_data;
    wire                 eng_busy, eng_done, eng_error;
    wire [95:0]          eng_dbg;

    reg start_q;
    wire start_pulse = start && !start_q;
    always @(posedge clk) begin
        if (rst) start_q <= 1'b0;
        else     start_q <= start;
    end

    reg par_dirty;
    always @(posedge clk) begin
        if (rst)
            par_dirty <= 1'b0;
        else if (cs_param_we)
            par_dirty <= 1'b1;      // new parameters: tables are stale
        else if (eng_done)
            par_dirty <= 1'b0;      // rebuild finished: tables are current
    end

    // rep_dirty is gone with the _rb scan, so eng_own loses that term;
    // the engine start likewise drops repair_start (it required a scan
    // mismatch, impossible without _rb).
    wire eng_own = par_dirty | eng_busy;

    // scalar TMR vote-mismatch flag, observed only (PROT_SCALAR=0)
    wire vote_mismatch;

    k7_seed_engine_f64 #(.SIDE_LOG2(SIDE_LOG2), .PROT_SCALAR(`PROT_SCALAR)) u_seed_engine_f64 (
        .clk(clk),
        .rst(rst),
        .start(start_pulse & par_dirty),
        .par_idx(cs_param_idx), .par_w0(cs_param_w0), .par_w1(cs_param_w1),
        .par_we(cs_param_we),

        .vote_mismatch(vote_mismatch),
        .wr_en(eng_wr_en), .wr_sel(eng_wr_sel), .wr_idx(eng_wr_idx),
        .wr_data(eng_wr_data),
        .busy(eng_busy), .done(eng_done), .error(eng_error),
        .dbg_addr(seed_rb_idx[7:0]), .dbg_data(eng_dbg)
    );

    assign cs_build_busy  = eng_busy;
    assign cs_build_done  = eng_done;
    assign cs_build_error = eng_error;
    assign params_dirty   = par_dirty;

    // The muxed write bus is broadcast to all three replicas.
    wire                 seed_wr_en_m   = eng_own ? eng_wr_en   : seed_wr_en;
    wire [1:0]           seed_wr_sel_m  = eng_own ? eng_wr_sel  : seed_wr_sel;
    wire [SIDE_LOG2-1:0] seed_wr_idx_m  = eng_own ? eng_wr_idx  : seed_wr_idx;
    wire [95:0]          seed_wr_data_m = eng_own ? eng_wr_data : seed_wr_data;
    wire [1:0]           seed_wr_tgt_m  = eng_own ? 2'b11       : seed_wr_tgt;

    // No _rb copies in this build: the host table read-back port has no
    // source, only the engine-debug path remains (deviation per task s7).
    // Since 2026-10-13 sel 3 of the debug view carries the frame word and
    // the fold word (below, after the voter).
    wire [95:0] dbg_sel3;
    assign seed_rb_data = seed_rb_dbg ?
                         ((seed_rb_sel == 2'd3) ? dbg_sel3 : eng_dbg) :
                         96'd0;

    // ------------------------------------------------------------------
    // Three replicas (csa_cs_app with the engine domain externalised).
    // ------------------------------------------------------------------
    wire [ID_W-1:0]   r_awid   [0:2];
    wire [ADDR_W-1:0] r_awaddr [0:2];
    wire [7:0]        r_awlen  [0:2];
    wire [2:0]        r_awsize [0:2];
    wire [1:0]        r_awburst[0:2];
    wire              r_awvalid[0:2];
    wire [255:0]      r_wdata  [0:2];
    wire [31:0]       r_wstrb  [0:2];
    wire              r_wlast  [0:2];
    wire              r_wvalid [0:2];
    wire              r_bready [0:2];
    wire [ID_W-1:0]   r_arid   [0:2];
    wire [ADDR_W-1:0] r_araddr [0:2];
    wire [7:0]        r_arlen  [0:2];
    wire [2:0]        r_arsize [0:2];
    wire [1:0]        r_arburst[0:2];
    wire              r_arvalid[0:2];
    wire              r_rready [0:2];
    wire              r_busy   [0:2];
    wire              r_done   [0:2];
    wire              r_error  [0:2];
    wire              r_proto  [0:2];
    wire              r_ovf    [0:2];
    wire [3:0]        r_ovf4   [0:2];
    wire [2:0]        r_pass   [0:2];
    wire              r_hashok [0:2];
    wire [63:0]       r_fold   [0:2];
    wire              r_imgdone[0:2];
    wire [2:0]        r_fsm    [0:2];
    wire              r_calib  [0:2];
    wire              r_preload[0:2];
    wire              r_wrun   [0:2];
    wire [2:0]        r_cstate [0:2];

    genvar gi;
    // ------------------------------------------------------------------
    // TMR16 alignment (task s6): each replica exports its scheduling
    // vector {app fsm, want_run, chain state, pass id}; the bitwise
    // majority is the voted trajectory.  A replica whose own vector
    // differs gets align=1 and rejoins the voted state in the same
    // cycle -- the latest a diverged replica can be forced back is the
    // start of the next pass (state_c=ST_PRECLEAR/ST_LAUNCH issues the
    // engine clear and the launch pulse together with the majority).
    // The vote uses only the three replicas' vectors, never one
    // replica's own signal.  Each replica has its own alignment voter
    // (g_rep[g].u_avote, tmr_align_vote): the three scheduling vectors fan
    // out to all three voters, so one wrong voter misaligns only the
    // replica it feeds.  DONT_TOUCH / keep_hierarchy keep synthesis from
    // merging the three identical majority circuits.
    // ------------------------------------------------------------------
    localparam integer SCHED_W = 10;
    wire [SCHED_W-1:0] sched   [0:2];
    wire [SCHED_W-1:0] sched_vg[0:2];
    wire [2:0]         align_rep;

    for (gi = 0; gi < 3; gi = gi + 1) begin : g_sched
        assign sched[gi] = {r_fsm[gi], r_wrun[gi], r_cstate[gi],
                            r_pass[gi]};
    end

    generate
    for (gi = 0; gi < 3; gi = gi + 1) begin : g_rep
        (* DONT_TOUCH = "TRUE", keep_hierarchy = "yes" *)
        tmr_align_vote #(.W(SCHED_W), .OWN(gi)) u_avote (
            .s0(sched[0]), .s1(sched[1]), .s2(sched[2]),
            .sched_v(sched_vg[gi]), .align(align_rep[gi])
        );
        (* DONT_TOUCH = "TRUE" *)
        csa_cs_app #(
            .SIDE_LOG2(SIDE_LOG2), .AZ_SCALE_MASK(AZ_SCALE_MASK),
            .RG_SCALE_MASK(RG_SCALE_MASK),
            .RG_IFFT_SCALE_MASK(RG_IFFT_SCALE_MASK),
            .AZ_IFFT_SCALE_MASK(AZ_IFFT_SCALE_MASK),
            .ADDR_W(ADDR_W), .ID_W(ID_W),
            .BASE_B0(BASE_B0), .BASE_B1(BASE_B1), .BASE_B2(BASE_B2),
            .BASE_B3(BASE_B3), .BASE_OUT(BASE_OUT),
            .EXPECTED_FOLD(EXPECTED_FOLD),
            .H1_MEM(H1_MEM), .H2_MEM(H2_MEM),
            .H3_MEM(H3_MEM), .C4_MEM(C4_MEM),
            .SYNC_SEED_ROM(SYNC_SEED_ROM),
            .SEED_EXT(1), .SEED_RB(0)
        ) u_rep (
            .clk(clk), .rst(rst),
            .init_calib_complete(init_calib_complete),
            .start(start), .clear(clear),
            .cs_param_we(1'b0), .cs_param_idx(4'd0),
            .cs_param_w0(32'd0), .cs_param_w1(32'd0),
            .cs_build_busy(), .cs_build_done(), .cs_build_error(),
            .params_dirty(),
            .seed_wr_en(seed_wr_en_m), .seed_wr_sel(seed_wr_sel_m),
            .seed_wr_idx(seed_wr_idx_m), .seed_wr_data(seed_wr_data_m),
            .seed_wr_tgt(seed_wr_tgt_m),
            .seed_rb_sel(2'd0), .seed_rb_idx({SIDE_LOG2{1'b0}}),
            .seed_rb_dbg(1'b0),
            .seed_rb_data(),
            .ext_eng_busy(eng_busy), .ext_eng_done(eng_done),
            .ext_eng_error(eng_error), .ext_par_dirty(par_dirty),
            .align(align_rep[gi]),
            .align_app_state(sched_vg[gi][9:7]),
            .align_want_run(sched_vg[gi][6]),
            .align_chain_state(sched_vg[gi][5:3]),
            .align_pass(sched_vg[gi][2:0]),
            .want_run_o(r_wrun[gi]), .chain_state_o(r_cstate[gi]),
            .calib_sync(r_calib[gi]), .preload_done(r_preload[gi]),
            .busy(r_busy[gi]), .done(r_done[gi]), .error(r_error[gi]),
            .protocol_error(r_proto[gi]), .overflow_sticky(r_ovf[gi]),
            .ovf4_o(r_ovf4[gi]),
            .active_pass_id(r_pass[gi]), .hash_ok(r_hashok[gi]),
            .fold(r_fold[gi]), .image_done(r_imgdone[gi]),
            .fsm_state(r_fsm[gi]),
            .m_axi_awid(r_awid[gi]), .m_axi_awaddr(r_awaddr[gi]),
            .m_axi_awlen(r_awlen[gi]), .m_axi_awsize(r_awsize[gi]),
            .m_axi_awburst(r_awburst[gi]),
            .m_axi_awvalid(r_awvalid[gi]), .m_axi_awready(m_axi_awready),
            .m_axi_wdata(r_wdata[gi]), .m_axi_wstrb(r_wstrb[gi]),
            .m_axi_wlast(r_wlast[gi]), .m_axi_wvalid(r_wvalid[gi]),
            .m_axi_wready(m_axi_wready),
            .m_axi_bid(m_axi_bid), .m_axi_bresp(m_axi_bresp),
            .m_axi_bvalid(m_axi_bvalid), .m_axi_bready(r_bready[gi]),
            .m_axi_arid(r_arid[gi]), .m_axi_araddr(r_araddr[gi]),
            .m_axi_arlen(r_arlen[gi]), .m_axi_arsize(r_arsize[gi]),
            .m_axi_arburst(r_arburst[gi]),
            .m_axi_arvalid(r_arvalid[gi]), .m_axi_arready(m_axi_arready),
            .m_axi_rid(m_axi_rid), .m_axi_rdata(m_axi_rdata),
            .m_axi_rresp(m_axi_rresp), .m_axi_rlast(m_axi_rlast),
            .m_axi_rvalid(m_axi_rvalid), .m_axi_rready(r_rready[gi])
        );
    end
    endgenerate

    // ------------------------------------------------------------------
    // Vote: every outward replica signal, bitwise majority, no register.
    // Inbound AXI signals above are broadcast, not voted.
    // ------------------------------------------------------------------
    localparam integer VOTE_W = 345 + 55 + 79 + 4;   // AXI W/B + AXI R + status + pass overflow flags

    wire [VOTE_W-1:0] rep_bus [0:2];
    wire [VOTE_W-1:0] voted_bus;

    for (gi = 0; gi < 3; gi = gi + 1) begin : g_bus
        assign rep_bus[gi] = {
            r_awid[gi], r_awaddr[gi], r_awlen[gi], r_awsize[gi],
            r_awburst[gi], r_awvalid[gi], r_wdata[gi], r_wstrb[gi],
            r_wlast[gi], r_wvalid[gi], r_bready[gi],
            r_arid[gi], r_araddr[gi], r_arlen[gi], r_arsize[gi],
            r_arburst[gi], r_arvalid[gi], r_rready[gi],
            r_busy[gi], r_done[gi], r_error[gi], r_proto[gi], r_ovf[gi],
            r_pass[gi], r_hashok[gi], r_fold[gi], r_imgdone[gi],
            r_fsm[gi], r_calib[gi], r_preload[gi], r_ovf4[gi]
        };
    end

    wire flag_a, flag_b, flag_c;
    wire [3:0] ovf4_v;               // voted {p6, p4, p3, p1}
    reg  clear_q;
    wire clear_pulse = clear && !clear_q;
    always @(posedge clk) begin
        if (rst) clear_q <= 1'b0;
        else     clear_q <= clear;
    end
    tmr_voter #(.W(VOTE_W)) u_vote (
        .clk(clk), .rst(rst), .clr(start_pulse),
        .in_a(rep_bus[0]), .in_b(rep_bus[1]), .in_c(rep_bus[2]),
        .voted(voted_bus), .flag_a(flag_a), .flag_b(flag_b),
        .flag_c(flag_c)
    );
    assign tmr_mismatch = {flag_c, flag_b, flag_a};

    assign {m_axi_awid, m_axi_awaddr, m_axi_awlen, m_axi_awsize,
            m_axi_awburst, m_axi_awvalid, m_axi_wdata, m_axi_wstrb,
            m_axi_wlast, m_axi_wvalid, m_axi_bready,
            m_axi_arid, m_axi_araddr, m_axi_arlen, m_axi_arsize,
            m_axi_arburst, m_axi_arvalid, m_axi_rready,
            busy, done, error, protocol_error, overflow_sticky,
            active_pass_id, hash_ok, fold, image_done,
            fsm_state, calib_sync, preload_done, ovf4_v} = voted_bus;

    // ---- per-frame overflow flags and the on-chip result check (readout
    // only; no feedback anywhere).  Each pass engine's overflow flag is
    // sticky inside the engine and cleared by the chain before every pass,
    // so after a frame only the last pass's flag is left, and at the start
    // of a frame the last pass's flag of the previous frame is still up.
    // ovf_seen collects, per engine, a flag that was raised during this
    // frame: a flag that is up at the frame start is ignored (ovf_stale)
    // until it has been seen low once.  Cleared by the application clear and
    // at every frame start.  This copy works on the voted signals; the
    // clear is the rising edge of the wrapper's clear input (the replicas
    // take the same edge).
    reg [3:0] ovf_seen, ovf_stale;   // {p6, p4, p3, p1}
    reg       frm_seen, frm_done;
    always @(posedge clk) begin
        if (rst || clear_pulse) begin
            ovf_seen  <= 4'd0;
            // all four "left over": the replicas take the same clear edge and
            // clear their engines two clocks later, so a flag of the cleared
            // frame is still up here; it is counted again only after it has
            // been seen low
            ovf_stale <= 4'hF;
            frm_seen  <= 1'b0;
            frm_done  <= 1'b0;
        end else if (start_pulse) begin
            ovf_seen  <= 4'd0;
            ovf_stale <= ovf4_v;
            frm_seen  <= 1'b1;
            frm_done  <= 1'b0;
        end else begin
            ovf_stale <= ovf_stale & ovf4_v;
            ovf_seen  <= ovf_seen | (ovf4_v & ~ovf_stale);
            if (image_done)
                frm_done <= 1'b1;
        end
    end
    // [95:88] = 8'hA5 and [87:80] = interface revision 1 mark a build that has
    // this word, the fold word, the application clear command and the busy
    // that holds in the error state; older builds read 0 here.
    localparam [7:0] IF_REV = 8'd1;
    // [10:8] the outward voter's per-replica mismatch flags {c, b, a}: set
    // when that replica's outward signals differed from the vote since the
    // frame start (tmr_voter); they had no host readout before.
    wire [95:0] frame_word = {8'hA5, IF_REV, 69'd0, tmr_mismatch, hash_ok,
                              overflow_sticky, ovf_seen, frm_done, frm_seen};
    wire [95:0] fold_word  = {32'd0, fold};
    // CS_RB debug view, sel 3, by row index:
    //   idx 8 -> frame word: [0] a frame was started since reset / clear;
    //            [1] that frame completed (image_done);  [5:2] a node of the
    //            pass saturated during the frame, {p6, p4, p3, p1};  [6] the
    //            chain's overflow flag as it is now;  [7] hash_ok (the fold
    //            equals the build's EXPECTED_FOLD parameter);  [10:8]
    //            replica mismatch flags {c, b, a};  [87:80]
    //            interface revision;  [95:88] 8'hA5.
    //   idx 9 -> [63:0] the XOR fold of the output frame's 64-bit words, as
    //            the result check read it from memory after the frame.
    assign dbg_sel3 = (seed_rb_idx == 8) ? frame_word :
                      (seed_rb_idx == 9) ? fold_word : 96'd0;
endmodule
