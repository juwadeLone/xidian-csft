`timescale 1ns/1ps
`include "prot_vec.vh"

// Processing core: three AXI masters on clk_125m, exclusive mux, then
// the unchanged csa_cs_chain_top.  Compile-time bases come from the
// board top; start/clear stay on dedicated wires; seeds/window stay on
// the DUT streams.
// Host-input processing core (simplified 2026-09-11).
//
// The verification-era fabric preload (STREAM_RAW pattern / RAW ROM) and its
// preload state machine are gone -- their mission is complete and the
// evidence is archived in results/k7_port.  The production contract is:
//
//   1. host writes the RAW frame (pulse-major cpx32) into B0 over UDP;
//   2. host sends CS_PARAM flags[7] (=0x90) -> run one frame on B0;
//   3. host reads OUT (=4*FRAME_BYTES) -- range-major image.
//
// B0 is transposed in place by the chain's P0, so step 1 repeats for every
// frame.  calib wait -> ST_IDLE -> (start) -> RUN -> CHECK -> IDLE.
module csa_cs_app #(
    parameter integer SIDE_LOG2 = 8,
    parameter [11:0] AZ_SCALE_MASK = 12'h680,       // P1 azimuth FFT
    parameter [11:0] RG_SCALE_MASK = 12'h6aa,       // P3 range FFT
    parameter [11:0] RG_IFFT_SCALE_MASK = 12'h7ac,  // P4 range IFFT
    parameter [11:0] AZ_IFFT_SCALE_MASK = 12'h7fa,  // P6 azimuth IFFT
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

    // ---- P3: per-frame CS parameter load (cmd_seq, v2 nine words) ----
    input  wire                     cs_param_we,
    input  wire [3:0]               cs_param_idx,
    input  wire [31:0]              cs_param_w0,
    input  wire [31:0]              cs_param_w1,
    output wire                     cs_build_busy,
    output wire                     cs_build_done,
    output wire                     cs_build_error,
    output wire                     params_dirty,

    // ---- P3: seed-builder write / read-back ports (tables ROM -> RAM) ----
    input  wire                     seed_wr_en,
    input  wire [1:0]               seed_wr_sel,
    input  wire [SIDE_LOG2-1:0]     seed_wr_idx,
    input  wire [95:0]              seed_wr_data,
    input  wire [1:0]               seed_wr_tgt,    // protection_cmp: CS_ROW flags[5:4]
    input  wire [1:0]               seed_rb_sel,
    input  wire [SIDE_LOG2-1:0]     seed_rb_idx,
    input  wire                     seed_rb_dbg,
    output wire [95:0]              seed_rb_data,

    output wire                     calib_sync,
    output wire                     preload_done,   // legacy: constant 1
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
    localparam [2:0] ST_WAIT_CALIB = 3'd0;
    localparam [2:0] ST_IDLE       = 3'd1;
    localparam [2:0] ST_RUN        = 3'd2;
    localparam [2:0] ST_CHECK      = 3'd3;
    localparam [2:0] ST_ERROR      = 3'd5;
    localparam [1:0] GRANT_CHAIN   = 2'd1;
    localparam [1:0] GRANT_SCORE   = 2'd2;

    reg [2:0] state;
    assign fsm_state = state;

    reg [2:0] calib_sr;
    always @(posedge clk) begin
        if (rst)
            calib_sr <= 3'b000;
        else
            calib_sr <= {calib_sr[1:0], init_calib_complete};
    end
    assign calib_sync = calib_sr[2];

    reg start_q;
    reg clear_q;
    wire start_pulse = start && !start_q;
    wire clear_pulse = clear && !clear_q;
    always @(posedge clk) begin
        if (rst) begin
            start_q <= 1'b0;
            clear_q <= 1'b0;
        end else begin
            start_q <= start;
            clear_q <= clear;
        end
    end

    reg chain_start;
    reg score_start;
    reg app_clear;
    reg image_done_q;
    assign image_done = image_done_q;
    assign preload_done = 1'b1;   // no preload stage any more

    // ------------------------------------------------------------------
    // P3: on-board seed builder (route A).
    //
    // cmd_seq raises cs_build_start (= this module's `start`) for the frame
    // and hands over the nine v2 parameter words on cs_param_*.  When any
    // parameter was written (params_dirty) the builder rewrites all four
    // seed tables first and the chain starts on its done pulse; with
    // params_dirty=0 nothing is written and the $readmemh preload is used
    // verbatim -- the bit-for-bit back-compat path.
    //
    // ext_* is tied off permanently: the row algebra is the only source.
    // ------------------------------------------------------------------
    // The unrolled soft-float seed builder is no longer instantiated.  Its
    // ~30 fully expanded 128-bit restoring dividers and 64-iteration sqrt
    // chains produce a netlist orders of magnitude larger than the part, and
    // Vivado never finishes RTL Optimization Phase 2 on it.  Tables arrive
    // from the host instead (cmd_seq row write / read-back); with
    // params_dirty tied low the chain runs off the $readmemh preload exactly
    // as before, which is the bit-for-bit back-compat path.
    // The unrolled builder is gone, but table building itself is not: it now
    // runs as k7_seed_engine_f64, a sequential binary64 micro-sequence in
    // front of the CORDIC.  Every parameter is used in binary64 (lam and Tp
    // arrive as full binary64 in two words) and the sequence is the same one
    // the frozen tables were generated with, so the board reproduces all
    // 24576 frozen fields bit for bit -- see scripts/gen_seed_microcode.py
    // --check and sim/tb/tb_k7_seed_engine_f64.sv.
    //
    // When the host sends the five CS_PARAM packets the tables are rebuilt on
    // the board; when it sends none, params_dirty stays low and nothing is
    // written, so the chain runs off the $readmemh preload exactly as the
    // host-uploaded tables left it (bit-for-bit back-compat path).
    wire                 eng_wr_en;
    wire [1:0]           eng_wr_sel;
    wire [SIDE_LOG2-1:0] eng_wr_idx;
    wire [95:0]          eng_wr_data;
    wire                 eng_busy, eng_done, eng_error;
    wire [95:0]          eng_dbg;

    reg par_dirty;
    always @(posedge clk) begin
        if (rst)
            par_dirty <= 1'b0;
        else if (cs_param_we)
            par_dirty <= 1'b1;      // new parameters: tables are stale
        else if (eng_done)
            par_dirty <= 1'b0;      // rebuild finished: tables are current
    end

    // The engine owns the write port while it rebuilds, the host owns it the
    // rest of the time, so uploading the very same tables by hand still works.
    // ---- protection_cmp B4: rb-compare scan + repair (contract s3.2, D2)
    // TAB_EN bit order {c4,h3,h2,h1} matches seed_window_driver.scan_neq.
    localparam [3:0] TAB_EN = {(`PROT_TAB_C4 != 0), (`PROT_TAB_H3 != 0),
                               (`PROT_TAB_H2 != 0), (`PROT_TAB_H1 != 0)};
    localparam ANY_TAB = (TAB_EN != 4'd0);
    reg  [15:0] scan_period;
    reg  [15:0] scan_ctr;
    reg         scan_pend;
    reg         rep_dirty;
    wire        scan_done, scan_mismatch_any, scan_row_mismatch;
    wire        scan_due   = ANY_TAB && (scan_period != 16'd0) && (scan_ctr == 16'd0);
    wire        scan_start = start_pulse && scan_due && !par_dirty;
    wire        repair_start = scan_done && scan_mismatch_any;
    always @(posedge clk) begin
        if (rst) begin
            scan_period <= 16'd1;       // default: compare at every GO
            scan_ctr    <= 16'd0;
            scan_pend   <= 1'b0;
            rep_dirty   <= 1'b0;
        end else begin
            if (cs_param_we && cs_param_idx == 4'd11)
                scan_period <= cs_param_w0[15:0];
            if (start_pulse && ANY_TAB && scan_period != 16'd0)
                scan_ctr <= (scan_ctr == 16'd0) ? scan_period - 16'd1 : scan_ctr - 16'd1;
            if (scan_start)        scan_pend <= 1'b1;
            else if (scan_done)    scan_pend <= 1'b0;
            if (repair_start)      rep_dirty <= 1'b1;
            else if (eng_done)     rep_dirty <= 1'b0;
        end
    end

    wire eng_own = par_dirty | eng_busy | rep_dirty;

    // scalar TMR vote-mismatch flag
    wire        vote_mismatch;

    k7_seed_engine_f64 #(.SIDE_LOG2(SIDE_LOG2), .PROT_SCALAR(`PROT_SCALAR)) u_seed_engine_f64 (
        .clk(clk),
        .rst(rst),
        .start((start_pulse & par_dirty) | repair_start),
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

    wire                 seed_wr_en_m   = eng_own ? eng_wr_en   : seed_wr_en;
    wire [1:0]           seed_wr_sel_m  = eng_own ? eng_wr_sel  : seed_wr_sel;
    wire [SIDE_LOG2-1:0] seed_wr_idx_m  = eng_own ? eng_wr_idx  : seed_wr_idx;
    wire [95:0]          seed_wr_data_m = eng_own ? eng_wr_data : seed_wr_data;
    wire [1:0]           seed_wr_tgt_m  = eng_own ? 2'b11       : seed_wr_tgt;

    // One run request per frame; held until the tables are current.
    reg  want_run;
    reg  bld_done_d;
    wire build_fresh  = cs_build_done & ~bld_done_d;
    wire chain_go_req = want_run & (~params_dirty | build_fresh)
                                & ~scan_pend & ~rep_dirty
                                & calib_sync & (state == ST_IDLE);
    always @(posedge clk) begin
        if (rst) begin
            want_run   <= 1'b0;
            bld_done_d <= 1'b0;
        end else begin
            bld_done_d <= cs_build_done;
            if (start_pulse)     want_run <= 1'b1;
            if (chain_go_req)    want_run <= 1'b0;
        end
    end

    wire chain_busy, chain_done, chain_error, chain_proto, chain_ovf;
    wire [3:0] chain_ovf4;       // {p6, p4, p3, p1}
    wire score_busy, score_done, score_error;
    wire [2:0] pass_id;
    assign active_pass_id = pass_id;
    // Busy also in the error state while the chain or the result check is
    // still working (2026-10-13).  The run control enters the error state on
    // the first error and does not stop the chain; a pass engine that is in
    // the middle of its pass goes on reading and writing memory until the
    // pass ends.  busy used to drop on entering the error state, and the
    // board core gives the memory bus back to the host commands when busy is
    // low -- possibly in the middle of a transfer.  cmd_seq answers the run
    // command (NAK) once busy is low, so that answer now comes when the
    // chain has come to rest; a chain that never does is covered by
    // cmd_seq's run time-out.
    assign busy = (state == ST_RUN) || (state == ST_CHECK) ||
                  ((state == ST_ERROR) && (chain_busy || score_busy));
    assign done = (state == ST_IDLE);
    assign error = (state == ST_ERROR) || chain_error || score_error;
    assign protocol_error = chain_proto;
    assign overflow_sticky = chain_ovf;

    reg [1:0] grant;
    wire allow_clear = !chain_busy || (state == ST_ERROR);

    always @(posedge clk) begin
        if (rst) begin
            state <= ST_WAIT_CALIB;
            chain_start <= 1'b0;
            score_start <= 1'b0;
            app_clear <= 1'b0;
            image_done_q <= 1'b0;
            grant <= GRANT_CHAIN;
        end else begin
            chain_start <= 1'b0;
            score_start <= 1'b0;
            app_clear <= 1'b0;
            image_done_q <= 1'b0;

            if (clear_pulse && allow_clear) begin
                app_clear <= 1'b1;
                state <= calib_sync ? ST_IDLE : ST_WAIT_CALIB;
                grant <= GRANT_CHAIN;
            end else begin
                case (state)
                    ST_WAIT_CALIB: begin
                        grant <= GRANT_CHAIN;
                        if (calib_sync) state <= ST_IDLE;
                    end
                    ST_RUN: begin
                        grant <= GRANT_CHAIN;
                        if (chain_error || chain_proto)
                            state <= ST_ERROR;
                        else if (chain_done && !chain_busy) begin
                            state <= ST_CHECK;
                            grant <= GRANT_SCORE;
                            score_start <= 1'b1;
                        end
                    end
                    ST_CHECK: begin
                        grant <= GRANT_SCORE;
                        if (score_error)
                            state <= ST_ERROR;
                        else if (score_done && !score_busy) begin
                            state <= ST_IDLE;
                            image_done_q <= 1'b1;
                            grant <= GRANT_CHAIN;
                        end
                    end
                    ST_IDLE: begin
                        grant <= GRANT_CHAIN;
                        if (chain_go_req) begin
                            // B0 already holds the host frame; there is no
                            // preload to skip and no one-cycle indirection
                            // needed any more.  The chain transposes B0 in
                            // place during P0, so the host re-uploads before
                            // every frame.
                            state <= ST_RUN;
                            grant <= GRANT_CHAIN;
                            chain_start <= 1'b1;
                        end
                    end
                    default: begin
                        grant <= GRANT_CHAIN;
                        if (clear_pulse && allow_clear) begin
                            app_clear <= 1'b1;
                            state <= calib_sync ? ST_IDLE : ST_WAIT_CALIB;
                        end
                    end
                endcase
            end
        end
    end

    wire pre_seed_valid, pre_seed_ready;
    wire post_seed_valid, post_seed_ready;
    wire window_coeff_valid;
    wire [31:0] pre_t0, pre_d1, pre_d2;
    wire [31:0] post_t0, post_d1, post_d2;
    wire signed [29:0] window_coeff;
    wire window_coeff_ready = 1'b0;   // no window stream in the trimmed chain
    wire [95:0] tab_rb_data;

    seed_window_driver #(
        .SIDE_LOG2(SIDE_LOG2),
        .H1_MEM(H1_MEM), .H2_MEM(H2_MEM),
        .H3_MEM(H3_MEM), .C4_MEM(C4_MEM),
        .SYNC_SEED_ROM(SYNC_SEED_ROM)
    ) u_seeds (
        .clk(clk), .rst(rst), .clear(app_clear),
        // The seed stream also runs in the error state while the chain is
        // still busy (2026-10-13): the pass under way can then finish and
        // the chain comes to rest by itself, instead of waiting for seeds
        // without end with its memory transfers half done.  No further
        // pass is started in the error state.
        .enable((state == ST_RUN) || ((state == ST_ERROR) && chain_busy)),
        .active_pass_id(pass_id),
        .pre_seed_valid(pre_seed_valid), .pre_seed_ready(pre_seed_ready),
        .pre_seed_theta0(pre_t0), .pre_seed_delta1(pre_d1),
        .pre_seed_delta2(pre_d2),
        .post_seed_valid(post_seed_valid), .post_seed_ready(post_seed_ready),
        .post_seed_theta0(post_t0), .post_seed_delta1(post_d1),
        .post_seed_delta2(post_d2),
        .window_coeff_valid(window_coeff_valid),
        .window_coeff_ready(window_coeff_ready),
        .window_coeff(window_coeff),
        .wr_en(seed_wr_en_m), .wr_sel(seed_wr_sel_m),
        .wr_idx(seed_wr_idx_m), .wr_data(seed_wr_data_m),
        .wr_tgt(seed_wr_tgt_m), .scan_start(scan_start), .scan_en(TAB_EN),
        .scan_busy(), .scan_done(scan_done),
        .scan_mismatch_any(scan_mismatch_any),
        .scan_row_mismatch(scan_row_mismatch),
        .rb_sel(seed_rb_sel), .rb_idx(seed_rb_idx), .rb_data(tab_rb_data)
    );
    // ---- per-frame overflow flags and the on-chip result check (readout
    // only; no feedback anywhere).  Each pass engine's overflow flag is
    // sticky inside the engine and cleared by the chain before every pass,
    // so after a frame only the last pass's flag is left, and at the start
    // of a frame the last pass's flag of the previous frame is still up.
    // ovf_seen collects, per engine, a flag that was raised during this
    // frame: a flag that is up at the frame start is ignored (ovf_stale)
    // until it has been seen low once.  Cleared by the application clear and
    // at every frame start.
    reg [3:0] ovf_seen, ovf_stale;   // {p6, p4, p3, p1}
    reg       frm_seen, frm_done;
    always @(posedge clk) begin
        if (rst || app_clear) begin
            ovf_seen  <= 4'd0;
            ovf_stale <= 4'd0;
            frm_seen  <= 1'b0;
            frm_done  <= 1'b0;
        end else if (start_pulse) begin
            ovf_seen  <= 4'd0;
            ovf_stale <= chain_ovf4;
            frm_seen  <= 1'b1;
            frm_done  <= 1'b0;
        end else begin
            ovf_stale <= ovf_stale & chain_ovf4;
            ovf_seen  <= ovf_seen | (chain_ovf4 & ~ovf_stale);
            if (image_done_q)
                frm_done <= 1'b1;
        end
    end
    // [95:88] = 8'hA5 and [87:80] = interface revision 1 mark a build that has
    // this word, the fold word, the application clear command and the busy
    // that holds in the error state; older builds read 0 here.
    localparam [7:0] IF_REV = 8'd1;
    wire [95:0] frame_word = {8'hA5, IF_REV, 72'd0, hash_ok, chain_ovf,
                              ovf_seen, frm_done, frm_seen};
    wire [95:0] fold_word  = {32'd0, fold};

    // CS_RB debug view: seed_rb_dbg=1 & sel=3 returns, by row index (the
    // protection status words and the first-error words of the protected
    // builds, idx 1 to 7, have no source in this chain and read 0):
    //   idx 8 -> frame word: [0] a frame was started since reset / clear;
    //            [1] that frame completed (image_done);  [5:2] a node of the
    //            pass saturated during the frame, {p6, p4, p3, p1};  [6] the
    //            chain's overflow flag as it is now;  [7] hash_ok (the fold
    //            equals the build's EXPECTED_FOLD parameter);  [87:80]
    //            interface revision;  [95:88] 8'hA5.
    //   idx 9 -> [63:0] the XOR fold of the output frame's 64-bit words, as
    //            the result check read it from memory after the frame.
    // sel 3 has no seed-engine debug meaning, so nothing is shadowed.
    wire [95:0] dbg_sel3 = (seed_rb_idx == 8) ? frame_word :
                           (seed_rb_idx == 9) ? fold_word : 96'd0;
    assign seed_rb_data = seed_rb_dbg ?
                         ((seed_rb_sel == 2'd3) ? dbg_sel3 : eng_dbg) :
                         tab_rb_data;

    wire [ID_W-1:0]   p_awid, c_awid, k_awid;
    wire [ADDR_W-1:0] p_awaddr, c_awaddr, k_awaddr;
    wire [7:0]        p_awlen, c_awlen, k_awlen;
    wire [2:0]        p_awsize, c_awsize, k_awsize;
    wire [1:0]        p_awburst, c_awburst, k_awburst;
    wire              p_awvalid, c_awvalid, k_awvalid;
    wire              p_awready, c_awready, k_awready;
    wire [255:0]      p_wdata, c_wdata, k_wdata;
    wire [31:0]       p_wstrb, c_wstrb, k_wstrb;
    wire              p_wlast, c_wlast, k_wlast;
    wire              p_wvalid, c_wvalid, k_wvalid;
    wire              p_wready, c_wready, k_wready;
    wire [ID_W-1:0]   p_bid, c_bid, k_bid;
    wire [1:0]        p_bresp, c_bresp, k_bresp;
    wire              p_bvalid, c_bvalid, k_bvalid;
    wire              p_bready, c_bready, k_bready;
    wire [ID_W-1:0]   p_arid, c_arid, k_arid;
    wire [ADDR_W-1:0] p_araddr, c_araddr, k_araddr;
    wire [7:0]        p_arlen, c_arlen, k_arlen;
    wire [2:0]        p_arsize, c_arsize, k_arsize;
    wire [1:0]        p_arburst, c_arburst, k_arburst;
    wire              p_arvalid, c_arvalid, k_arvalid;
    wire              p_arready, c_arready, k_arready;
    wire [ID_W-1:0]   p_rid, c_rid, k_rid;
    wire [255:0]      p_rdata, c_rdata, k_rdata;
    wire [1:0]        p_rresp, c_rresp, k_rresp;
    wire              p_rlast, c_rlast, k_rlast;
    wire              p_rvalid, c_rvalid, k_rvalid;
    wire              p_rready, c_rready, k_rready;

    assign p_arid = {ID_W{1'b0}};
    assign p_araddr = {ADDR_W{1'b0}};
    assign p_arlen = 8'd0;
    assign p_arsize = 3'd0;
    assign p_arburst = 2'd0;
    assign p_arvalid = 1'b0;
    assign p_rready = 1'b0;
    // preload slot (p_*) is gone: tie the mux's slot-0 inputs off.
    assign p_awid = {ID_W{1'b0}};
    assign p_awaddr = {ADDR_W{1'b0}};
    assign p_awlen = 8'd0;
    assign p_awsize = 3'd0;
    assign p_awburst = 2'd0;
    assign p_awvalid = 1'b0;
    assign p_wdata = 256'd0;
    assign p_wstrb = 32'd0;
    assign p_wlast = 1'b0;
    assign p_wvalid = 1'b0;
    assign p_bready = 1'b0;

    assign k_awid = {ID_W{1'b0}};
    assign k_awaddr = {ADDR_W{1'b0}};
    assign k_awlen = 8'd0;
    assign k_awsize = 3'd0;
    assign k_awburst = 2'd0;
    assign k_awvalid = 1'b0;
    assign k_wdata = 256'd0;
    assign k_wstrb = 32'd0;
    assign k_wlast = 1'b0;
    assign k_wvalid = 1'b0;
    assign k_bready = 1'b0;

    // GR17: with the build-time define GR16S_CHAIN the grouped-RPR chain top
    // replaces the unchanged W17 chain; without the define this is the
    // original csa_cs_chain_top instantiation, bit for bit.
`ifdef GR16S_CHAIN
    wire [11:0] chain_rpr_sticky;
    (* keep = "true" *) wire [11:0] rpr_sticky_obs = chain_rpr_sticky;
    csa_cs_chain_top_grp #(
`else
    csa_cs_chain_top #(
`endif
        .SIDE_LOG2(SIDE_LOG2), .ADDR_W(ADDR_W), .ID_W(ID_W),
        .AZ_SCALE_MASK(AZ_SCALE_MASK), .RG_SCALE_MASK(RG_SCALE_MASK),
        .RG_IFFT_SCALE_MASK(RG_IFFT_SCALE_MASK), .AZ_IFFT_SCALE_MASK(AZ_IFFT_SCALE_MASK)
    ) u_dut (
        .clk(clk), .rst(rst), .clear(app_clear), .start(chain_start),

        .base_b0(BASE_B0), .base_b1(BASE_B1), .base_b2(BASE_B2),
        .base_b3(BASE_B3), .base_out(BASE_OUT),
        .pre_seed_valid(pre_seed_valid), .pre_seed_ready(pre_seed_ready),
        .pre_seed_theta0(pre_t0), .pre_seed_delta1(pre_d1),
        .pre_seed_delta2(pre_d2),
        .post_seed_valid(post_seed_valid), .post_seed_ready(post_seed_ready),
        .post_seed_theta0(post_t0), .post_seed_delta1(post_d1),
        .post_seed_delta2(post_d2),
        .active_pass_id(pass_id),
        .busy(chain_busy), .done(chain_done), .error(chain_error),
        .protocol_error(chain_proto), .overflow_sticky(chain_ovf),
        .o_eng_ovf(chain_ovf4),
`ifdef GR16S_CHAIN
        .rpr_sticky(chain_rpr_sticky),
`endif
        .m_axi_awid(c_awid), .m_axi_awaddr(c_awaddr), .m_axi_awlen(c_awlen),
        .m_axi_awsize(c_awsize), .m_axi_awburst(c_awburst),
        .m_axi_awvalid(c_awvalid), .m_axi_awready(c_awready),
        .m_axi_wdata(c_wdata), .m_axi_wstrb(c_wstrb), .m_axi_wlast(c_wlast),
        .m_axi_wvalid(c_wvalid), .m_axi_wready(c_wready),
        .m_axi_bid(c_bid), .m_axi_bresp(c_bresp),
        .m_axi_bvalid(c_bvalid), .m_axi_bready(c_bready),
        .m_axi_arid(c_arid), .m_axi_araddr(c_araddr), .m_axi_arlen(c_arlen),
        .m_axi_arsize(c_arsize), .m_axi_arburst(c_arburst),
        .m_axi_arvalid(c_arvalid), .m_axi_arready(c_arready),
        .m_axi_rid(c_rid), .m_axi_rdata(c_rdata), .m_axi_rresp(c_rresp),
        .m_axi_rlast(c_rlast), .m_axi_rvalid(c_rvalid),
        .m_axi_rready(c_rready)
    );

    axi_out_scoreboard #(
        .SIDE_LOG2(SIDE_LOG2), .ADDR_W(ADDR_W), .ID_W(ID_W),
        .AXI_ID(4'h2), .EXPECTED_FOLD(EXPECTED_FOLD)
    ) u_score (
        .clk(clk), .rst(rst), .clear(app_clear), .start(score_start),
        .base_out(BASE_OUT),
        .busy(score_busy), .done(score_done), .error(score_error),
        .hash_ok(hash_ok), .fold(fold),
        .m_axi_arid(k_arid), .m_axi_araddr(k_araddr), .m_axi_arlen(k_arlen),
        .m_axi_arsize(k_arsize), .m_axi_arburst(k_arburst),
        .m_axi_arvalid(k_arvalid), .m_axi_arready(k_arready),
        .m_axi_rid(k_rid), .m_axi_rdata(k_rdata), .m_axi_rresp(k_rresp),
        .m_axi_rlast(k_rlast), .m_axi_rvalid(k_rvalid),
        .m_axi_rready(k_rready)
    );

    axi_m_mux_3to1 #(.ADDR_W(ADDR_W), .ID_W(ID_W)) u_mux (
        .select(grant),
        .s_axi_awid({k_awid, c_awid, p_awid}),
        .s_axi_awaddr({k_awaddr, c_awaddr, p_awaddr}),
        .s_axi_awlen({k_awlen, c_awlen, p_awlen}),
        .s_axi_awsize({k_awsize, c_awsize, p_awsize}),
        .s_axi_awburst({k_awburst, c_awburst, p_awburst}),
        .s_axi_awvalid({k_awvalid, c_awvalid, p_awvalid}),
        .s_axi_awready({k_awready, c_awready, p_awready}),
        .s_axi_wdata({k_wdata, c_wdata, p_wdata}),
        .s_axi_wstrb({k_wstrb, c_wstrb, p_wstrb}),
        .s_axi_wlast({k_wlast, c_wlast, p_wlast}),
        .s_axi_wvalid({k_wvalid, c_wvalid, p_wvalid}),
        .s_axi_wready({k_wready, c_wready, p_wready}),
        .s_axi_bid({k_bid, c_bid, p_bid}),
        .s_axi_bresp({k_bresp, c_bresp, p_bresp}),
        .s_axi_bvalid({k_bvalid, c_bvalid, p_bvalid}),
        .s_axi_bready({k_bready, c_bready, p_bready}),
        .s_axi_arid({k_arid, c_arid, p_arid}),
        .s_axi_araddr({k_araddr, c_araddr, p_araddr}),
        .s_axi_arlen({k_arlen, c_arlen, p_arlen}),
        .s_axi_arsize({k_arsize, c_arsize, p_arsize}),
        .s_axi_arburst({k_arburst, c_arburst, p_arburst}),
        .s_axi_arvalid({k_arvalid, c_arvalid, p_arvalid}),
        .s_axi_arready({k_arready, c_arready, p_arready}),
        .s_axi_rid({k_rid, c_rid, p_rid}),
        .s_axi_rdata({k_rdata, c_rdata, p_rdata}),
        .s_axi_rresp({k_rresp, c_rresp, p_rresp}),
        .s_axi_rlast({k_rlast, c_rlast, p_rlast}),
        .s_axi_rvalid({k_rvalid, c_rvalid, p_rvalid}),
        .s_axi_rready({k_rready, c_rready, p_rready}),
        .m_axi_awid(m_axi_awid), .m_axi_awaddr(m_axi_awaddr),
        .m_axi_awlen(m_axi_awlen), .m_axi_awsize(m_axi_awsize),
        .m_axi_awburst(m_axi_awburst), .m_axi_awvalid(m_axi_awvalid),
        .m_axi_awready(m_axi_awready), .m_axi_wdata(m_axi_wdata),
        .m_axi_wstrb(m_axi_wstrb), .m_axi_wlast(m_axi_wlast),
        .m_axi_wvalid(m_axi_wvalid), .m_axi_wready(m_axi_wready),
        .m_axi_bid(m_axi_bid), .m_axi_bresp(m_axi_bresp),
        .m_axi_bvalid(m_axi_bvalid), .m_axi_bready(m_axi_bready),
        .m_axi_arid(m_axi_arid), .m_axi_araddr(m_axi_araddr),
        .m_axi_arlen(m_axi_arlen), .m_axi_arsize(m_axi_arsize),
        .m_axi_arburst(m_axi_arburst), .m_axi_arvalid(m_axi_arvalid),
        .m_axi_arready(m_axi_arready), .m_axi_rid(m_axi_rid),
        .m_axi_rdata(m_axi_rdata), .m_axi_rresp(m_axi_rresp),
        .m_axi_rlast(m_axi_rlast), .m_axi_rvalid(m_axi_rvalid),
        .m_axi_rready(m_axi_rready)
    );
endmodule
