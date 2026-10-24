`timescale 1ns/1ps

// Algorithm-parameter stream for csa_cs_chain_top.  Same contract as
// tb_csa_cs_chain: one seed handshake per vector, window handshake per
// sample, muxed by active_pass_id.
//
// SYNC_SEED_ROM=0: combinational table read (CS256).
// SYNC_SEED_ROM=1: registered table read so 4096x96 stays in block RAM.
//
// P3 (v2) ROM -> RAM: the four 96-bit tables are now writable.  The
// $readmemh preload is KEPT -- it is the back-compat default (params_dirty=0
// must behave bit-for-bit like the existing bitstream) and the simulation
// default.  The write port is driven by cs_seed_builder and is only active
// while the builder is busy, so it never races the imaging chain's read port.
// The read port timing and the pass -> table mapping are unchanged.
module seed_window_driver #(
    parameter integer SIDE_LOG2 = 8,
    parameter H1_MEM = "cs256_h1_seeds.mem",
    parameter H2_MEM = "cs256_h2_seeds.mem",
    parameter H3_MEM = "cs256_h3_seeds.mem",
    parameter C4_MEM = "cs256_c4_seeds.mem",
    parameter integer SYNC_SEED_ROM = 0,
    // TMR16: RB_COPY=1 keeps the four _rb read-back copies, the pre-frame
    // compare scan and the rb port (the W16 single-chain behaviour);
    // RB_COPY=0 (a TMR replica) generates none of them -- 22 BRAM fewer
    // per replica -- and ties scan_* / rb_data off.
    parameter integer RB_COPY = 1
) (
    input  wire                     clk,
    input  wire                     rst,
    input  wire                     clear,
    input  wire                     enable,
    input  wire [2:0]               active_pass_id,

    output reg                      pre_seed_valid,
    input  wire                     pre_seed_ready,
    output reg  [31:0]              pre_seed_theta0,
    output reg  [31:0]              pre_seed_delta1,
    output reg  [31:0]              pre_seed_delta2,

    output reg                      post_seed_valid,
    input  wire                     post_seed_ready,
    output reg  [31:0]              post_seed_theta0,
    output reg  [31:0]              post_seed_delta1,
    output reg  [31:0]              post_seed_delta2,

    output reg                      window_coeff_valid,
    input  wire                     window_coeff_ready,
    output wire signed [29:0]       window_coeff,

    // ---- P3 seed-builder write port (0 = h1, 1 = h2, 2 = h3, 3 = c4) ----
    input  wire                     wr_en,
    input  wire [1:0]               wr_sel,
    input  wire [SIDE_LOG2-1:0]         wr_idx,
    input  wire [95:0]              wr_data,
    // ---- protection_cmp B4 ----
    input  wire [1:0]               wr_tgt,     // 00/11 both, 01 main, 10 rb
    input  wire                     scan_start,
    input  wire [3:0]               scan_en,    // {c4, h3, h2, h1}
    output wire                     scan_busy,
    output wire                     scan_done,
    output wire                     scan_mismatch_any,
    output wire                     scan_row_mismatch,

    // ---- P3 table read-back port (1-clock latency; see note below) ----
    input  wire [1:0]               rb_sel,
    input  wire [SIDE_LOG2-1:0]         rb_idx,
    output reg  [95:0]              rb_data
);
    localparam integer SIDE = 1 << SIDE_LOG2;
    localparam integer SAMPLES = SIDE * SIDE;
    localparam integer IDX_W = SIDE_LOG2;
    localparam integer WIN_W = 2 * SIDE_LOG2;
    // rpr_noroll: the half-turn roll seed {0, 0x80000000, 0} is no longer
    // sent; outside its pass a seed port carries zeros.
    localparam [95:0] NO_SEED = 96'd0;
    // Max positive Q1.29; 30'h20000000 is -1.0 on signed [29:0].
    assign window_coeff = 30'h1FFFFFFF;

    // ram_style, not rom_style: with a write port Vivado must not infer a ROM.
    // Main arrays: 1 write port + 1 read port (registered, g_sync_data) --
    // the canonical simple-dual-port BRAM shape.
    (* ram_style = "block" *) reg [95:0] h1_seeds [0:SIDE-1];
    (* ram_style = "block" *) reg [95:0] h2_seeds [0:SIDE-1];
    (* ram_style = "block" *) reg [95:0] h3_seeds [0:SIDE-1];
    (* ram_style = "block" *) reg [95:0] c4_seeds [0:SIDE-1];

    initial begin
        $readmemh(H1_MEM, h1_seeds);
        $readmemh(H2_MEM, h2_seeds);
        $readmemh(H3_MEM, h3_seeds);
        $readmemh(C4_MEM, c4_seeds);
    end

    // protection_cmp B4 write target (CS_ROW flags[5:4]; the engine forces 11)
    wire wr_main = (wr_tgt != 2'b10);

    // Write steering.  One write port, shared by the four tables.
    // BRAM-inference shape (UG901): every write to an array lives in this one
    // clocked block; read ports live in their own blocks.  Do NOT route the
    // write through a task -- Vivado 2018.3 does not recognise task-based
    // array writes as RAM and dissolves all four tables into 787k FFs
    // (Place 30-640).
    // Main arrays: canonical if(write) mem[addr] <= din; shape.
    always @(posedge clk) begin
        if (wr_en && wr_main) begin
            case (wr_sel)
                2'd0: h1_seeds[wr_idx] <= wr_data;
                2'd1: h2_seeds[wr_idx] <= wr_data;
                2'd2: h3_seeds[wr_idx] <= wr_data;
                default: c4_seeds[wr_idx] <= wr_data;
            endcase
        end
    end

    // The _rb compare scan borrows the chain read ports while the chain is
    // idle; with RB_COPY=0 there is nothing to compare and the ports always
    // serve the seed counters.
    wire        rb_scan_busy;
    wire [SIDE_LOG2-1:0] rb_scan_row;
    wire [3:0]  scan_neq;              // {c4,h3,h2,h1}, valid with scan_cmp_v
    // Registered _rb reads; module scope so g_sync_data can compare them.
    reg  [95:0] rb_q0, rb_q1, rb_q2, rb_q3;

    // Read-back copies (P4 CMD_TABRD source).  BRAM has only two ports; a
    // third read port makes ram_style="block" Infeasible (Synth 8-6849) and
    // the tool silently falls back to LUTRAM.  So the rb port gets its own
    // physical copies, written in lockstep -- 4 more 2048x96 arrays, i.e.
    // 6 BRAM36 per table pair instead of LUTRAM sprawl.
    generate
    if (RB_COPY != 0) begin : g_rb
        (* ram_style = "block" *) reg [95:0] h1_rb [0:SIDE-1];
        (* ram_style = "block" *) reg [95:0] h2_rb [0:SIDE-1];
        (* ram_style = "block" *) reg [95:0] h3_rb [0:SIDE-1];
        (* ram_style = "block" *) reg [95:0] c4_rb [0:SIDE-1];

        initial begin
            // Same preload into the read-back copies so a never-written
            // board still reads back the preload through CMD_TABRD
            // (back-compat).
            $readmemh(H1_MEM, h1_rb);
            $readmemh(H2_MEM, h2_rb);
            $readmemh(H3_MEM, h3_rb);
            $readmemh(C4_MEM, c4_rb);
        end

        // protection_cmp B4 write target
        wire wr_rb = (wr_tgt != 2'b01);

        // Read-back copies: identical shape, separate always block per
        // array so each is an independent 1W1R SDP RAM.
        always @(posedge clk) begin
            if (wr_en && wr_rb && (wr_sel == 2'd0)) h1_rb[wr_idx] <= wr_data;
        end
        always @(posedge clk) begin
            if (wr_en && wr_rb && (wr_sel == 2'd1)) h2_rb[wr_idx] <= wr_data;
        end
        always @(posedge clk) begin
            if (wr_en && wr_rb && (wr_sel == 2'd2)) h3_rb[wr_idx] <= wr_data;
        end
        always @(posedge clk) begin
            if (wr_en && wr_rb && (wr_sel == 2'd3)) c4_rb[wr_idx] <= wr_data;
        end

        // ---- protection_cmp B4: rb-compare scan ----------------------
        reg                 scan_busy_q = 1'b0;
        reg                 scan_cmp_v  = 1'b0;
        reg [SIDE_LOG2-1:0] scan_row    = {SIDE_LOG2{1'b0}};
        reg                 scan_acc    = 1'b0;
        wire [SIDE_LOG2-1:0] rb_addr = scan_busy_q ? scan_row : rb_idx;
        always @(posedge clk) begin
            if (rst) begin
                scan_busy_q <= 1'b0;
                scan_cmp_v  <= 1'b0;
                scan_row    <= {SIDE_LOG2{1'b0}};
            end else begin
                scan_cmp_v <= scan_busy_q;
                if (scan_start && !scan_busy_q) begin
                    scan_busy_q <= 1'b1;
                    scan_row    <= {SIDE_LOG2{1'b0}};
                end else if (scan_busy_q) begin
                    if (scan_row == {SIDE_LOG2{1'b1}})
                        scan_busy_q <= 1'b0;
                    scan_row <= scan_row + 1'b1;
                end
            end
        end
        wire [3:0] scan_hit = scan_cmp_v ? (scan_en & scan_neq) : 4'd0;
        always @(posedge clk) begin
            if (rst || (scan_start && !scan_busy_q)) scan_acc <= 1'b0;
            else if (|scan_hit)                       scan_acc <= 1'b1;
        end
        assign rb_scan_busy      = scan_busy_q;
        assign rb_scan_row       = scan_row;
        assign scan_busy         = scan_busy_q | scan_cmp_v;
        assign scan_done         = scan_cmp_v && !scan_busy_q;   // after the last row
        assign scan_mismatch_any = scan_acc | (|scan_hit);
        assign scan_row_mismatch = |scan_hit;

        // Registered read of the rb copies; rb is the P4 CMD_TABRD
        // source.  Read all four copies at rb_idx and mux AFTER the
        // registers: keeps each array a plain dout <= mem[raddr]; read
        // so ram_style="block" holds (a case()-selected read breaks
        // Vivado 2018.3 BRAM inference).
        always @(posedge clk) begin
            rb_q0 <= h1_rb[rb_addr];
            rb_q1 <= h2_rb[rb_addr];
            rb_q2 <= h3_rb[rb_addr];
            rb_q3 <= c4_rb[rb_addr];
        end
        always @* begin
            case (rb_sel)
                2'd0: rb_data = rb_q0;
                2'd1: rb_data = rb_q1;
                2'd2: rb_data = rb_q2;
                default: rb_data = rb_q3;
            endcase
        end
    end else begin : g_no_rb
        // TMR replica: no _rb copies and no compare logic at all.
        assign rb_scan_busy      = 1'b0;
        assign rb_scan_row       = {SIDE_LOG2{1'b0}};
        assign scan_busy         = 1'b0;
        assign scan_done         = 1'b0;
        assign scan_mismatch_any = 1'b0;
        assign scan_row_mismatch = 1'b0;
        always @* rb_data = 96'd0;
    end
    endgenerate

    reg [2:0] pass_q;
    reg [IDX_W:0] pre_idx;
    reg [IDX_W:0] post_idx;
    reg [WIN_W:0] win_idx;

    // scan borrows the chain read port while the chain is idle
    wire [IDX_W-1:0] pre_addr  = rb_scan_busy ? rb_scan_row[IDX_W-1:0] : pre_idx[IDX_W-1:0];
    wire [IDX_W-1:0] post_addr = rb_scan_busy ? rb_scan_row[IDX_W-1:0] : post_idx[IDX_W-1:0];

    // rpr_noroll: no half-turn rolls and no identity window.  Pre seeds only
    // in P6 (C4), post seeds only in P1/P3/P4 (H1/H2/H3), no window stream.
    wire pass_pre = (active_pass_id == 3'd6);
    wire pass_post = (active_pass_id == 3'd1) || (active_pass_id == 3'd3) ||
                     (active_pass_id == 3'd4);
    wire pass_win = 1'b0;

    generate
        if (SYNC_SEED_ROM != 0) begin : g_sync_data
            // Registered reads so 4096x96 stays in block RAM.  Mux the
            // registered words; do not combo-read the arrays.  The builder
            // write is folded into the same clocked block so the arrays stay
            // a single-port-write / read-first RAM.
            reg [95:0] h1_q, h2_q, h3_q, c4_q;
            always @(posedge clk) begin
                c4_q <= c4_seeds[pre_addr];
                h1_q <= h1_seeds[post_addr];
                h2_q <= h2_seeds[post_addr];
                h3_q <= h3_seeds[post_addr];
            end
            if (RB_COPY != 0) begin : g_scan_cmp
                assign scan_neq = {c4_q != rb_q3, h3_q != rb_q2,
                                   h2_q != rb_q1, h1_q != rb_q0};
            end else begin
                assign scan_neq = 4'd0;
            end
            always @* begin
                {pre_seed_theta0, pre_seed_delta1, pre_seed_delta2} = NO_SEED;
                if (active_pass_id == 3'd6)
                    {pre_seed_theta0, pre_seed_delta1, pre_seed_delta2} = c4_q;
                {post_seed_theta0, post_seed_delta1, post_seed_delta2} =
                    NO_SEED;
                case (active_pass_id)
                    3'd1: {post_seed_theta0, post_seed_delta1,
                           post_seed_delta2} = h1_q;
                    3'd3: {post_seed_theta0, post_seed_delta1,
                           post_seed_delta2} = h2_q;
                    3'd4: {post_seed_theta0, post_seed_delta1,
                           post_seed_delta2} = h3_q;
                    default: begin
                    end
                endcase
            end
        end else begin : g_combo_data
            // combinational-read model: no scan support (board uses SYNC)
            assign scan_neq = 4'd0;
            always @* begin
                {pre_seed_theta0, pre_seed_delta1, pre_seed_delta2} = NO_SEED;
                if (active_pass_id == 3'd6)
                    {pre_seed_theta0, pre_seed_delta1, pre_seed_delta2} =
                        c4_seeds[pre_idx[IDX_W-1:0]];
                {post_seed_theta0, post_seed_delta1, post_seed_delta2} =
                    NO_SEED;
                case (active_pass_id)
                    3'd1: {post_seed_theta0, post_seed_delta1,
                           post_seed_delta2} = h1_seeds[post_idx[IDX_W-1:0]];
                    3'd3: {post_seed_theta0, post_seed_delta1,
                           post_seed_delta2} = h2_seeds[post_idx[IDX_W-1:0]];
                    3'd4: {post_seed_theta0, post_seed_delta1,
                           post_seed_delta2} = h3_seeds[post_idx[IDX_W-1:0]];
                    default: begin
                    end
                endcase
            end
        end
    endgenerate

    always @(posedge clk) begin
        if (rst || clear) begin
            pass_q <= 3'd0;
            pre_idx <= '0;
            post_idx <= '0;
            win_idx <= '0;
            pre_seed_valid <= 1'b0;
            post_seed_valid <= 1'b0;
            window_coeff_valid <= 1'b0;
        end else if (!enable) begin
            pre_seed_valid <= 1'b0;
            post_seed_valid <= 1'b0;
            window_coeff_valid <= 1'b0;
        end else if (active_pass_id != pass_q) begin
            pass_q <= active_pass_id;
            pre_idx <= '0;
            post_idx <= '0;
            win_idx <= '0;
            // SYNC ROM: data for idx 0 appears next clock.
            pre_seed_valid <= pass_pre && (SYNC_SEED_ROM == 0);
            post_seed_valid <= pass_post && (SYNC_SEED_ROM == 0);
            window_coeff_valid <= pass_win;
        end else begin
            if (pre_seed_valid && pre_seed_ready && (pre_idx < SIDE)) begin
                if (pre_idx == SIDE - 1) begin
                    pre_seed_valid <= 1'b0;
                    pre_idx <= pre_idx + 1'b1;
                end else begin
                    pre_idx <= pre_idx + 1'b1;
                    if (SYNC_SEED_ROM != 0)
                        pre_seed_valid <= 1'b0;
                end
            end else
                pre_seed_valid <= pass_pre && (pre_idx < SIDE);

            if (post_seed_valid && post_seed_ready && (post_idx < SIDE)) begin
                if (post_idx == SIDE - 1) begin
                    post_seed_valid <= 1'b0;
                    post_idx <= post_idx + 1'b1;
                end else begin
                    post_idx <= post_idx + 1'b1;
                    if (SYNC_SEED_ROM != 0)
                        post_seed_valid <= 1'b0;
                end
            end else
                post_seed_valid <= pass_post && (post_idx < SIDE);

            if (window_coeff_valid && window_coeff_ready &&
                (win_idx < SAMPLES)) begin
                if (win_idx == SAMPLES - 1) begin
                    window_coeff_valid <= 1'b0;
                    win_idx <= win_idx + 1'b1;
                end else
                    win_idx <= win_idx + 1'b1;
            end else
                window_coeff_valid <= pass_win && (win_idx < SAMPLES);
        end
    end
endmodule
