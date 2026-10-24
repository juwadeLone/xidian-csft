`timescale 1ns/1ps
// K7 CS seed generation, stage 2: the on-board table sequencer.
//
// Wraps k7_f64_seq_core and turns it into the four seed tables the CS chain
// reads: latch the parameter words, run the setup program (which derives the
// runtime constants from the parameters), then run each table's row program
// 2048 times and write the three encoded phase words out on the same write
// port the host upload path uses.
//
// Parameter words (protocol v3, five CS_PARAM packets, two 32-bit words each,
// carried in the command's addr/length fields):
//     idx 0 : w0 = C    (f32)   w1 = B   (f32)
//     idx 1 : w0 = v    (f32)   w1 = Da  (f32)
//     idx 2 : w0 = Rs   (f32)   w1 = lam[63:32]
//     idx 3 : w0 = lam[31:0]    w1 = Tp[63:32]
//     idx 4 : w0 = Tp[31:0]     w1 = unused
// C, B, v, Da and Rs are binary32 on the link (they are exactly representable
// enough: the reference tables were produced from the same binary32 values);
// lam and Tp are carried as full binary64 because the c4 table's phase spans
// 5.9e5 turns and a binary32 lam moves its encoded word by up to 5e6 counts.
// With lam and Tp exact, the f64 sequence reproduces all 24576 frozen table
// fields bit for bit (scripts/gen_seed_microcode.py --check).
//
// The write port is the same shape as the host upload path
// (en / sel / idx / 96-bit data), so whoever owns it is a mux decision.

module k7_seed_engine_f64 #(
    parameter integer SIDE_LOG2 = 11,
    parameter integer PROT_SCALAR = 0      // protection_cmp B5: 1 = TMR
)(
    input  wire                     clk,
    input  wire                     rst,
    input  wire                     start,          // 1-cycle: regenerate all four tables
    input  wire [3:0]               par_idx,        // CS_PARAM packet index
    input  wire [31:0]              par_w0,
    input  wire [31:0]              par_w1,
    input  wire                     par_we,
    // ---- scalar TMR vote-mismatch flag
    output wire                     vote_mismatch,
    output reg                      wr_en,
    output reg  [1:0]               wr_sel,
    output reg  [SIDE_LOG2-1:0]     wr_idx,
    output reg  [95:0]              wr_data,
    output reg                      busy,
    output reg                      done,
    output reg                      error,
    // combinational debug read (host observability via CS_RB flags[3])
    input  wire [7:0]               dbg_addr,
    output reg  [95:0]              dbg_data
);

    localparam integer N_ROWS = (1 << SIDE_LOG2);

    // Program layout, from configs/f64mc_manifest.json.
    // Program lengths must track configs/f64mc_manifest.json.
    localparam [9:0] OFF_SETUP = 10'd0,   LEN_SETUP = 8'd37;
    localparam [9:0] OFF_H1    = 10'd256, LEN_H1    = 8'd161;
    localparam [9:0] OFF_H2    = 10'd512, LEN_H2    = 8'd104;
    localparam [9:0] OFF_H3    = 10'd640, LEN_H3    = 8'd113;
    localparam [9:0] OFF_C4    = 10'd768, LEN_C4    = 8'd122;

    // One program is at most ~16k cycles (h1: 2591 cycles of IP latency plus a
    // few cycles of handshake per operation).  200k leaves a 10x margin and
    // still turns a stuck handshake into an error instead of a silent hang.
    localparam [17:0] WD_LIMIT = 18'd200_000;

    // ------------------------------------------------------- parameter words
    // CS_PARAM wire layout (v3, ten words in five packets).  Every value is
    // transferred exactly: C/B/v/Da/Rs are binary32-representable as they
    // stand, and lam/Tp are carried as the two halves of their binary64 --
    // high word first, low word second -- so the board sees the identical
    // double the reference tables were generated with (no hi/lo residual).
    //
    //   pkt  w0                       w1
    //   0    C   (f32)                B    (f32)
    //   1    v   (f32)                Da   (f32)
    //   2    Rs  (f32)                lam_hi = bits[63:32]
    //   3    lam_lo = bits[31:0]      Tp_hi  = bits[63:32]
    //   4    Tp_lo = bits[31:0]       spare (0)
    reg [31:0] pw [0:9];

    always @(posedge clk) begin
        if (par_we) begin
            case (par_idx)
                4'd0: begin pw[0] <= par_w0; pw[1] <= par_w1; end
                4'd1: begin pw[2] <= par_w0; pw[3] <= par_w1; end
                4'd2: begin pw[4] <= par_w0; pw[8] <= par_w1; end
                4'd3: begin pw[9] <= par_w0; pw[6] <= par_w1; end
                4'd4: begin pw[7] <= par_w0; pw[5] <= par_w1; end
                default: ;
            endcase
        end
    end

    // Voted view of the scalars: every read below uses pwv.
    wire [31:0] pwv [0:9];
    wire [9:0]  sc_dis;
    genvar gi;
    generate
        if (PROT_SCALAR != 0) begin : g_tmr
            reg [31:0] pw_b [0:9];
            reg [31:0] pw_c [0:9];
            always @(posedge clk) begin
                if (par_we) begin
                case (par_idx)
                    4'd0: begin pw_b[0] <= par_w0; pw_b[1] <= par_w1; end
                    4'd1: begin pw_b[2] <= par_w0; pw_b[3] <= par_w1; end
                    4'd2: begin pw_b[4] <= par_w0; pw_b[8] <= par_w1; end
                    4'd3: begin pw_b[9] <= par_w0; pw_b[6] <= par_w1; end
                    4'd4: begin pw_b[7] <= par_w0; pw_b[5] <= par_w1; end
                    default: ;
                endcase
                end
            end
            always @(posedge clk) begin
                if (par_we) begin
                case (par_idx)
                    4'd0: begin pw_c[0] <= par_w0; pw_c[1] <= par_w1; end
                    4'd1: begin pw_c[2] <= par_w0; pw_c[3] <= par_w1; end
                    4'd2: begin pw_c[4] <= par_w0; pw_c[8] <= par_w1; end
                    4'd3: begin pw_c[9] <= par_w0; pw_c[6] <= par_w1; end
                    4'd4: begin pw_c[7] <= par_w0; pw_c[5] <= par_w1; end
                    default: ;
                endcase
                end
            end
            for (gi = 0; gi < 10; gi = gi + 1) begin : g_vote
                assign pwv[gi] = (pw[gi] & pw_b[gi]) | (pw[gi] & pw_c[gi]) |
                                 (pw_b[gi] & pw_c[gi]);
                assign sc_dis[gi] = |((pw[gi] ^ pw_b[gi]) | (pw[gi] ^ pw_c[gi]));
            end
        end else begin : g_plain
            for (gi = 0; gi < 10; gi = gi + 1) begin : g_pass
                assign pwv[gi]    = pw[gi];
                assign sc_dis[gi] = 1'b0;
            end
        end
    endgenerate
    assign vote_mismatch = start && (|sc_dis);

    // binary32 -> binary64, exact.  Subnormals, infinities and NaNs cannot
    // arise from this link; rather than widen them wrongly, flag them.
    function [63:0] f32_to_f64;
        input [31:0] x;
        reg        s;
        reg [7:0]  e8;
        reg [22:0] m23;
        begin
            s   = x[31];
            e8  = x[30:23];
            m23 = x[22:0];
            if (e8 == 8'd0) begin
                f32_to_f64 = 64'd0;                    // +-0 (checked separately)
            end else begin
                f32_to_f64 = {s, (11'd1023 + {3'd0, e8} - 11'd127), m23, 29'd0};
            end
        end
    endfunction

    wire par_bad = par_we & ((par_w0[30:23] == 8'd0 && par_w0[22:0] != 23'd0)
                          || (par_w1[30:23] == 8'd0 && par_w1[22:0] != 23'd0)
                          || (par_w0[30:23] == 8'd255)
                          || (par_w1[30:23] == 8'd255));

    // ------------------------------------------------------- param program
    reg [2:0] par_i;
    reg [7:0] p_addr;
    reg [63:0] p_data;

    always @(*) begin
        p_addr = {5'd0, par_i};
        case (par_i)
            3'd0: p_data = f32_to_f64(pwv[0]);          // C
            3'd1: p_data = {pwv[8], pwv[9]};             // lam = {hi, lo}
            3'd2: p_data = f32_to_f64(pwv[1]);          // B
            3'd3: p_data = 64'd0;                      // Fs: no link word;
                                                       // the setup program derives
                                                       // it and does not copy it back
            3'd4: p_data = {pwv[6], pwv[7]};             // Tp  = {hi, lo}
            3'd5: p_data = f32_to_f64(pwv[2]);          // v
            3'd6: p_data = f32_to_f64(pwv[3]);          // Da
            default: p_data = f32_to_f64(pwv[4]);       // Rs
        endcase
    end

    // --------------------------------------------------- sequencer states
    localparam [2:0] W_IDLE  = 3'd0;
    localparam [2:0] W_PAR   = 3'd1;
    localparam [2:0] W_SETUP = 3'd2;
    localparam [2:0] W_ROW   = 3'd3;
    localparam [2:0] W_WR    = 3'd4;
    localparam [2:0] W_DONE  = 3'd5;
    localparam [2:0] W_ERR   = 3'd6;

    reg [2:0]  st;

    // ------------------------------------------------------------- core hook
    // par_we is raised for eight cycles: one per parameter register.
    wire par_load = (st == W_PAR) & (par_i <= 3'd7);

    reg         core_run;
    reg  [9:0]  core_off;
    reg  [7:0]  core_len;
    reg  [11:0] core_row;
    wire        core_busy, core_done;
    wire        fin_we;
    wire [1:0]  fin_word;
    wire [31:0] fin_data;
    wire [63:0] core_dbg;

    k7_f64_seq_core u_core (
        .clk(clk),
        .rst(rst),
        .run(core_run),
        .prog_off(core_off),
        .prog_len(core_len),
        .row_idx(core_row),
        .par_addr(p_addr),
        .par_data(p_data),
        .par_we(par_load),
        .busy(core_busy),
        .done(core_done),
        .fin_we(fin_we),
        .fin_word(fin_word),
        .fin_data(fin_data),
        .dbg_addr(dbg_addr - 8'd64),
        .dbg_data(core_dbg)
    );

    // Host-visible debug map (CS_RB flags[3]=1, addr = dbg_addr):
    //   0..9   -> {64'd0, pw[addr]}            parameter word file
    //   10     -> engine status {st,tbl,r,wd,busy,done,error,fin_seen}
    //   11     -> last fin {fin_word, fin_data}
    //   12     -> row_sh (last row written)
    //   64..   -> core dbg_data passthrough (rf / cst / core state)
    reg        fin_seen;
    reg [31:0] fin_last;
    reg [1:0]  fin_wlast;
    always @(posedge clk) begin
        if (rst) begin
            fin_seen <= 1'b0; fin_last <= 32'd0; fin_wlast <= 2'd0;
        end else if (fin_we) begin
            fin_seen <= 1'b1; fin_last <= fin_data; fin_wlast <= fin_word;
        end
    end

    // --------------------------------------------------------------- staging
    // row_sh lives in the sequencer block below: a second always block here
    // made it multi-driven, and Vivado 2018.3 kept the constant-clear driver
    // while dropping the fin writes entirely (Synth 8-6858) -- sim kept both
    // blocks so every unit test passed while the board wrote all-zero rows.
    reg [95:0] row_sh;

    // ------------------------------------------------------------- sequencer
    reg        sub;                 // 0 = issue the core run, 1 = wait for it
    reg [1:0]  tbl;
    reg [10:0] r;
    reg [17:0] wd;

    always @(posedge clk) begin
        if (rst) begin
            st       <= W_IDLE;
            busy     <= 1'b0;
            done     <= 1'b0;
            error    <= 1'b0;
            wr_en    <= 1'b0;
            core_run <= 1'b0;
            par_i    <= 3'd0;
            tbl      <= 2'd0;
            r        <= 11'd0;
            sub      <= 1'b0;
            wd       <= 18'd0;
            row_sh   <= 96'd0;
            core_off <= OFF_SETUP;
            core_len <= LEN_SETUP;
            core_row <= 12'd0;
        end else begin
            wr_en    <= 1'b0;
            core_run <= 1'b0;
            done     <= 1'b0;

            if (fin_we) begin
                // seed_window_driver splits the row MSB first --
                //     {theta0, delta1, delta2} = row
                // -- so fin_word 0 (theta0) belongs in [95:64], not [31:0].
                // Getting this backwards only permutes the three 32-bit words,
                // which makes every row compare unequal while every word is right.
                row_sh[{2'd2 - fin_word, 5'd0} +: 32] <= fin_data;
            end

            if (st != W_IDLE && st != W_ERR) begin
                wd <= wd + 18'd1;
                if (wd >= WD_LIMIT) begin
                    st    <= W_ERR;
                    error <= 1'b1;
                end
            end

            case (st)
                W_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        par_i <= 3'd0;
                        tbl   <= 2'd0;
                        r     <= 11'd0;
                        sub   <= 1'b0;
                        wd    <= 18'd0;
                        error <= par_bad;
                        st    <= par_bad ? W_ERR : W_PAR;
                    end
                end

                W_PAR: begin
                    if (par_i == 3'd7) begin
                        core_off <= OFF_SETUP;
                        core_len <= LEN_SETUP;
                        core_row <= 12'd0;
                        sub      <= 1'b0;
                        st       <= W_SETUP;
                    end else begin
                        par_i <= par_i + 3'd1;
                    end
                end

                W_SETUP: begin
                    if (sub == 1'b0) begin
                        core_run <= 1'b1;
                        sub      <= 1'b1;
                    end else if (core_done) begin
                        row_sh <= 96'd0;
                        core_off <= OFF_H1;
                        core_len <= LEN_H1;
                        core_row <= 12'd0;
                        wd       <= 18'd0;
                        sub      <= 1'b0;
                        st       <= W_ROW;
                    end
                end

                W_ROW: begin
                    core_row <= {1'b0, r};
                    if (sub == 1'b0) begin
                        core_run <= 1'b1;
                        wd       <= 18'd0;
                        sub      <= 1'b1;
                    end else if (core_done) begin
                        sub <= 1'b0;
                        st  <= W_WR;
                    end
                end

                W_WR: begin
                    wr_en   <= 1'b1;
                    wr_sel  <= tbl;
                    wr_idx  <= r[SIDE_LOG2-1:0];
                    wr_data <= row_sh;
                    row_sh  <= 96'd0;
                    wd      <= 18'd0;
                    if (r == N_ROWS - 1) begin
                        r <= 11'd0;
                        if (tbl == 2'd3) begin
                            st <= W_DONE;
                        end else begin
                            tbl      <= tbl + 2'd1;
                            core_off <= (tbl == 2'd0) ? OFF_H2
                                      : (tbl == 2'd1) ? OFF_H3 : OFF_C4;
                            core_len <= (tbl == 2'd0) ? LEN_H2
                                      : (tbl == 2'd1) ? LEN_H3 : LEN_C4;
                            core_row <= 12'd0;
                            st       <= W_ROW;
                        end
                    end else begin
                        r  <= r + 11'd1;
                        st <= W_ROW;
                    end
                end

                W_DONE: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    st   <= W_IDLE;
                end

                W_ERR: begin
                    busy  <= 1'b0;
                    error <= 1'b1;
                    if (start) begin
                        st    <= W_IDLE;
                        error <= 1'b0;
                    end
                end

                default: st <= W_IDLE;
            endcase
        end
    end

    always @(*) begin
        if (dbg_addr < 8'd10)
            dbg_data = {64'd0, pwv[dbg_addr[3:0]]};
        else if (dbg_addr == 8'd10)
            dbg_data = {58'd0, st, tbl, r, wd[17:0], 1'b0, busy, done, error};
        else if (dbg_addr == 8'd11)
            dbg_data = {61'd0, fin_seen, fin_wlast, fin_last};
        else if (dbg_addr == 8'd12)
            dbg_data = row_sh;
        else if (dbg_addr >= 8'd64)
            dbg_data = {32'hDB6_0000, core_dbg};
        else
            dbg_data = 96'hD3AD_0000_0000_0000_0000_D3AD;
    end

endmodule
