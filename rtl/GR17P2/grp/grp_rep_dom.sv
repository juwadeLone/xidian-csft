`timescale 1ns/1ps

// One replica domain of one grouped GR17 stage (stages 0..8 of a transform).
// Mirrors sdf_stage's structure beat-for-beat at reduced width K, per
// gr17_model.rep_stage (operand mode TRUNC):
//
//   * input register: the 18-bit sign extension of the replica stream word
//     (stage 0: the main input word; stage g > 0: the previous replica output
//     re-expanded by << (17 - K_prev));
//   * own delay line: TW = 18 - SP bits per word, same DEPTH and the same
//     read/write timing as the main stage (register / SRL / RAM arms);
//   * write word: first_half ? x_t : fb_t - x_t, low TW bits (modular);
//   * rotated operand pre = first_half ? fb_t : fb_t + x_t (KA = TW+1 bits);
//   * own twiddle ROM (grp_twiddle_rom): this stage's KT-bit table
//     (gr17_model.rep_twiddles) -- the replica no longer shares the main
//     path's table, so a twiddle-table fault is not common to both;
//   * LUT product (rp_cmul_lut, exact modes only) + bypass/quarter select +
//     (+2^(KT-2)) >>> (KT-1), saturated to K bits;
//   * output word on the 17-bit grid: r << (17 - K), registered on the same
//     ce edge as the main stage output (input reg -> output reg = 5 edges).
//
// All control (phase, addresses, halves, twiddle-cycle flags, ce) is the main
// stage's SHARED copy; only data-path state belongs to the domain.
// Instantiate with (* dont_touch = "true" *) per domain so synthesis never
// merges the two identical replicas.
module grp_rep_dom #(
    parameter integer LOGN = 11,
    parameter integer STAGE = 0,
    parameter integer MAX_LOGN = 11,
    parameter integer DATA_W = 17,
    parameter integer INVERSE = 0,
    parameter integer K = 16,
    parameter integer SC = 0,
    parameter integer KT = 13,
    parameter MEM_BASE = "chain/rtl/mem/grp17_twtab_p1_s"
) (
    input  wire                      clk,
    input  wire                      rst,       // rst || clear of the stage
    input  wire                      ce,        // the main stage's local_ce
    input  wire                      first_half,// first_half_d
    input  wire [10:0]               phase_addr,// shared delay-line read address
    input  wire [10:0]               addr_d,    // shared delay-line write address
    input  wire                      ctrl_valid,// ctrl_valid_d
    input  wire                      do_twiddle,// do_twiddle_tw_d
    input  wire                      unity,     // unity_tw_d
    input  wire                      quarter,   // quarter_tw_d
    input  wire                      i_valid,   // level DEPTH+1 token, the beat
                                               // aligned with the comb operand
    input  wire signed [DATA_W-1:0]  i_re,
    input  wire signed [DATA_W-1:0]  i_im,
    output wire                      o_valid,
    output wire signed [DATA_W-1:0]  o_re,
    output wire signed [DATA_W-1:0]  o_im
);
    `include "grp17_defs.vh"
    localparam integer DEPTH = 1 << (LOGN-STAGE-1);
    localparam integer ADDR_W = (DEPTH <= 1) ? 1 : $clog2(DEPTH);
    localparam integer TWIDDLE_STAGE_ID = STAGE + (MAX_LOGN-LOGN);
    localparam integer SP = grp_sp17(K, SC);    // operand bits dropped
    localparam integer TW = grp_tw17(K, SC);    // operand bits kept
    localparam integer KA = TW + 1;             // butterfly-sum width
    localparam integer MODE = grp_mode17(KA, KT);
    localparam integer OW = KA + KT + 1;        // replica product width
    localparam integer SR = KT - 1;             // replica shift (gr17_model)
    localparam integer RW = K + 2;
    localparam signed [RW-1:0] RMAX =  (1 <<< (K-1)) - 1;
    localparam signed [RW-1:0] RMIN = -(1 <<< (K-1));
    localparam [OW-1:0] HALF = {{(OW-1){1'b0}}, 1'b1} << (KT-2);

    // ---- input register (same edge as the main stage's x_d) ---------------
    reg signed [17:0] x_re_q, x_im_q;
    always @(posedge clk) begin
        if (rst) begin
            x_re_q <= '0;
            x_im_q <= '0;
        end else if (ce) begin
            x_re_q <= {i_re[DATA_W-1], i_re};
            x_im_q <= {i_im[DATA_W-1], i_im};
        end
    end
    wire signed [TW-1:0] x_re_t = x_re_q[17 -: TW];
    wire signed [TW-1:0] x_im_t = x_im_q[17 -: TW];

    wire signed [TW-1:0] fb_re, fb_im;

    // ---- replica write word (TW bits; the difference wraps) ----------------
    wire signed [TW:0]   wd_re = fb_re - x_re_t;
    wire signed [TW:0]   wd_im = fb_im - x_im_t;
    wire signed [TW-1:0] w_re = first_half ? x_re_t : wd_re[TW-1:0];
    wire signed [TW-1:0] w_im = first_half ? x_im_t : wd_im[TW-1:0];

    // ---- own delay line, same arm structure as the main stage's ------------
    generate
        if (DEPTH == 1) begin : g_feedback_register
            reg signed [TW-1:0] fb_re_q, fb_im_q;
            assign fb_re = fb_re_q;
            assign fb_im = fb_im_q;
            always @(posedge clk) begin
                if (ce && ctrl_valid) begin
                    fb_re_q <= w_re;
                    fb_im_q <= w_im;
                end
            end
        end else if (DEPTH <= 32) begin : g_feedback_srl
            reg signed [TW-1:0] sr_re [0:DEPTH-1];
            reg signed [TW-1:0] sr_im [0:DEPTH-1];
            integer k;
            always @(posedge clk) begin
                if (ce) begin
                    sr_re[0] <= w_re;
                    sr_im[0] <= w_im;
                    for (k = 1; k < DEPTH; k = k + 1) begin
                        sr_re[k] <= sr_re[k-1];
                        sr_im[k] <= sr_im[k-1];
                    end
                end
            end
            assign fb_re = sr_re[DEPTH-1];
            assign fb_im = sr_im[DEPTH-1];
        end else begin : g_feedback_ram
            reg [2*TW-1:0] mem [0:DEPTH-1];
            reg [2*TW-1:0] rd_q;
            always @(posedge clk) begin
                if (ce) begin
                    if (ctrl_valid)
                        mem[addr_d[ADDR_W-1:0]] <= {w_im, w_re};
                    rd_q <= mem[phase_addr[ADDR_W-1:0]];
                end
            end
            assign fb_re = rd_q[TW-1:0];
            assign fb_im = rd_q[2*TW-1:TW];
        end
    endgenerate

    // ---- own twiddle ROM: this stage's KT-bit table -------------------------
    wire signed [KT-1:0] tw_wr, tw_wi;
    grp_twiddle_rom #(
        .STAGE_ID(TWIDDLE_STAGE_ID),
        .DEPTH(DEPTH),
        .ADDR_W(ADDR_W),
        .INVERSE(INVERSE),
        .KT(KT),
        .MEM_BASE(MEM_BASE)
    ) u_twiddle_rom (
        .clk(clk),
        .ce(ce),
        .addr(addr_d[ADDR_W-1:0]),
        .wr(tw_wr),
        .wi(tw_wi)
    );

    // ---- rotate (mirrors the FP stage's a/b/c/d register levels) -----------
    // edge a (same edge as the main stage's pre_tw_d): butterfly sum operand
    reg signed [KA-1:0] pr_a, pi_a;
    reg vld_a;
    always @(posedge clk) begin
        if (rst) begin
            pr_a <= '0;
            pi_a <= '0;
            vld_a <= 1'b0;
        end else if (ce) begin
            pr_a <= first_half ? fb_re : fb_re + x_re_t;
            pi_a <= first_half ? fb_im : fb_im + x_im_t;
            vld_a <= i_valid;
        end
    end

    // edges b, c, d: the reduced product (LUT only, exact mode)
    wire signed [OW-1:0] p_re, p_im;
    (* dont_touch = "true" *) rp_cmul_lut #(
        .KA(KA), .KT(KT), .MODE(MODE), .OW(OW)
    ) u_mul (
        .clk(clk), .ce(ce),
        .pr(pr_a), .pi(pi_a), .wr(tw_wr), .wi(tw_wi),
        .re(p_re), .im(p_im));

    // bypass / quarter operands and the mode, carried b -> c -> d
    reg signed [KA-1:0] pr_b, pi_b, pr_c, pi_c, pr_d, pi_d;
    reg [2:0] md_b, md_c, md_d;   // {exact_quarter, do_twiddle, quarter_negative}
    reg vld_b, vld_c, vld_d;
    always @(posedge clk) begin
        if (rst) begin
            pr_b <= '0; pi_b <= '0; md_b <= 3'b000; vld_b <= 1'b0;
            pr_c <= '0; pi_c <= '0; md_c <= 3'b000; vld_c <= 1'b0;
            pr_d <= '0; pi_d <= '0; md_d <= 3'b000; vld_d <= 1'b0;
        end else if (ce) begin
            pr_b <= pr_a; pi_b <= pi_a;
            md_b <= {do_twiddle && quarter, do_twiddle && !unity, tw_wi[KT-1]};
            vld_b <= vld_a;
            pr_c <= pr_b; pi_c <= pi_b; md_c <= md_b; vld_c <= vld_b;
            pr_d <= pr_c; pi_d <= pi_c; md_d <= md_c; vld_d <= vld_c;
        end
    end

    // acc select + half + >>> (KT-1) + sat_K, combinational during cycle d,
    // registered into the domain output word on the same edge the FP stage
    // registers its o_re (edge d+1 ... the fifth edge after the input reg).
    wire signed [OW-1:0] bx_re = $signed({{(OW-KA){pr_d[KA-1]}}, pr_d}) <<< (KT-1);
    wire signed [OW-1:0] bx_im = $signed({{(OW-KA){pi_d[KA-1]}}, pi_d}) <<< (KT-1);
    wire signed [OW-1:0] acc_re = md_d[2] ? (md_d[0] ? bx_im : -bx_im)
                                        : (md_d[1] ? p_re : bx_re);
    wire signed [OW-1:0] acc_im = md_d[2] ? (md_d[0] ? -bx_re : bx_re)
                                        : (md_d[1] ? p_im : bx_im);
    wire signed [OW:0]   sh_re  = ($signed({acc_re[OW-1], acc_re}) + $signed({1'b0, HALF})) >>> SR;
    wire signed [OW:0]   sh_im  = ($signed({acc_im[OW-1], acc_im}) + $signed({1'b0, HALF})) >>> SR;
    localparam signed [OW:0] RMAX_W = RMAX;
    localparam signed [OW:0] RMIN_W = RMIN;
    wire signed [RW-1:0] nx_re = (sh_re > RMAX_W) ? RMAX :
                                 (sh_re < RMIN_W) ? RMIN : sh_re[RW-1:0];
    wire signed [RW-1:0] nx_im = (sh_im > RMAX_W) ? RMAX :
                                 (sh_im < RMIN_W) ? RMIN : sh_im[RW-1:0];

    reg signed [RW-1:0] r_re_q, r_im_q;
    reg vld_q;
    always @(posedge clk) begin
        if (rst) begin
            r_re_q <= '0;
            r_im_q <= '0;
            vld_q <= 1'b0;
        end else if (ce) begin
            r_re_q <= nx_re;
            r_im_q <= nx_im;
            vld_q <= vld_d;
        end
    end

    // output on the 17-bit grid (r is saturated to the K-bit range)
    generate
        if (K < DATA_W) begin : g_grid
            assign o_re = {r_re_q[K-1:0], {(DATA_W-K){1'b0}}};
            assign o_im = {r_im_q[K-1:0], {(DATA_W-K){1'b0}}};
        end else begin : g_full
            assign o_re = r_re_q[DATA_W-1:0];
            assign o_im = r_im_q[DATA_W-1:0];
        end
    endgenerate
    assign o_valid = vld_q;

    initial begin
        if (K < 4 || K > 16 || KT < 8 || KT > 24)
            $error("grp_rep_dom: unsupported K=%0d KT=%0d", K, KT);
    end
endmodule
