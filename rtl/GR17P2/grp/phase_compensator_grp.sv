`timescale 1ns/1ps

// GR17 protected reference product node (h1/h2/h3/c4): the W17
// phase_compensator datapath VERBATIM (data delay line, coeff_seed_engine
// recurrence + CORDIC, sdf_complex_rotator product) plus
//   * two grp_prod_dom replica domains -- each own phase_recurrence
//     (same load/advance/seeds), own full-circle coefficient table,
//     reduced product at K bits (gr17_model.rep_product);
//   * one rpr_vote per component at the product output: replicas agree
//     (r1 == r2) AND |main_topK - r1| > T -> substitute
//     {r1_topK, 1'b1, zeros} (= replica + half-LSB; flat bias, comp = 0).
//     REG = 0 keeps the W17 output timing: the mux is combinational on
//     the registered product and replica words, m_valid/m_sof/m_eof are
//     untouched.
module phase_compensator_grp #(
    parameter integer CORDIC_ITERATIONS = 25,
    parameter integer COEF_W = 25,
    parameter integer IN_W  = 11,
    parameter integer OUT_W = 11,
    parameter integer REC_TMR = 0,
    parameter integer GRP_K = 10,
    parameter integer GRP_T = 4,
    parameter integer GRP_KC = 12,
    parameter GRP_MEM = "chain/rtl/mem/grp17_coef_kc12.mem"
) (
    input  wire                         clk,
    input  wire                         rst,
    input  wire                         clear,
    input  wire                         seed_load,
    input  wire [31:0]                  theta0,
    input  wire [31:0]                  delta1,
    input  wire [31:0]                  delta2,
    input  wire                         s_valid,
    input  wire                         s_sof,
    input  wire                         s_eof,
    input  wire signed [IN_W-1:0]       s_re,
    input  wire signed [IN_W-1:0]       s_im,
    output wire                         m_valid,
    output wire                         m_sof,
    output wire                         m_eof,
    output wire signed [OUT_W-1:0]      m_re,
    output wire signed [OUT_W-1:0]      m_im,
    output wire                         sample_overflow,
    output wire                         overflow_sticky,
    output reg                          alignment_error,
    input  wire                         rpr_clr,
    output wire [2:0]                   rpr_sticky
);
    // phase_recurrence is 1 cycle; cordic_phase_to_unit takes CORDIC_ITERATIONS + 2
    // clocks from i_valid to o_valid (capture, one per iteration, output register).
    localparam integer ALIGN_DELAY = CORDIC_ITERATIONS + 2;
    reg signed [IN_W-1:0] re_pipe [0:ALIGN_DELAY];
    reg signed [IN_W-1:0] im_pipe [0:ALIGN_DELAY];
    reg [ALIGN_DELAY:0] valid_pipe,sof_pipe,eof_pipe;
    integer i;
    wire coeff_valid;
    wire signed [COEF_W-1:0] coeff_re,coeff_im;

    wire [31:0] cordic_phase;
    wire rec_sticky;
    coeff_seed_engine #(.CORDIC_ITERATIONS(CORDIC_ITERATIONS), .COEF_W(COEF_W), .REC_TMR(REC_TMR)) u_coeff(
        .clk(clk),.rst(rst),.clear(clear),.load(seed_load),
        .theta0(theta0),.delta1(delta1),.delta2(delta2),
        .advance(s_valid),.coeff_valid(coeff_valid),
        .coeff_re(coeff_re),.coeff_im(coeff_im),
        .o_phase(cordic_phase),.rpr_clr(rpr_clr),.rec_sticky(rec_sticky));

    always @(posedge clk) begin
        if (rst || clear) begin
            for (i=0; i<=ALIGN_DELAY; i=i+1) begin
                re_pipe[i] <= '0;
                im_pipe[i] <= '0;
            end
            valid_pipe <= '0;
            sof_pipe <= '0;
            eof_pipe <= '0;
            alignment_error <= 1'b0;
        end else begin
            re_pipe[0] <= s_re;
            im_pipe[0] <= s_im;
            valid_pipe[0] <= s_valid;
            sof_pipe[0] <= s_valid && s_sof;
            eof_pipe[0] <= s_valid && s_eof;
            for (i=0; i<ALIGN_DELAY; i=i+1) begin
                re_pipe[i+1] <= re_pipe[i];
                im_pipe[i+1] <= im_pipe[i];
                valid_pipe[i+1] <= valid_pipe[i];
                sof_pipe[i+1] <= sof_pipe[i];
                eof_pipe[i+1] <= eof_pipe[i];
            end
            if (coeff_valid != valid_pipe[ALIGN_DELAY])
                alignment_error <= 1'b1;
        end
    end

    // ---- W16R2 product (verbatim); its raw output feeds the two votes -----
    wire signed [OUT_W-1:0] prod_re, prod_im;
    wire prod_valid, prod_sof, prod_eof;
    assign m_valid = prod_valid;
    assign m_sof = prod_sof;
    assign m_eof = prod_eof;
    sdf_complex_rotator #(
        .IN_W(IN_W),.OUT_W(OUT_W),.TW_W(COEF_W),.TW_FRAC(COEF_W-1),.ROUND(1)
    ) u_product(
        .clk(clk),.rst(rst),.clear(clear),.ce(1'b1),
        .i_valid(coeff_valid && valid_pipe[ALIGN_DELAY]),
        .i_sof(sof_pipe[ALIGN_DELAY]),.i_eof(eof_pipe[ALIGN_DELAY]),
        .i_do_twiddle(1'b1),.i_exact_quarter(1'b0),.i_scale(1'b0),
        .i_re(re_pipe[ALIGN_DELAY]),.i_im(im_pipe[ALIGN_DELAY]),
        .i_wr(coeff_re),.i_wi(coeff_im),
        .o_valid(prod_valid),.o_sof(prod_sof),.o_eof(prod_eof),
        .o_re(prod_re),.o_im(prod_im),
        .o_sample_overflow(sample_overflow),
        .o_overflow_re(),.o_overflow_im(),
        .o_overflow_sticky(overflow_sticky));

    // ---- GR16S: the two product replica domains ----------------------------
    wire signed [16:0] x_re17 = {{(17-IN_W){re_pipe[ALIGN_DELAY][IN_W-1]}},
                                 re_pipe[ALIGN_DELAY]};
    wire signed [16:0] x_im17 = {{(17-IN_W){im_pipe[ALIGN_DELAY][IN_W-1]}},
                                 im_pipe[ALIGN_DELAY]};
    wire signed [GRP_K+1:0] r1_re, r1_im, r2_re, r2_im;
    (* dont_touch = "true" *) grp_prod_dom #(
        .K(GRP_K), .KC(GRP_KC), .CORDIC_ITERATIONS(CORDIC_ITERATIONS),
        .MEM_FILE(GRP_MEM)
    ) u_pdom1 (
        .clk(clk), .rst(rst), .clear(clear), .ce(1'b1),
        .load(seed_load), .theta0(theta0), .delta1(delta1), .delta2(delta2),
        .advance(s_valid),
        .x_re(x_re17), .x_im(x_im17),
        .r_re(r1_re), .r_im(r1_im), .o_valid());

    (* dont_touch = "true" *) grp_prod_dom #(
        .K(GRP_K), .KC(GRP_KC), .CORDIC_ITERATIONS(CORDIC_ITERATIONS),
        .MEM_FILE(GRP_MEM)
    ) u_pdom2 (
        .clk(clk), .rst(rst), .clear(clear), .ce(1'b1),
        .load(seed_load), .theta0(theta0), .delta1(delta1), .delta2(delta2),
        .advance(s_valid),
        .x_re(x_re17), .x_im(x_im17),
        .r_re(r2_re), .r_im(r2_im), .o_valid());

    // ---- one vote per component (REG=0: the W16R2 timing is kept) ----------
    wire st_re, st_im;
    (* dont_touch = "true" *) rpr_vote #(
        .W(OUT_W), .K(GRP_K), .T(GRP_T), .REG(0), .MOD(0)
    ) u_vote_re (
        .clk(clk), .rst(rst), .ce(1'b1), .clr(rpr_clr), .vld(prod_valid),
        .y(prod_re), .r1(r1_re), .r2(r2_re), .o(m_re), .sticky(st_re));
    (* dont_touch = "true" *) rpr_vote #(
        .W(OUT_W), .K(GRP_K), .T(GRP_T), .REG(0), .MOD(0)
    ) u_vote_im (
        .clk(clk), .rst(rst), .ce(1'b1), .clr(rpr_clr), .vld(prod_valid),
        .y(prod_im), .r1(r1_im), .r2(r2_im), .o(m_im), .sticky(st_im));

    assign rpr_sticky = {st_re | st_im, 1'b0, rec_sticky};

    initial begin
        if (IN_W != 17 || OUT_W != 17)
            $error("phase_compensator_grp: GR17 product nodes are 17-bit");
    end
endmodule
