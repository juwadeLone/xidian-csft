`timescale 1ns/1ps

// Aligns each complex sample with a second-order phase-recursion coefficient (Q1.(COEF_W-1), COEF_W bits
// from the CORDIC) and multiplies through the shared sdf_complex_rotator: half_up rounding of the
// Q(COEF_W-1) product, no /2, always a full multiply, saturation to OUT_W.  Input may contain gaps for
// unit testing, but an FFT vector is expected to be gap-free.
module phase_compensator #(
    parameter integer CORDIC_ITERATIONS = 25,
    parameter integer IN_W  = 11,
    parameter integer OUT_W = 11,
    // coefficient component width (CORDIC output width; product is IN_W x COEF_W)
    parameter integer COEF_W = 25,
    // 1 = phase recurrence triplicated (combinational vote)
    parameter integer REC_TMR = 0
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
    output wire [2:0]                   rpr_sticky       // {product, coefficient, recurrence}; the first two are always 0
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
    coeff_seed_engine #(.CORDIC_ITERATIONS(CORDIC_ITERATIONS), .COEF_W(COEF_W),
                        .REC_TMR(REC_TMR)) u_coeff(
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

    sdf_complex_rotator #(
        .IN_W(IN_W),.OUT_W(OUT_W),.TW_W(COEF_W),.TW_FRAC(COEF_W-1),.ROUND(1)
    ) u_product(
        .clk(clk),.rst(rst),.clear(clear),.ce(1'b1),
        .i_valid(coeff_valid && valid_pipe[ALIGN_DELAY]),
        .i_sof(sof_pipe[ALIGN_DELAY]),.i_eof(eof_pipe[ALIGN_DELAY]),
        .i_do_twiddle(1'b1),.i_exact_quarter(1'b0),.i_scale(1'b0),
        .i_re(re_pipe[ALIGN_DELAY]),.i_im(im_pipe[ALIGN_DELAY]),
        .i_wr(coeff_re),.i_wi(coeff_im),
        .o_valid(m_valid),.o_sof(m_sof),.o_eof(m_eof),
        .o_re(m_re),.o_im(m_im),
        .o_sample_overflow(sample_overflow),
        .o_overflow_re(),.o_overflow_im(),
        .o_overflow_sticky(overflow_sticky));

    assign rpr_sticky = {2'b00, rec_sticky};
endmodule
