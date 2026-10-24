`timescale 1ns/1ps

// One replica domain of a GR17 protected product node (h1/h2/h3/c4):
//   * the domain's OWN phase_recurrence (the task's "one recurrence per
//     domain"), driven by the SHARED controls (seed load, theta/delta,
//     advance) so it tracks the main recurrence beat for beat;
//   * the domain's OWN full-circle coefficient table (grp_coef_tab), whose
//     latency matches cordic_phase_to_unit's CORDIC_ITERATIONS + 2 clocks,
//     so its coefficient is presented on the same beat as the main path's
//     CORDIC coefficient;
//   * the reduced product rp_phase_product17: the main path's ALIGNED data
//     word (the shared delay line's re_pipe[ALIGN]) truncated to its top
//     K bits, times the domain's own KC-bit coefficient, +2^(KC-2),
//     >>> (KC-1), saturated to K -- gr17_model.rep_product.
// Instantiate with dont_touch so the two identical domains are never merged.
module grp_prod_dom #(
    parameter integer K = 10,           // product replica width
    parameter integer KC = 12,          // replica coefficient width
    parameter integer CORDIC_ITERATIONS = 25,
    parameter MEM_FILE = "chain/rtl/mem/grp17_coef_kc12.mem"
) (
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 clear,
    input  wire                 ce,        // always 1 in the compensator
    input  wire                 load,      // shared seed_load
    input  wire [31:0]          theta0,
    input  wire [31:0]          delta1,
    input  wire [31:0]          delta2,
    input  wire                 advance,   // shared s_valid
    input  wire signed [16:0]   x_re,      // aligned main data (re_pipe[ALIGN])
    input  wire signed [16:0]   x_im,
    output wire signed [K+1:0]  r_re,      // K-bit product on a K+2 container
    output wire signed [K+1:0]  r_im,
    output wire                 o_valid    // aligned product-valid (obs.)
);
    wire [31:0] phase;
    wire phase_valid;
    (* dont_touch = "true" *) phase_recurrence u_rec (
        .clk(clk), .rst(rst), .clear(clear), .load(load),
        .theta0(theta0), .delta1(delta1), .delta2(delta2),
        .advance(advance), .phase(phase), .valid(phase_valid));

    wire coeff_valid;
    wire signed [KC-1:0] c_re, c_im;
    grp_coef_tab #(
        .CORDIC_ITERATIONS(CORDIC_ITERATIONS), .KC(KC), .MEM_FILE(MEM_FILE)
    ) u_tab (
        .clk(clk), .rst(rst), .clear(clear), .ce(ce),
        .i_valid(phase_valid), .phase(phase[31:20]),
        .o_valid(coeff_valid), .c_re(c_re), .c_im(c_im));

    (* dont_touch = "true" *) rp_phase_product17 #(.K(K), .KC(KC)) u_prod (
        .clk(clk), .ce(ce), .x_re(x_re), .x_im(x_im),
        .c_re(c_re), .c_im(c_im), .r_re(r_re), .r_im(r_im),
        .r_re_d(), .r_im_d());

    // valid chain: coeff_valid is presented with the coefficient beat; the
    // product lands 4 clocks later (cmul 3 + output reg), same as the main
    // rotator's o_valid.
    reg [3:0] vld_q;
    always @(posedge clk) begin
        if (rst || clear)
            vld_q <= 4'b0000;
        else if (ce)
            vld_q <= {vld_q[2:0], coeff_valid};
    end
    assign o_valid = vld_q[3];

    initial begin
        if (K < 4 || K > 16 || KC < 8 || KC > 24)
            $error("grp_prod_dom: unsupported K=%0d KC=%0d", K, KC);
    end
endmodule
