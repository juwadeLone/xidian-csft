`timescale 1ns/1ps

// One coefficient per clock after pipeline fill.  Phase uses unsigned turns:
// 2^32 represents one full revolution.
module coeff_seed_engine #(
    parameter integer CORDIC_ITERATIONS = 25,
    // coefficient component width = the CORDIC's output width
    parameter integer COEF_W = 25,
    // rpr_cmp: 1 = the phase recurrence triplicated, phase and valid voted bit
    // by bit (combinational, so the CORDIC alignment is unchanged).  A copy
    // that goes wrong resynchronises at the next row's seed load.
    parameter integer REC_TMR = 0
) (
    input  wire                         clk,
    input  wire                         rst,
    input  wire                         clear,
    input  wire                         load,
    input  wire [31:0]                  theta0,
    input  wire [31:0]                  delta1,
    input  wire [31:0]                  delta2,
    input  wire                         advance,
    output wire                         coeff_valid,
    output wire signed [COEF_W-1:0]     coeff_re,
    output wire signed [COEF_W-1:0]     coeff_im,
    output wire [31:0]                  o_phase,        // rpr_cmp: the CORDIC's input phase
    input  wire                         rpr_clr,
    output wire                         rec_sticky      // the three copies disagreed
);
    wire [31:0] phase;
    wire phase_valid;

    generate
        if (REC_TMR != 0) begin : g_rec_tmr
            wire [31:0] p0, p1, p2;
            wire v0, v1, v2;
            (* dont_touch = "true" *) phase_recurrence u_phase0(
                .clk(clk),.rst(rst),.clear(clear),.load(load),
                .theta0(theta0),.delta1(delta1),.delta2(delta2),
                .advance(advance),.phase(p0),.valid(v0));
            (* dont_touch = "true" *) phase_recurrence u_phase1(
                .clk(clk),.rst(rst),.clear(clear),.load(load),
                .theta0(theta0),.delta1(delta1),.delta2(delta2),
                .advance(advance),.phase(p1),.valid(v1));
            (* dont_touch = "true" *) phase_recurrence u_phase2(
                .clk(clk),.rst(rst),.clear(clear),.load(load),
                .theta0(theta0),.delta1(delta1),.delta2(delta2),
                .advance(advance),.phase(p2),.valid(v2));
            assign phase = (p0 & p1) | (p0 & p2) | (p1 & p2);
            assign phase_valid = (v0 & v1) | (v0 & v2) | (v1 & v2);
            reg st;
            always @(posedge clk) begin
                if (rst || rpr_clr) st <= 1'b0;
                else if ((p0 != p1) || (p0 != p2) || (v0 != v1) || (v0 != v2)) st <= 1'b1;
            end
            assign rec_sticky = st;
        end else begin : g_rec_one
            phase_recurrence u_phase(
                .clk(clk),.rst(rst),.clear(clear),.load(load),
                .theta0(theta0),.delta1(delta1),.delta2(delta2),
                .advance(advance),.phase(phase),.valid(phase_valid));
            assign rec_sticky = 1'b0;
        end
    endgenerate
    assign o_phase = phase;

    cordic_phase_to_unit #(.ITERATIONS(CORDIC_ITERATIONS), .OUT_W(COEF_W)) u_cordic(
        .clk(clk),.rst(rst),.clear(clear),.ce(1'b1),
        .i_valid(phase_valid),.i_phase(phase),.o_valid(coeff_valid),
        .o_cos(coeff_re),.o_sin(coeff_im),.o_sat_cos(),.o_sat_sin());
endmodule
