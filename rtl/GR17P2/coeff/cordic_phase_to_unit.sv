`timescale 1ns/1ps

// One rotation step per clock.  x and y are XW-bit and wrap modulo 2^XW (the model wraps to its internal width
// after every iteration); z stays 32 bits because the direction of each step is the sign of a 32-bit
// accumulation of the 32-bit atan constants.  x_full / y_full are the (XW+1)-bit sums before wrapping.
module cordic_iter_pipe #(
    parameter integer SHIFT = 0,
    parameter integer XW = 29
) (
    input  wire                         clk,
    input  wire                         rst,
    input  wire                         clear,
    input  wire                         ce,
    input  wire signed [31:0]           atan,
    input  wire signed [XW-1:0]         x_in,
    input  wire signed [XW-1:0]         y_in,
    input  wire signed [31:0]           z_in,
    input  wire                         cos_neg_in,
    input  wire                         sin_neg_in,
    input  wire                         valid_in,
    output reg  signed [XW-1:0]         x_out,
    output reg  signed [XW-1:0]         y_out,
    output reg  signed [31:0]           z_out,
    output reg                          cos_neg_out,
    output reg                          sin_neg_out,
    output reg                          valid_out
);
    wire               sub = ~z_in[31];
    wire signed [XW-1:0] xsh = x_in >>> SHIFT;
    wire signed [XW-1:0] ysh = y_in >>> SHIFT;
    wire signed [XW:0]   xe = {x_in[XW-1], x_in};
    wire signed [XW:0]   ye = {y_in[XW-1], y_in};
    wire signed [XW:0]   xse = {xsh[XW-1], xsh};
    wire signed [XW:0]   yse = {ysh[XW-1], ysh};
    wire signed [XW:0]   x_full = sub ? (xe - yse) : (xe + yse);
    wire signed [XW:0]   y_full = sub ? (ye + xse) : (ye - xse);
    wire signed [31:0]   z_next = sub ? (z_in - atan) : (z_in + atan);

    always @(posedge clk) begin
        if (ce) begin
            x_out <= x_full[XW-1:0];
            y_out <= y_full[XW-1:0];
            z_out <= z_next;
            cos_neg_out <= cos_neg_in;
            sin_neg_out <= sin_neg_in;
        end
    end

    always @(posedge clk) begin
        if (rst || clear) valid_out <= 1'b0;
        else if (ce) valid_out <= valid_in;
    end
endmodule

// Cosine and sine of a 32-bit phase (turns, 2^32 = one revolution) as Q1.(OUT_W-1), OUT_W-bit signed,
// saturated.  Only phase[31:32-PHASE_BITS] is used.  Latency from i_valid to o_valid is ITERATIONS + 2
// clocks: one quadrant-fold capture, one clock per iteration, one output register (negation, rounding,
// saturation).  phase_compensator's ALIGN_DELAY must equal that latency.
module cordic_phase_to_unit #(
    parameter integer ITERATIONS = 25,
    parameter integer OUT_W = 25,
    parameter integer PHASE_BITS = 29
) (
    input  wire                         clk,
    input  wire                         rst,
    input  wire                         clear,
    input  wire                         ce,
    input  wire                         i_valid,
    input  wire [31:0]                  i_phase,
    output wire                         o_valid,
    output wire signed [OUT_W-1:0]      o_cos,
    output wire signed [OUT_W-1:0]      o_sin,
    output wire                         o_sat_cos,
    output wire                         o_sat_sin
);
`include "cordic_constants.vh"

    // Internal binary point is XW-3 (fxp/cordic.py: frac = width - 3); the output
    // requantization drops XW-3-(OUT_W-1) = XW-OUT_W-2 fraction bits.
    localparam integer XW = CORDIC_W;
    localparam integer ROUND_SHIFT = XW - OUT_W - 2;
    localparam signed [XW+1:0] ROUND_HALF = {{(XW+1){1'b0}}, 1'b1} << (ROUND_SHIFT - 1);
    localparam signed [XW+1:0] SAT_HI = ({{(XW+1){1'b0}}, 1'b1} << (OUT_W - 1)) - 1;
    localparam signed [XW+1:0] SAT_LO = ~SAT_HI;

    initial begin
        if (ITERATIONS != CORDIC_ITER)
            $error("cordic_phase_to_unit supports exactly ITERATIONS=%0d; constants are fixed", CORDIC_ITER);
        if (OUT_W != CORDIC_OUT_W)
            $error("cordic_phase_to_unit supports exactly OUT_W=%0d; constants are fixed", CORDIC_OUT_W);
        if (PHASE_BITS != CORDIC_PHASE_BITS)
            $error("cordic_phase_to_unit supports exactly PHASE_BITS=%0d; constants are fixed", CORDIC_PHASE_BITS);
        if (ROUND_SHIFT < 1 || ROUND_SHIFT > XW)
            $error("cordic_phase_to_unit: ROUND_SHIFT=%0d outside 1..%0d", ROUND_SHIFT, XW);
    end

    wire signed [XW-1:0] x [0:ITERATIONS];
    wire signed [XW-1:0] y [0:ITERATIONS];
    wire signed [31:0]   z [0:ITERATIONS];
    wire                 cos_neg [0:ITERATIONS];
    wire                 sin_neg [0:ITERATIONS];
    wire                 valid_pipe [0:ITERATIONS];

    // Quadrant fold of the PHASE_BITS-wide trimmed phase; the low 32-PHASE_BITS bits of z are zero here,
    // so the 32-bit fold (model: bound 2^31) is a PHASE_BITS one (bound 2^(PHASE_BITS-1)).
    wire [PHASE_BITS-1:0] ph = i_phase[31 -: PHASE_BITS];
    localparam [PHASE_BITS-1:0] HALF_TURN = {{(PHASE_BITS-1){1'b0}}, 1'b1} << (PHASE_BITS - 1);
    reg  [PHASE_BITS-1:0] z_hi;
    reg         input_cos_neg;
    reg         input_sin_neg;
    always_comb begin
        case (ph[PHASE_BITS-1 -: 2])
            2'b00: begin
                z_hi = ph;
                input_cos_neg = 1'b0;
                input_sin_neg = 1'b0;
            end
            2'b01: begin
                z_hi = HALF_TURN - ph;
                input_cos_neg = 1'b1;
                input_sin_neg = 1'b0;
            end
            2'b10: begin
                z_hi = ph - HALF_TURN;
                input_cos_neg = 1'b1;
                input_sin_neg = 1'b1;
            end
            default: begin
                z_hi = {PHASE_BITS{1'b0}} - ph;
                input_cos_neg = 1'b0;
                input_sin_neg = 1'b1;
            end
        endcase
    end

    reg signed [31:0] z0;
    reg               cos_neg0;
    reg               sin_neg0;
    reg               valid0;
    always @(posedge clk) begin
        if (ce) begin
            z0 <= {z_hi, {(32-PHASE_BITS){1'b0}}};
            cos_neg0 <= input_cos_neg;
            sin_neg0 <= input_sin_neg;
        end
    end
    always @(posedge clk) begin
        if (rst || clear) valid0 <= 1'b0;
        else if (ce) valid0 <= i_valid;
    end

    assign x[0] = CORDIC_X0;
    assign y[0] = {XW{1'b0}};
    assign z[0] = z0;
    assign cos_neg[0] = cos_neg0;
    assign sin_neg[0] = sin_neg0;
    assign valid_pipe[0] = valid0;

    genvar g;
    generate
        for (g = 0; g < ITERATIONS; g = g + 1) begin : gen_iter
            wire signed [31:0] atan_i = cordic_atan(g);
            cordic_iter_pipe #(.SHIFT(g), .XW(XW)) u_iter (
                .clk(clk),
                .rst(rst),
                .clear(clear),
                .ce(ce),
                .atan(atan_i),
                .x_in(x[g]),
                .y_in(y[g]),
                .z_in(z[g]),
                .cos_neg_in(cos_neg[g]),
                .sin_neg_in(sin_neg[g]),
                .valid_in(valid_pipe[g]),
                .x_out(x[g+1]),
                .y_out(y[g+1]),
                .z_out(z[g+1]),
                .cos_neg_out(cos_neg[g+1]),
                .sin_neg_out(sin_neg[g+1]),
                .valid_out(valid_pipe[g+1])
            );
        end
    endgenerate

    // Negation does not wrap (two extra bits); (v + 2^(ROUND_SHIFT-1)) >>> ROUND_SHIFT moves from XW-3 to
    // OUT_W-1 fraction bits, ties toward +inf.
    wire signed [XW+1:0] x_ext = {{2{x[ITERATIONS][XW-1]}}, x[ITERATIONS]};
    wire signed [XW+1:0] y_ext = {{2{y[ITERATIONS][XW-1]}}, y[ITERATIONS]};
    wire signed [XW+1:0] cos_s = cos_neg[ITERATIONS] ? -x_ext : x_ext;
    wire signed [XW+1:0] sin_s = sin_neg[ITERATIONS] ? -y_ext : y_ext;
    wire signed [XW+1:0] cos_r = (cos_s + ROUND_HALF) >>> ROUND_SHIFT;
    wire signed [XW+1:0] sin_r = (sin_s + ROUND_HALF) >>> ROUND_SHIFT;
    wire cos_hi = (cos_r > SAT_HI);
    wire cos_lo = (cos_r < SAT_LO);
    wire sin_hi = (sin_r > SAT_HI);
    wire sin_lo = (sin_r < SAT_LO);

    reg signed [OUT_W-1:0] cos_q, sin_q;
    reg               sat_cos_q, sat_sin_q, valid_q;
    always @(posedge clk) begin
        if (ce) begin
            cos_q <= cos_hi ? SAT_HI[OUT_W-1:0] : cos_lo ? SAT_LO[OUT_W-1:0] : cos_r[OUT_W-1:0];
            sin_q <= sin_hi ? SAT_HI[OUT_W-1:0] : sin_lo ? SAT_LO[OUT_W-1:0] : sin_r[OUT_W-1:0];
            sat_cos_q <= valid_pipe[ITERATIONS] & (cos_hi | cos_lo);
            sat_sin_q <= valid_pipe[ITERATIONS] & (sin_hi | sin_lo);
        end
    end
    always @(posedge clk) begin
        if (rst || clear) valid_q <= 1'b0;
        else if (ce) valid_q <= valid_pipe[ITERATIONS];
    end

    assign o_valid = valid_q;
    assign o_cos = cos_q;
    assign o_sin = sin_q;
    assign o_sat_cos = sat_cos_q;
    assign o_sat_sin = sat_sin_q;
endmodule
