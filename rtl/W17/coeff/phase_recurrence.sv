`timescale 1ns/1ps

module phase_recurrence (
    input  wire             clk,
    input  wire             rst,
    input  wire             clear,
    input  wire             load,
    input  wire [31:0]      theta0,
    input  wire [31:0]      delta1,
    input  wire [31:0]      delta2,
    input  wire             advance,
    output reg  [31:0]      phase,
    output reg              valid
);
    reg [31:0] theta_q;
    reg [31:0] delta_q;
    reg [31:0] delta2_q;
    always @(posedge clk) begin
        if (rst || clear) begin
            theta_q <= 0;
            delta_q <= 0;
            delta2_q <= 0;
            phase <= 0;
            valid <= 1'b0;
        end else if (load) begin
            theta_q <= theta0;
            delta_q <= delta1;
            delta2_q <= delta2;
            phase <= 0;
            valid <= 1'b0;
        end else if (advance) begin
            phase <= theta_q;
            valid <= 1'b1;
            theta_q <= theta_q + delta_q;
            delta_q <= delta_q + delta2_q;
        end else begin
            valid <= 1'b0;
        end
    end
endmodule

