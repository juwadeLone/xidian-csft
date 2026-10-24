// 8N1 UART transmitter, one byte per send pulse.
//
// DIV is the clock cycles per bit: CLK_HZ / BAUD.  At 125 MHz and 115200 baud
// that is 1085 (0.007% error, far inside the +-2% a UART receiver tolerates).
// Testbenches set DIV small so a byte costs tens of cycles instead of ~10k.
`timescale 1ns/1ps
module uart_tx #(
    parameter integer DIV = 1085
) (
    input  wire       clk,
    input  wire       rst,        // synchronous, active high
    input  wire       send,       // single-cycle pulse; ignored while busy
    input  wire [7:0] data,
    output reg        tx,
    output wire       busy,
    output reg        done        // single-cycle pulse when the stop bit ends
);
    localparam integer DIV_W = (DIV <= 2) ? 2 : $clog2(DIV);

    reg [DIV_W-1:0] div_cnt;
    reg [3:0]       bit_idx;      // 0 = start, 1..8 = data, 9 = stop
    reg [7:0]       shifter;
    reg             active;

    assign busy = active;

    always @(posedge clk) begin
        if (rst) begin
            tx      <= 1'b1;      // idle high
            active  <= 1'b0;
            done    <= 1'b0;
            div_cnt <= {DIV_W{1'b0}};
            bit_idx <= 4'd0;
            shifter <= 8'd0;
        end else begin
            done <= 1'b0;
            if (!active) begin
                tx <= 1'b1;
                if (send) begin
                    shifter <= data;
                    active  <= 1'b1;
                    bit_idx <= 4'd0;
                    div_cnt <= {DIV_W{1'b0}};
                    tx      <= 1'b0;          // start bit begins immediately
                end
            end else if (div_cnt == DIV_W'(DIV - 1)) begin
                div_cnt <= {DIV_W{1'b0}};
                if (bit_idx == 4'd9) begin
                    active <= 1'b0;
                    done   <= 1'b1;
                    tx     <= 1'b1;
                end else begin
                    bit_idx <= bit_idx + 4'd1;
                    tx      <= (bit_idx == 4'd8) ? 1'b1        // stop bit
                                                 : shifter[0];
                    shifter <= {1'b0, shifter[7:1]};
                end
            end else begin
                div_cnt <= div_cnt + 1'b1;
            end
        end
    end
endmodule
