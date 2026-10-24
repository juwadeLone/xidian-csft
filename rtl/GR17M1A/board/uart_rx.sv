// 8N1 UART receiver, one valid pulse per received byte.
//
// Pair of rtl/board/uart_tx.sv: same DIV (cycles per bit), same idle-high
// line.  Samples in the middle of each bit.  A framing error (start not 0 or
// stop not 1) drops the byte; no valid pulse.
`timescale 1ns/1ps
module uart_rx #(
    parameter integer DIV = 1085
) (
    input  wire       clk,
    input  wire       rst,        // synchronous, active high
    input  wire       rx,
    output reg  [7:0] data,
    output reg        valid
);
    localparam integer DIV_W = (DIV <= 2) ? 2 : $clog2(DIV);

    reg [1:0]         rx_sync;
    reg               rx_q;
    wire              start_fall = rx_q & ~rx_sync[1];

    reg               active;
    reg [DIV_W-1:0]   div_cnt;
    reg [3:0]         bit_idx;    // 0 = start sample, 1..8 = data, 9 = stop
    reg [7:0]         shifter;

    always @(posedge clk) begin
        if (rst) begin
            rx_sync <= 2'b11;
            rx_q    <= 1'b1;
        end else begin
            rx_sync <= {rx_sync[0], rx};
            rx_q    <= rx_sync[1];
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            active  <= 1'b0;
            div_cnt <= {DIV_W{1'b0}};
            bit_idx <= 4'd0;
            shifter <= 8'd0;
            data    <= 8'd0;
            valid   <= 1'b0;
        end else begin
            valid <= 1'b0;
            if (!active) begin
                if (start_fall) begin
                    active  <= 1'b1;
                    bit_idx <= 4'd0;
                    div_cnt <= {DIV_W{1'b0}};
                end
            end else if (bit_idx == 4'd0) begin
                // Center of start bit: DIV/2 cycles after the falling edge.
                if (div_cnt == DIV_W'(DIV / 2)) begin
                    div_cnt <= {DIV_W{1'b0}};
                    if (rx_sync[1] != 1'b0) begin
                        active <= 1'b0;          // false start
                    end else begin
                        bit_idx <= 4'd1;
                    end
                end else begin
                    div_cnt <= div_cnt + 1'b1;
                end
            end else if (div_cnt == DIV_W'(DIV - 1)) begin
                div_cnt <= {DIV_W{1'b0}};
                if (bit_idx <= 4'd8) begin
                    shifter <= {rx_sync[1], shifter[7:1]};
                    bit_idx <= bit_idx + 4'd1;
                end else begin
                    active <= 1'b0;
                    if (rx_sync[1] == 1'b1) begin
                        data  <= shifter;
                        valid <= 1'b1;
                    end
                end
            end else begin
                div_cnt <= div_cnt + 1'b1;
            end
        end
    end
endmodule
