`timescale 1ns / 1ps

module uart_tx_glue (
    input          clk,
    input          rst_n,
    input  [7:0]   s_tdata,
    input          s_tvalid,
    output         s_tready,
    output reg     send_en,
    output reg [7:0] tx_data,
    input          tx_done
);

reg busy;
reg tx_done_d;

assign s_tready = ~busy;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        busy     <= 1'b0;
        send_en  <= 1'b0;
        tx_data  <= 8'd0;
        tx_done_d<= 1'b0;
    end else begin
        tx_done_d <= tx_done;
        send_en   <= 1'b0;
        if (!busy && s_tvalid) begin
            tx_data <= s_tdata;
            send_en <= 1'b1;
            busy    <= 1'b1;
        end else if (busy && tx_done && !tx_done_d) begin
            busy <= 1'b0;
        end
    end
end

endmodule
