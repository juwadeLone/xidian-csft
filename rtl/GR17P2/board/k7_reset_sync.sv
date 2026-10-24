`timescale 1ns/1ps
module k7_reset_sync(input wire clk, input wire arst_n, output wire rst_n);
    (* ASYNC_REG = "TRUE" *) reg [2:0] sync = 0;
    always @(posedge clk or negedge arst_n)
        if (!arst_n) sync <= 0;
        else sync <= {sync[1:0],1'b1};
    assign rst_n = sync[2];
endmodule
