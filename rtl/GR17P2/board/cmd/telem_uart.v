`timescale 1ns / 1ps
`include "udp_cmd_hdr.vh"

// 1 Hz unsolicited MPH3 telemetry on UART (cmd=0x84).
// Command ACKs win if both want the TX serializer.
//
// flags[7:0]:
//   [1:0] mux_sel  [2] wr_busy  [3] swap_busy  [4] rd_busy
//   [5] wr_err     [6] swap_err [7] rd_err
// seq[15:0]:
//   [2:0] cmd_state  [3] calib  [7:4] swap_done_cnt  [15:8] heart
// addr  = wr_bytes (saturating)
// length= rd_bytes (saturating)
module telem_uart (
    input              clk,
    input              rst_n,
    input      [7:0]   heart_flag,
    input      [2:0]   dbg_app,        // {cs_app err, CS_PARAM flags[7], cs_app idle}
    input              calib_done,
    input      [2:0]   cmd_state,
    input      [1:0]   mux_sel,
    input              wr_busy,
    input              swap_busy,
    input              rd_busy,
    input              wr_error,
    input              swap_error,
    input              rd_error,
    input              wr_beat,
    input              rd_beat,
    input              swap_done,
    input              ack_busy,
    input              cmd_ack_start,

    output reg         telem_start,
    output     [7:0]   telem_cmd,
    output     [7:0]   telem_flags,
    output     [15:0]  telem_seq,
    output     [31:0]  telem_addr,
    output     [31:0]  telem_length
);

reg [7:0]  heart_d;
reg        pending;
reg [31:0] wr_bytes;
reg [31:0] rd_bytes;
reg [3:0]  swap_done_cnt;

wire tick = (heart_flag != heart_d);

assign telem_cmd    = `UDP_CMD_TELEM;
assign telem_flags  = {rd_error, swap_error, wr_error, rd_busy, swap_busy, wr_busy, mux_sel};
// [15:13] dbg_app (app error / run-command seen / app idle) replaces the top
        // three heart bits: the host treats heart as a free-running
        // counter, and losing 3 bits of it costs nothing here.
assign telem_seq    = {dbg_app, heart_flag[4:0], swap_done_cnt, calib_done, cmd_state};
assign telem_addr   = wr_bytes;
assign telem_length = rd_bytes;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        heart_d        <= 8'd0;
        pending        <= 1'b0;
        wr_bytes       <= 32'd0;
        rd_bytes       <= 32'd0;
        swap_done_cnt  <= 4'd0;
        telem_start    <= 1'b0;
    end else begin
        heart_d     <= heart_flag;
        telem_start <= 1'b0;

        if (wr_beat && wr_bytes <= 32'hFFFF_FFF0)
            wr_bytes <= wr_bytes + 32'd16;
        if (rd_beat && rd_bytes <= 32'hFFFF_FFF0)
            rd_bytes <= rd_bytes + 32'd16;
        if (swap_done && swap_done_cnt != 4'hF)
            swap_done_cnt <= swap_done_cnt + 4'd1;

        if (pending && !ack_busy && !cmd_ack_start) begin
            telem_start <= 1'b1;
            pending     <= tick;
        end else if (tick) begin
            pending <= 1'b1;
        end
    end
end

endmodule
