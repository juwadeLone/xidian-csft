`timescale 1ns / 1ps
// MPH3 CMD_SEM payload <-> SEM monitor FIFO. Does not touch imaging.
// flags[0]=1: dump unsolicited pages, do not send.
// flags[3:1]: page 0..3 of the 64-byte unsol buffer (16 B each).
// Leading 'N'/'n' while SEM not idle -> error string, no inject.
// Non-dump commands clear the unsol write pointer (keep report head).
module sem_monitor_bridge #(
    parameter integer PAY_MAX     = 16,
    parameter integer UNSOL_MAX   = 64,
    parameter integer CLK_HZ      = 100_000_000,
    parameter integer QUIET_US    = 1000
) (
    input                      clk,
    input                      rst_n,

    input                      start,
    input      [7:0]           flags,
    input      [31:0]          timeout_ms,
    input      [31:0]          cmd_len,
    input      [8*PAY_MAX-1:0] cmd_payload,

    input                      status_idle,
    input                      status_injection,

    output     [7:0]           monitor_rxdata,
    input                      monitor_rxread,
    output                     monitor_rxempty,
    input      [7:0]           monitor_txdata,
    input                      monitor_txwrite,
    output                     monitor_txfull,

    output reg                 done,
    output reg                 err,
    output     [7:0]           resp_len,
    output     [8*PAY_MAX-1:0] resp_payload
);

localparam integer QUIET_CYC = (CLK_HZ / 1_000_000) * QUIET_US;
localparam S_IDLE  = 3'd0;
localparam S_SEND  = 3'd1;
localparam S_WAIT  = 3'd2;
localparam S_DONE  = 3'd3;
localparam S_DUMP  = 3'd4;

reg [2:0]  state;
reg [7:0]  send_n;
reg [7:0]  send_i;
reg [7:0]  recv_n;
reg [8*PAY_MAX-1:0] send_buf;
reg [8*PAY_MAX-1:0] recv_buf;
reg [31:0] wait_cnt;
reg [31:0] quiet_cnt;
reg [31:0] to_cyc;
reg        is_ncmd;
reg        saw_tx;
reg        inj_seen;
reg [7:0]  unsol_n;
(* ram_style = "distributed" *)
reg [7:0]  unsol_mem [0:UNSOL_MAX-1];
reg [2:0]  dump_page;
reg [3:0]  dump_i;
reg [7:0]  flags_q;

wire [7:0] dump_base = {2'b00, dump_page[1:0], 4'b0000};
wire [7:0] dump_addr = dump_base + {4'b0, dump_i};

assign resp_len     = recv_n;
assign resp_payload = recv_buf;
assign monitor_rxdata  = send_buf[8*send_i +: 8];
assign monitor_rxempty = ~((state == S_SEND) && (send_i < send_n));
assign monitor_txfull  = 1'b0;

function [8*PAY_MAX-1:0] put_nak;
    input dummy;
    begin
        put_nak = {8*PAY_MAX{1'b0}};
        put_nak[8*0 +: 8] = "N";
        put_nak[8*1 +: 8] = "A";
        put_nak[8*2 +: 8] = "K";
        put_nak[8*3 +: 8] = "_";
        put_nak[8*4 +: 8] = "B";
        put_nak[8*5 +: 8] = "U";
        put_nak[8*6 +: 8] = "S";
        put_nak[8*7 +: 8] = "Y";
        put_nak[8*8 +: 8] = 8'h0D;
    end
endfunction

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state     <= S_IDLE;
        send_n    <= 8'd0;
        send_i    <= 8'd0;
        recv_n    <= 8'd0;
        send_buf  <= {8*PAY_MAX{1'b0}};
        recv_buf  <= {8*PAY_MAX{1'b0}};
        wait_cnt  <= 32'd0;
        quiet_cnt <= 32'd0;
        to_cyc    <= 32'd0;
        is_ncmd   <= 1'b0;
        saw_tx    <= 1'b0;
        inj_seen  <= 1'b0;
        unsol_n   <= 8'd0;
        dump_page <= 3'd0;
        dump_i    <= 4'd0;
        flags_q   <= 8'd0;
        done      <= 1'b0;
        err       <= 1'b0;
    end else begin
        done <= 1'b0;
        if (monitor_txwrite && (unsol_n < UNSOL_MAX[7:0])) begin
            unsol_mem[unsol_n] <= monitor_txdata;
            unsol_n <= unsol_n + 8'd1;
        end
        if (monitor_txwrite && (state == S_WAIT || state == S_SEND)
                && (recv_n < PAY_MAX[7:0])) begin
            recv_buf[8*recv_n +: 8] <= monitor_txdata;
            recv_n <= recv_n + 8'd1;
            saw_tx <= 1'b1;
            quiet_cnt <= 32'd0;
        end else if (state == S_WAIT)
            quiet_cnt <= quiet_cnt + 32'd1;

        case (state)
            S_IDLE: begin
                if (start) begin
                    err      <= 1'b0;
                    send_i   <= 8'd0;
                    saw_tx   <= 1'b0;
                    inj_seen <= 1'b0;
                    wait_cnt <= 32'd0;
                    quiet_cnt<= 32'd0;
                    flags_q  <= flags;
                    to_cyc   <= (timeout_ms == 32'd0) ? 32'd200_000_000
                                : (timeout_ms > 32'd5000) ? 32'd500_000_000
                                : timeout_ms * 32'd100_000;
                    is_ncmd  <= (cmd_len != 32'd0) &&
                                ((cmd_payload[7:0] == "N") || (cmd_payload[7:0] == "n"));
                    if (flags[0]) begin
                        dump_page <= flags[3:1];
                        dump_i    <= 4'd0;
                        recv_n    <= 8'd0;
                        recv_buf  <= {8*PAY_MAX{1'b0}};
                        state     <= S_DUMP;
                    end else if ((cmd_len != 32'd0) &&
                                 ((cmd_payload[7:0] == "N") || (cmd_payload[7:0] == "n")) &&
                                 !status_idle) begin
                        recv_buf <= put_nak(1'b0);
                        recv_n   <= 8'd9;
                        err      <= 1'b1;
                        state    <= S_DONE;
                    end else begin
                        unsol_n  <= 8'd0;
                        send_buf <= cmd_payload;
                        send_n   <= (cmd_len > PAY_MAX) ? PAY_MAX[7:0] : cmd_len[7:0];
                        recv_n   <= 8'd0;
                        recv_buf <= {8*PAY_MAX{1'b0}};
                        state    <= (cmd_len == 32'd0) ? S_WAIT : S_SEND;
                    end
                end
            end
            S_SEND: begin
                if (monitor_rxread && !monitor_rxempty)
                    send_i <= send_i + 8'd1;
                if (send_i >= send_n && !monitor_rxread)
                    state <= S_WAIT;
                wait_cnt <= wait_cnt + 32'd1;
                if (wait_cnt >= to_cyc)
                    state <= S_DONE;
            end
            S_WAIT: begin
                wait_cnt <= wait_cnt + 32'd1;
                if (status_injection)
                    inj_seen <= 1'b1;
                if (wait_cnt >= to_cyc)
                    state <= S_DONE;
                else if (saw_tx && (quiet_cnt >= QUIET_CYC)) begin
                    if (!is_ncmd)
                        state <= S_DONE;
                    else if (inj_seen && status_idle)
                        state <= S_DONE;
                    else if (!inj_seen && status_idle && (quiet_cnt >= (QUIET_CYC * 4)))
                        state <= S_DONE;
                end
            end
            S_DUMP: begin
                recv_buf <= {unsol_mem[dump_addr], recv_buf[8*PAY_MAX-1:8]};
                if (dump_i == 4'd15) begin
                    if (unsol_n <= dump_base)
                        recv_n <= 8'd0;
                    else if (unsol_n >= (dump_base + 8'd16))
                        recv_n <= 8'd16;
                    else
                        recv_n <= unsol_n - dump_base;
                    state <= S_DONE;
                end
                dump_i <= dump_i + 4'd1;
            end
            S_DONE: begin
                done  <= 1'b1;
                state <= S_IDLE;
            end
            default: state <= S_IDLE;
        endcase
    end
end

endmodule
