`timescale 1ns / 1ps
`include "udp_cmd_hdr.vh"

// Assemble MPH3 header (+ optional CMD_SEM payload) and serialize ACK.
module uart_cmd_if #(
    parameter integer PAY_MAX = 16,
    parameter [31:0]  MAX_BYTES = `UDP_CMD_MAX_BYTES
) (
    input              clk,
    input              rst_n,

    input      [7:0]   rx_data,
    input              rx_valid,

    output             cmd_valid,
    input              cmd_ready,
    output     [7:0]   cmd,
    output     [7:0]   flags,
    output     [15:0]  seq,
    output     [31:0]  addr,
    output     [31:0]  length,
    output             hdr_ok,
    output             cmd_is_wr,
    output             cmd_is_rd,
    output             cmd_is_xpose,
    output     [8*PAY_MAX-1:0] rx_payload,

    input              ack_start,
    input      [7:0]   ack_cmd,
    input      [7:0]   ack_flags,
    input      [15:0]  ack_seq,
    input      [31:0]  ack_addr,
    input      [31:0]  ack_length,
    input      [8*PAY_MAX-1:0] tx_payload,
    output             ack_busy,

    output             [15:0] rx_ovf_cnt,
    output                    debug_rx_hold,
    // The ACK command latched at ack_start, still valid while S_TX_PAY streams
    // tx_payload -- the core muxes tx_payload between the CS_RB readback and
    // the SEM bridge response on this.
    output     [7:0]          ack_cmd_last,

    output     [7:0]   tx_data,
    output             tx_valid,
    input              tx_ready
);

localparam S_RX_HDR  = 2'd0;
localparam S_RX_PAY  = 2'd1;
localparam S_RX_HOLD = 2'd2;
localparam S_RX_PEEK = 2'd3;
localparam S_TX_IDLE = 2'd0;
localparam S_TX_HDR  = 2'd1;
localparam S_TX_PAY  = 2'd2;

reg  [1:0]  rx_state;
reg  [7:0]  rx_cnt;
reg  [127:0] raw_le;
reg  [8*PAY_MAX-1:0] rx_pay;

reg  [1:0]  tx_state;
reg  [7:0]  tx_cnt;
reg  [127:0] ack_packed_q;
reg  [7:0]  tx_pay_n;
reg  [7:0]  ack_cmd_q;
reg [15:0] ovf_cnt;

wire [31:0] magic_unused;
wire        magic_ok_unused;
wire        addr_align_ok_unused;
wire        len_align_ok_unused;
wire        len_range_ok_unused;
wire [127:0] ack_packed;

udp_cmd_hdr #(.MAX_BYTES(MAX_BYTES)) u_dec (
    .raw_le        (raw_le),
    .magic         (magic_unused),
    .cmd           (cmd),
    .flags         (flags),
    .seq           (seq),
    .addr          (addr),
    .length        (length),
    .magic_ok      (magic_ok_unused),
    .cmd_is_wr     (cmd_is_wr),
    .cmd_is_rd     (cmd_is_rd),
    .cmd_is_xpose  (cmd_is_xpose),
    .addr_align_ok (addr_align_ok_unused),
    .len_align_ok  (len_align_ok_unused),
    .len_range_ok  (len_range_ok_unused),
    .hdr_ok        (hdr_ok),
    .i_cmd         (ack_cmd),
    .i_flags       (ack_flags),
    .i_seq         (ack_seq),
    .i_addr        (ack_addr),
    .i_length      (ack_length),
    .packed_le     (ack_packed)
);

wire [7:0] rx_pay_n = (length > PAY_MAX) ? PAY_MAX[7:0] : length[7:0];

assign cmd_valid     = (rx_state == S_RX_HOLD);
assign ack_busy      = (tx_state != S_TX_IDLE);
assign rx_payload    = rx_pay;
assign rx_ovf_cnt    = ovf_cnt;
assign debug_rx_hold = (rx_state == S_RX_HOLD);
assign ack_cmd_last  = ack_cmd_q;
assign tx_valid      = (tx_state == S_TX_HDR) || (tx_state == S_TX_PAY);
// The SEM ACK payload is read straight out of tx_payload instead of into a
// local shift copy: the bridge sits in S_IDLE with resp_payload held from the
// moment it raises done until the host's next command, and the host does not
// send that command until this ACK lands. Saves a 128-bit register and its
// load mux, which the part cannot spare at 98% LUT.
assign tx_data       = (tx_state == S_TX_PAY) ? tx_payload[8*tx_cnt[3:0] +: 8]
                                              : ack_packed_q[7:0];

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_state <= S_RX_HDR;
        rx_cnt   <= 8'd0;
        raw_le   <= 128'd0;
        rx_pay   <= {8*PAY_MAX{1'b0}};
        ovf_cnt  <= 16'd0;
    end else begin
        case (rx_state)
            S_RX_HDR: begin
                if (rx_valid) begin
                    raw_le[8*rx_cnt[3:0] +: 8] <= rx_data;
                    if (rx_cnt == 8'd15) begin
                        rx_cnt   <= 8'd0;
                        rx_state <= S_RX_PEEK;
                    end else
                        rx_cnt <= rx_cnt + 8'd1;
                end
            end
            S_RX_PEEK: begin
                if (((cmd == `UDP_CMD_SEM) || (cmd == `UDP_CMD_CS_ROW)) &&
                    (rx_pay_n != 8'd0)) begin
                    rx_pay   <= {8*PAY_MAX{1'b0}};
                    rx_cnt   <= 8'd0;
                    rx_state <= S_RX_PAY;
                end else
                    rx_state <= S_RX_HOLD;
            end
            S_RX_PAY: begin
                if (rx_valid) begin
                    // index with 4 bits, not 8: PAY_MAX is 16, and an 8-bit
                    // index makes the synthesizer build a 256-way write decode
                    rx_pay[8*rx_cnt[3:0] +: 8] <= rx_data;
                    if (rx_cnt + 8'd1 >= rx_pay_n) begin
                        rx_cnt   <= 8'd0;
                        rx_state <= S_RX_HOLD;
                    end else
                        rx_cnt <= rx_cnt + 8'd1;
                end
            end
            S_RX_HOLD: begin
                if (cmd_ready)
                    rx_state <= S_RX_HDR;
                if (rx_valid)
                    ovf_cnt <= ovf_cnt + 16'd1;
            end
            default: rx_state <= S_RX_HDR;
        endcase
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx_state     <= S_TX_IDLE;
        tx_cnt       <= 8'd0;
        ack_packed_q <= 128'd0;
        tx_pay_n     <= 8'd0;
        ack_cmd_q    <= 8'd0;
    end else begin
        case (tx_state)
            S_TX_IDLE: begin
                if (ack_start) begin
                    ack_packed_q <= ack_packed;
                    tx_pay_n     <= (ack_length > PAY_MAX) ? PAY_MAX[7:0]
                                    : ack_length[7:0];
                    ack_cmd_q    <= ack_cmd;
                    tx_cnt       <= 8'd0;
                    tx_state     <= S_TX_HDR;
                end
            end
            S_TX_HDR: begin
                if (tx_valid && tx_ready) begin
                    ack_packed_q <= {8'd0, ack_packed_q[127:8]};
                    if (tx_cnt == 8'd15) begin
                        tx_cnt <= 8'd0;
                        if (((ack_cmd_q == `UDP_CMD_SEM_ACK) ||
                         (ack_cmd_q == `UDP_CMD_CS_RB_ACK)) && (tx_pay_n != 8'd0))
                            tx_state <= S_TX_PAY;
                        else
                            tx_state <= S_TX_IDLE;
                    end else
                        tx_cnt <= tx_cnt + 8'd1;
                end
            end
            S_TX_PAY: begin
                if (tx_valid && tx_ready) begin
                    if (tx_cnt + 8'd1 >= tx_pay_n)
                        tx_state <= S_TX_IDLE;
                    else
                        tx_cnt <= tx_cnt + 8'd1;
                end
            end
            default: tx_state <= S_TX_IDLE;
        endcase
    end
end

endmodule
