`timescale 1ns/1ps
// Command-handshake CDC for the SEM monitor bridge.
//
// cmd_seq runs in the 125 MHz processing clock; the SEM IP and its monitor
// bridge run on icap_clk (100 MHz sysclk, the ICAPE2 limit).  The command
// bundle (flags/timeout/len/payload) plus start/done therefore cross a clock
// boundary that the A7 single-domain wiring never had.
//
// Toggle handshake, safe by protocol: cmd_seq holds its request in S_WAIT_SEM
// until done returns, so the c_* registers below are frozen for the whole
// transaction; the s-side samples them directly (multi-cycle stable bus).
// Likewise the response registers are held from s_done until the next
// transaction completes, so the c side reads them after the ack toggle edge.
module sem_cmd_cdc (
    // ---- requester domain (cmd_seq clock) ----
    input  wire         c_clk,
    input  wire         c_rst_n,
    input  wire         c_start,        // 1-cycle pulse
    input  wire [7:0]   c_flags,
    input  wire [31:0]  c_timeout_ms,
    input  wire [31:0]  c_cmd_len,
    input  wire [127:0] c_cmd_payload,
    output reg          c_done,         // 1-cycle pulse
    output wire         c_err,
    output wire [7:0]   c_resp_len,
    output wire [127:0] c_resp_payload,
    // ---- SEM domain (icap_clk) ----
    input  wire         s_clk,
    input  wire         s_rst_n,
    output reg          s_start,        // 1-cycle pulse into bridge
    output wire [7:0]   s_flags,
    output wire [31:0]  s_timeout_ms,
    output wire [31:0]  s_cmd_len,
    output wire [127:0] s_cmd_payload,
    input  wire         s_done,         // 1-cycle pulse from bridge
    input  wire         s_err,
    input  wire [7:0]   s_resp_len,
    input  wire [127:0] s_resp_payload
);

    // c -> s : request bundle, latched once per transaction and held.
    reg        req_tgl;
    reg [7:0]  flags_q;
    reg [31:0] timeout_q;
    reg [31:0] len_q;
    reg [127:0] payload_q;
    always @(posedge c_clk or negedge c_rst_n) begin
        if (!c_rst_n) begin
            req_tgl   <= 1'b0;
            flags_q   <= 8'd0;
            timeout_q <= 32'd0;
            len_q     <= 32'd0;
            payload_q <= 128'd0;
        end else if (c_start) begin
            req_tgl   <= ~req_tgl;
            flags_q   <= c_flags;
            timeout_q <= c_timeout_ms;
            len_q     <= c_cmd_len;
            payload_q <= c_cmd_payload;
        end
    end

    (* ASYNC_REG = "TRUE" *) reg [2:0] req_sync;
    reg req_d;
    always @(posedge s_clk or negedge s_rst_n) begin
        if (!s_rst_n) begin
            req_sync <= 3'b000;
            req_d    <= 1'b0;
            s_start  <= 1'b0;
        end else begin
            req_sync <= {req_sync[1:0], req_tgl};
            req_d    <= req_sync[2];
            s_start  <= (req_sync[2] ^ req_d);
        end
    end

    // s -> c : response bundle, latched at bridge done and held.
    reg         ack_tgl;
    reg         err_q;
    reg [7:0]   resp_len_q;
    reg [127:0] resp_payload_q;
    always @(posedge s_clk or negedge s_rst_n) begin
        if (!s_rst_n) begin
            ack_tgl         <= 1'b0;
            err_q           <= 1'b0;
            resp_len_q      <= 8'd0;
            resp_payload_q  <= 128'd0;
        end else if (s_done) begin
            ack_tgl         <= ~ack_tgl;
            err_q           <= s_err;
            resp_len_q      <= s_resp_len;
            resp_payload_q  <= s_resp_payload;
        end
    end

    (* ASYNC_REG = "TRUE" *) reg [2:0] ack_sync;
    reg ack_d;
    always @(posedge c_clk or negedge c_rst_n) begin
        if (!c_rst_n) begin
            ack_sync <= 3'b000;
            ack_d    <= 1'b0;
            c_done   <= 1'b0;
        end else begin
            ack_sync <= {ack_sync[1:0], ack_tgl};
            ack_d    <= ack_sync[2];
            c_done   <= (ack_sync[2] ^ ack_d);
        end
    end

    assign s_flags        = flags_q;
    assign s_timeout_ms   = timeout_q;
    assign s_cmd_len      = len_q;
    assign s_cmd_payload  = payload_q;
    assign c_err          = err_q;
    assign c_resp_len     = resp_len_q;
    assign c_resp_payload = resp_payload_q;

endmodule
