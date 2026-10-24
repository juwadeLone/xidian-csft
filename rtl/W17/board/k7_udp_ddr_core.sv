`timescale 1ns/1ps
`include "udp_cmd_hdr.vh"

module k7_udp_ddr_core #(
    parameter integer DIV       = 1085,           // 100 MHz / 115200
    parameter integer ADDR_W    = 31,
    parameter integer HEART_DIV = 125_000_000,   // ~1 Hz at 100 MHz
    // One full 4096x4096 complex frame -- the largest single transfer the
    // design ever needs.  Not a memory bound: the DDR is 2 GiB.  It is the
    // sanity bound that stops a corrupted header from starting a huge
    // transfer, and it has to be raised from the A7's 512 KB, which is what
    // NAKed the first 134 MB write on this board.
    parameter [31:0]  MAX_BYTES = 32'h0800_0000,
    // One DDR-to-DDR FFT pass at the K7's frame side.  2048 and Q1.17 are the
    // sizing decision (results/k7_port/SIZING_DECISION.md).  The per-pass scale
    // masks are the trimmed model's (configs/final_w17t25.json fft_az / fft_rg /
    // ifft_rg / ifft_az).
    parameter integer FFT_LOGN = 11,
    parameter [11:0]  FFT_AZ_SCALE_MASK = 12'h680,
    parameter [11:0]  FFT_RG_SCALE_MASK = 12'h6aa,
    parameter [11:0]  FFT_RG_IFFT_SCALE_MASK = 12'h7ac,
    parameter [11:0]  FFT_AZ_IFFT_SCALE_MASK = 12'h7fa,
    // The seven-pass CS chain, wrapped by csa_cs_app exactly as the A7 and V7
    // board tops instantiate it.  Five frame buffers of SIDE_LOG2 a side:
    // B0..B3 for the passes and OUT for the result, 33.5 MB each at 2048.
    parameter [63:0]  CS_EXPECTED_FOLD = 64'd0,
    // 0 holds the chain in its calibration wait, so it never takes the bus.
    // Only for the end-to-end testbench, whose early WR/RD checks would
    // otherwise stall behind a chain that starts the moment reset lifts.
    parameter integer CS_ENABLE = 1
) (
    input  wire clk,
    input  wire rst_n,
`ifdef CS_CLK_SPLIT
    input  wire cs_clk,         // rpr_iso fallback: csa_cs_app's own clock (cs_xclk.sv)
`endif
    // ICAPE2 clock for the SEM core: the 100 MHz sysclk pin buffered through
    // a BUFG in the top.  Kept separate from clk (125 MHz) -- PG036 requires
    // icap_clk <= 100 MHz, so the processing clock cannot drive it.
    input  wire icap_clk,
    input  wire icap_rst_n,
    input  wire uart_rx_i,
    output wire uart_tx_o,
    input wire calib_done,
    input wire rx_clk, rx_rst,
    input wire gmii_rx_dv,
    input wire [7:0] gmii_rd,
    output wire gmii_tx_dv,
    output wire [7:0] gmii_td,
    output wire [3:0] m_awid,
    output wire [30:0] m_awaddr,
    output wire [7:0] m_awlen,
    output wire [2:0] m_awsize,
    output wire [1:0] m_awburst,
    output wire m_awvalid,
    input wire m_awready,
    output wire [255:0] m_wdata,
    output wire [31:0] m_wstrb,
    output wire m_wlast,
    output wire m_wvalid,
    input wire m_wready,
    input wire [3:0] m_bid,
    input wire [1:0] m_bresp,
    input wire m_bvalid,
    output wire m_bready,
    output wire [3:0] m_arid,
    output wire [30:0] m_araddr,
    output wire [7:0] m_arlen,
    output wire [2:0] m_arsize,
    output wire [1:0] m_arburst,
    output wire m_arvalid,
    input wire m_arready,
    input wire [3:0] m_rid,
    input wire [255:0] m_rdata,
    input wire [1:0] m_rresp,
    input wire m_rlast,
    input wire m_rvalid,
    output wire m_rready
);
    wire rst = ~rst_n;

    wire [7:0] rx_data;
    wire       rx_valid;
    uart_rx #(.DIV(DIV)) u_uart_rx (
        .clk(clk), .rst(rst), .rx(uart_rx_i),
        .data(rx_data), .valid(rx_valid)
    );

    wire        cmd_valid, cmd_ready, hdr_ok;
    wire        cmd_is_wr, cmd_is_rd, cmd_is_xpose;
    wire [7:0]  cmd, flags;
    wire [15:0] seq;
    wire [31:0] cmd_addr, cmd_length;
    wire [127:0] rx_payload;

    // ---- seed-table row write / read-back (host-built tables) ----
    wire                 cs_row_we;
    wire [1:0]           cs_row_sel;
    wire [1:0]           cs_row_tgt;
    wire [10:0]          cs_row_idx;
    wire [95:0]          cs_row_data;
    wire [1:0]           cs_rb_sel;
    wire [10:0]          cs_rb_idx;
    wire                 cs_rb_dbg;
    wire [95:0]          seed_rb_data_w;
    wire [127:0]         cs_rb_tx_payload = {32'd0, seed_rb_data_w};
    wire [127:0]         sem_resp_payload;
    wire [15:0] rx_ovf_cnt;
    wire [7:0]  if_tx_data;
    wire        if_tx_valid, if_tx_ready;
    wire        ack_busy;
    wire [7:0]  ack_cmd_last;
    // S_TX_PAY reads tx_payload live; ack_cmd_last (latched at ack_start)
    // picks the SEM bridge response while that ACK is on the wire.
    wire [127:0] if_tx_payload =
        (ack_cmd_last == `UDP_CMD_SEM_ACK) ? sem_resp_payload
                                          : cs_rb_tx_payload;

    wire        seq_ack_start;
    wire [7:0]  seq_ack_cmd, seq_ack_flags;
    wire [15:0] seq_ack_seq;
    wire [31:0] seq_ack_addr, seq_ack_length;

    wire        telem_start;
    wire [7:0]  telem_cmd, telem_flags;
    wire [15:0] telem_seq;
    wire [31:0] telem_addr, telem_length;

    // ---- SEM status (assembled before uart_cmd_if so the SEM_ACK mux below
    // can use it; the wires themselves are driven at the bottom of the file) ----
    wire        sem_busy;
    wire        sem_status_idle_c, sem_status_injection_c;
    wire        sem_stat_heartbeat, sem_stat_uncorrectable, sem_stat_essential;
    wire        sem_stat_classification, sem_stat_correction;
    wire        sem_stat_observation, sem_stat_initialization;
    wire        sem_icap_request, sem_icap_grant;
    wire        sem_icap_request_c, sem_icap_grant_c;
    wire [15:0] sem_stat16 = {5'd0, sem_icap_grant_c, sem_icap_request_c,
        sem_stat_heartbeat, sem_status_idle_c, sem_stat_uncorrectable,
        sem_stat_essential, sem_status_injection_c, sem_stat_classification,
        sem_stat_correction, sem_stat_observation, sem_stat_initialization};

    // A7 parity: a pending SEM command suppresses the telemetry poll ack so
    // the two cannot interleave on the single UART TX path.
    wire        ack_start = seq_ack_start | (telem_start & ~sem_busy);
    wire [7:0]  ack_cmd    = seq_ack_start ? seq_ack_cmd    : telem_cmd;
    wire [7:0]  ack_flags  = seq_ack_start ? seq_ack_flags  : telem_flags;
    wire [15:0] ack_seq    = seq_ack_start ? seq_ack_seq    : telem_seq;
    wire [31:0] ack_addr   = seq_ack_start ?
        ((seq_ack_cmd == `UDP_CMD_SEM_ACK) ? {sem_stat16, seq_ack_addr[15:0]}
                                          : seq_ack_addr)
        : telem_addr;
    wire [31:0] ack_length = seq_ack_start ? seq_ack_length : telem_length;

    uart_cmd_if #(.MAX_BYTES(MAX_BYTES)) u_uart_cmd_if (
        .clk(clk), .rst_n(rst_n),
        .rx_data(rx_data), .rx_valid(rx_valid),
        .cmd_valid(cmd_valid), .cmd_ready(cmd_ready),
        .cmd(cmd), .flags(flags), .seq(seq),
        .addr(cmd_addr), .length(cmd_length),
        .hdr_ok(hdr_ok),
        .cmd_is_wr(cmd_is_wr), .cmd_is_rd(cmd_is_rd), .cmd_is_xpose(cmd_is_xpose),
        .rx_payload(rx_payload),
        .ack_start(ack_start), .ack_cmd(ack_cmd), .ack_flags(ack_flags),
        .ack_seq(ack_seq), .ack_addr(ack_addr), .ack_length(ack_length),
        .tx_payload(if_tx_payload),
        .ack_busy(ack_busy),
        .rx_ovf_cnt(rx_ovf_cnt),
        .debug_rx_hold(),
        .ack_cmd_last(ack_cmd_last),
        .tx_data(if_tx_data), .tx_valid(if_tx_valid), .tx_ready(if_tx_ready)
    );

    wire [2:0] cmd_state;
    wire       wr_start, swap_start, rd_start, fft_start, cs_build_start, sem_start;
    // application clear on host command (cmd_seq CS_CLEAR, 2026-10-13): a
    // level 32 clocks long, ORed with the reset into the application's clear
    wire       cs_clear;
    // P3: per-frame CS parameter load, cmd_seq -> csa_cs_app -> seed builder
    wire       cs_param_we;
    wire [3:0] cs_param_idx;
    wire [31:0] cs_param_w0, cs_param_w1;
    wire       cs_bld_busy, cs_bld_done, cs_bld_error, cs_params_dirty;
    wire [1:0] fft_mode;
    wire [1:0] mux_sel;
    wire [30:0] op_addr;
    wire [31:0] op_length;
    wire wr_busy, wr_done, wr_error, rd_busy, rd_done, rd_error;
    // Unsupported compute commands fail explicitly in the DDR-only image.
    wire fft_busy, fft_done_pulse, fft_error;   // driven by the CS engines
    // FFT2D and TRANSPOSE no longer have an engine behind them.  cmd_seq waits
    // on (done && !busy), so leaving them unanswered would hang the command
    // layer; they are refused instead.  The pulse is registered because the
    // latches below clear on start with higher priority than they set, so a
    // completion asserted in the same cycle would be swallowed.
    wire swap_busy_w, swap_done_pulse, swap_error_w;
    reg refused_fft, refused_swap;
    always @(posedge clk) begin
        if (!rst_n) begin
            refused_fft  <= 1'b0;
            refused_swap <= 1'b0;
        end else begin
            refused_fft  <= fft_start;
            refused_swap <= swap_start;
        end
    end
    assign fft_busy       = 1'b0;
    assign fft_done_pulse = refused_fft;
    assign fft_error      = 1'b1;
    assign swap_busy_w    = 1'b0;
    assign swap_done_pulse = refused_swap;
    assign swap_error_w   = 1'b1;

    // csa_cs_app's done is a level, not a pulse: it is simply (state ==
    // ST_IDLE), so it is already high before a run starts.  Latching it the way
    // the pulsed engines needed would set it again the moment the start pulse
    // cleared it, and cmd_seq would acknowledge a run that had not begun.  Gate
    // it on having left idle instead.
    // All of csa_cs_app's status lives here, above cmd_seq and telem_uart,
    // because both reference it and SystemVerilog has no implicit nets.  The
    // instance itself is further down with the rest of the datapath.
    wire cs_app_busy, cs_app_done, cs_app_error;
`ifdef CS_CLK_SPLIT
    wire cs_app_busy_ran;
`endif
    wire cs_app_hash_ok, cs_app_image_done, cs_app_preload_done;
    // CS_PARAM flags[4], latched by cmd_seq: 1 = the current run takes its RAW
    // frame from what the host wrote into B0 instead of the fabric preload.

    // The app is command-started only (CS_PARAM flags[7]): the power-on
    // auto-run of the verification era is gone together with the fabric
    // preload.  After calibration the app sits in ST_IDLE and takes the outer
    // mux whenever it is busy; when it returns to ST_IDLE it has no
    // transaction outstanding, so handing the mux back to cmd_seq there is
    // safe.  Since 2026-10-13 the app also stays busy in its error state for
    // as long as the chain or the result check is still working (csa_cs_app:
    // busy), so the mux is not taken from a chain that is in mid-transfer.
    wire [1:0] bus_sel = cs_app_busy ? 2'd1 : mux_sel;
    wire cs_app_protocol_error, cs_app_overflow_sticky, cs_app_calib_sync;
    wire [2:0]  cs_app_pass_id, cs_app_fsm_state;
    wire [63:0] cs_app_fold;
    reg  cs_app_ran;
    always @(posedge clk) begin
        if (!rst_n)              cs_app_ran <= 1'b0;
        else if (cs_build_start) cs_app_ran <= 1'b0;
`ifdef CS_CLK_SPLIT
        else if (cs_app_busy_ran) cs_app_ran <= 1'b1;   // cs_xclk: busy, 3 cycles later
`else
        else if (cs_app_busy)    cs_app_ran <= 1'b1;
`endif
    end
    // Masked on cs_build_start.  cmd_seq raises cs_build_start and enters
    // S_WAIT_B on the same edge, so it samples this in the very next cycle --
    // before cs_app_ran has been cleared and before the app has left ST_IDLE.
    // Unmasked, a rerun saw the previous frame's (ran && idle) and was
    // acknowledged one cycle after it was issued, measured at 0.020 s on the
    // board.  The data was never wrong, because bus_sel holds the bus for the
    // chain until it is idle, but the acknowledgement meant nothing.
    wire cs_app_done_l = cs_app_ran && cs_app_done && !cs_build_start;
    reg  swap_done_l;
    always @(posedge clk) begin
        if (!rst_n)               swap_done_l <= 1'b0;
        else if (swap_start)      swap_done_l <= 1'b0;
        else if (swap_done_pulse) swap_done_l <= 1'b1;
    end
    // axi_fft_pass_engine pulses done for a single cycle when its reader and
    // writer close, and stays busy after that while the lane drains -- the
    // module testbenches wait on the two separately, in that order.  cmd_seq
    // waits for (fft_done && !fft_busy) at the same instant, which the pulse
    // never satisfies, so wiring done straight through would hang in S_WAIT_F
    // forever.  Latch it, and clear the latch when a new pass starts so a
    // stale completion cannot acknowledge the next command immediately.
    reg fft_done_l;
    always @(posedge clk) begin
        if (!rst_n)             fft_done_l <= 1'b0;
        else if (fft_start)     fft_done_l <= 1'b0;
        else if (fft_done_pulse) fft_done_l <= 1'b1;
    end
    wire fft_done = fft_done_l;
    wire sem_done, sem_err;
    wire [7:0] sem_resp_len;
    wire [127:0] wr_tdata, rd_tdata;
    wire wr_tvalid, wr_tready, rd_tvalid, rd_tready;
    cmd_seq #(.ADDR_W(ADDR_W)) u_cmd_seq (
        .clk(clk), .rst_n(rst_n), .calib_done(calib_done),
        .cmd_valid(cmd_valid), .cmd_ready(cmd_ready),
        .cmd(cmd), .flags(flags), .seq(seq),
        .addr(cmd_addr), .length(cmd_length),
        .hdr_ok(hdr_ok),
        .cmd_is_wr(cmd_is_wr), .cmd_is_rd(cmd_is_rd), .cmd_is_xpose(cmd_is_xpose),
        .ack_start(seq_ack_start), .ack_cmd(seq_ack_cmd),
        .ack_flags(seq_ack_flags), .ack_seq(seq_ack_seq),
        .ack_addr(seq_ack_addr), .ack_length(seq_ack_length),
        .ack_busy(ack_busy),
        .mux_sel(mux_sel), .op_addr(op_addr), .op_length(op_length), .wr_err_sticky(),
        .wr_start(wr_start), .wr_busy(wr_busy), .wr_done(wr_done), .wr_error(wr_error),
        .swap_start(swap_start), .swap_busy(swap_busy_w),
        .swap_done(swap_done_l), .swap_error(swap_error_w),
        .rd_start(rd_start), .rd_busy(rd_busy), .rd_done(rd_done), .rd_error(rd_error),
        .fft_mode(fft_mode), .fft_start(fft_start),
        .fft_busy(fft_busy), .fft_done(fft_done), .fft_error(fft_error),
        .debug_state(cmd_state),
        .cs_a_hcs(), .cs_b_hcs(), .cs_c_hcs(),
        .cs_a_hr(), .cs_b_hr(), .cs_c_hr(),
        .cs_a_ha(), .cs_b_ha(), .cs_c_ha(),
        .cs_tab_we(), .cs_tab_sel(), .cs_tab_fa(),
        .cs_tab_a(), .cs_tab_b(), .cs_tab_c(),
        .cs_param_we(cs_param_we), .cs_param_idx(cs_param_idx),
        .cs_param_w0(cs_param_w0), .cs_param_w1(cs_param_w1),
        .cs_build_start(cs_build_start),
        .cs_build_busy(cs_app_busy), .cs_build_done(cs_app_done_l),
        .cs_build_error(cs_app_error),
        .cs_clear(cs_clear),
        .sem_busy(sem_busy), .sem_start(sem_start),
        .sem_done(sem_done), .sem_err(sem_err), .sem_resp_len(sem_resp_len),
        .rx_payload(rx_payload),
        .cs_row_we(cs_row_we), .cs_row_sel(cs_row_sel), .cs_row_tgt(cs_row_tgt),
        .cs_row_idx(cs_row_idx), .cs_row_data(cs_row_data),
        .cs_rb_sel(cs_rb_sel), .cs_rb_idx(cs_rb_idx),
        .cs_rb_dbg(cs_rb_dbg)
    );

    localparam integer HEART_W = (HEART_DIV <= 2) ? 2 : $clog2(HEART_DIV);
    reg [HEART_W-1:0] heart_div;
    reg [7:0]         heart_flag;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            heart_div  <= {HEART_W{1'b0}};
            heart_flag <= 8'd0;
        end else if (heart_div == HEART_W'(HEART_DIV - 1)) begin
            heart_div  <= {HEART_W{1'b0}};
            heart_flag <= heart_flag + 8'd1;
        end else begin
            heart_div <= heart_div + 1'b1;
        end
    end

    telem_uart u_telem_uart (
        .clk(clk), .rst_n(rst_n),
        .heart_flag(heart_flag),
        .dbg_app({cs_app_fsm_state[2], flags[7], cs_app_fsm_state[0]}),
        .calib_done(calib_done),
        .cmd_state(cmd_state),
        // The routing actually in force, not cmd_seq's request: while the chain
        // holds the bus these differ, and reporting the request would send the
        // next diagnosis the wrong way.
        .mux_sel(bus_sel),
        .wr_busy(wr_busy), .swap_busy(cs_app_busy), .rd_busy(rd_busy),
        .wr_error(wr_error), .swap_error(cs_app_error), .rd_error(rd_error),
        .wr_beat(wr_tvalid & wr_tready), .rd_beat(rd_tvalid & rd_tready),
        // The standalone transpose is gone, so these three telemetry slots
        // carry the chain: swap_busy/swap_error/swap_done read as the CS
        // app's busy, error and image_done.  No host-side change needed,
        // and without them a chain that stalls shows nothing at all.
        .swap_done(cs_app_image_done),
        .ack_busy(ack_busy),
        .cmd_ack_start(seq_ack_start),
        .telem_start(telem_start),
        .telem_cmd(telem_cmd),
        .telem_flags(telem_flags),
        .telem_seq(telem_seq),
        .telem_addr(telem_addr),
        .telem_length(telem_length)
    );

    wire       send_en, tx_done;
    wire [7:0] glue_data;
    uart_tx_glue u_glue (
        .clk(clk), .rst_n(rst_n),
        .s_tdata(if_tx_data), .s_tvalid(if_tx_valid), .s_tready(if_tx_ready),
        .send_en(send_en), .tx_data(glue_data), .tx_done(tx_done)
    );

    uart_tx #(.DIV(DIV)) u_uart_tx (
        .clk(clk), .rst(rst),
        .send(send_en), .data(glue_data),
        .tx(uart_tx_o), .busy(), .done(tx_done)
    );


    // A7 host protocol is unchanged: 192.168.0.4 / 00:11:22:33:44:55,
    // UDP 8080. The host sends raw Ethernet on its 192.168.137.1 NIC.
    wire [47:0] rx_mac;
    wire [31:0] rx_ip;
    udp_eth_rx u_eth_rx (
        .clk(rx_clk), .rst(rx_rst), .I_sys_clk(clk), .I_sys_rst_n(rst_n),
        .gmii_rx_dv(gmii_rx_dv), .gmii_rd(gmii_rd),
        .status(), .fil_data(), .fil_data_vld(),
        .pld_tdata(wr_tdata), .pld_tvalid(wr_tvalid), .pld_tready(wr_tready),
        .pc_mac(rx_mac), .pc_ip(rx_ip)
    );
    // Atomic peer snapshot across RX -> processing clock, not an unsynchronised bus.
    reg [79:0] peer_sent;
    wire peer_full, peer_valid;
    wire [79:0] peer_out;
    wire peer_write = !rx_rst && !peer_full && rx_mac != 0 && {rx_mac,rx_ip} != peer_sent;
    always @(posedge rx_clk) begin
        if (rx_rst) peer_sent <= 0;
        else if (peer_write) peer_sent <= {rx_mac,rx_ip};
    end
    FWFT_Async_fifo #(.WRITE_DATA_WIDTH(80), .READ_DATA_WIDTH(80),
        .FIFO_WRITE_DEPTH(32), .FIFO_MEMORY_TYPE("distributed")) u_peer_fifo (
        .rst(rx_rst), .wr_clk(rx_clk), .rd_clk(clk),
        .din({rx_mac,rx_ip}), .wr_en(peer_write), .full(peer_full),
        .rd_en(peer_valid), .dout(peer_out), .data_valid(peer_valid)
    );
    reg [79:0] peer;
    always @(posedge clk) begin
        if (!rst_n) peer <= 0;
        else if (peer_valid) peer <= peer_out;
    end
    wire wr_protocol_error, rd_protocol_error;
    wire [3:0] wr_awid;
    wire [30:0] wr_awaddr;
    wire [7:0] wr_awlen;
    wire [2:0] wr_awsize;
    wire [1:0] wr_awburst;
    wire wr_awvalid;
    wire wr_awready;
    wire [127:0] wr_wdata;
    wire [15:0] wr_wstrb;
    wire wr_wlast;
    wire wr_wvalid;
    wire wr_wready;
    wire [3:0] wr_bid;
    wire [1:0] wr_bresp;
    wire wr_bvalid;
    wire wr_bready;
    wire [3:0] wr_arid;
    wire [30:0] wr_araddr;
    wire [7:0] wr_arlen;
    wire [2:0] wr_arsize;
    wire [1:0] wr_arburst;
    wire wr_arvalid;
    wire wr_arready;
    wire [3:0] wr_rid;
    wire [127:0] wr_rdata;
    wire [1:0] wr_rresp;
    wire wr_rlast;
    wire wr_rvalid;
    wire wr_rready;
    wire [3:0] rd_awid;
    wire [30:0] rd_awaddr;
    wire [7:0] rd_awlen;
    wire [2:0] rd_awsize;
    wire [1:0] rd_awburst;
    wire rd_awvalid;
    wire rd_awready;
    wire [127:0] rd_wdata;
    wire [15:0] rd_wstrb;
    wire rd_wlast;
    wire rd_wvalid;
    wire rd_wready;
    wire [3:0] rd_bid;
    wire [1:0] rd_bresp;
    wire rd_bvalid;
    wire rd_bready;
    wire [3:0] rd_arid;
    wire [30:0] rd_araddr;
    wire [7:0] rd_arlen;
    wire [2:0] rd_arsize;
    wire [1:0] rd_arburst;
    wire rd_arvalid;
    wire rd_arready;
    wire [3:0] rd_rid;
    wire [127:0] rd_rdata;
    wire [1:0] rd_rresp;
    wire rd_rlast;
    wire rd_rvalid;
    wire rd_rready;
    wire [3:0] uw_awid;
    wire [30:0] uw_awaddr;
    wire [7:0] uw_awlen;
    wire [2:0] uw_awsize;
    wire [1:0] uw_awburst;
    wire uw_awvalid;
    wire uw_awready;
    wire [255:0] uw_wdata;
    wire [31:0] uw_wstrb;
    wire uw_wlast;
    wire uw_wvalid;
    wire uw_wready;
    wire [3:0] uw_bid;
    wire [1:0] uw_bresp;
    wire uw_bvalid;
    wire uw_bready;
    wire [3:0] uw_arid;
    wire [30:0] uw_araddr;
    wire [7:0] uw_arlen;
    wire [2:0] uw_arsize;
    wire [1:0] uw_arburst;
    wire uw_arvalid;
    wire uw_arready;
    wire [3:0] uw_rid;
    wire [255:0] uw_rdata;
    wire [1:0] uw_rresp;
    wire uw_rlast;
    wire uw_rvalid;
    wire uw_rready;
    wire [3:0] ur_awid;
    wire [30:0] ur_awaddr;
    wire [7:0] ur_awlen;
    wire [2:0] ur_awsize;
    wire [1:0] ur_awburst;
    wire ur_awvalid;
    wire ur_awready;
    wire [255:0] ur_wdata;
    wire [31:0] ur_wstrb;
    wire ur_wlast;
    wire ur_wvalid;
    wire ur_wready;
    wire [3:0] ur_bid;
    wire [1:0] ur_bresp;
    wire ur_bvalid;
    wire ur_bready;
    wire [3:0] ur_arid;
    wire [30:0] ur_araddr;
    wire [7:0] ur_arlen;
    wire [2:0] ur_arsize;
    wire [1:0] ur_arburst;
    wire ur_arvalid;
    wire ur_arready;
    wire [3:0] ur_rid;
    wire [255:0] ur_rdata;
    wire [1:0] ur_rresp;
    wire ur_rlast;
    wire ur_rvalid;
    wire ur_rready;
    udp_wr_master  u_wr (
        .clk(clk),
        .rst_n(rst_n),
        .start(wr_start),
        .addr(op_addr),
        .length(op_length),
        .busy(wr_busy),
        .done(wr_done),
        .error(wr_error),
        .dbg_state(),
        .s_tdata(wr_tdata),
        .s_tvalid(wr_tvalid),
        .s_tready(wr_tready),
        .m_awid(wr_awid),
        .m_awaddr(wr_awaddr),
        .m_awlen(wr_awlen),
        .m_awsize(wr_awsize),
        .m_awburst(wr_awburst),
        .m_awvalid(wr_awvalid),
        .m_awready(wr_awready),
        .m_wdata(wr_wdata),
        .m_wstrb(wr_wstrb),
        .m_wlast(wr_wlast),
        .m_wvalid(wr_wvalid),
        .m_wready(wr_wready),
        .m_bid(wr_bid),
        .m_bresp(wr_bresp),
        .m_bvalid(wr_bvalid),
        .m_bready(wr_bready)
    );
    wire rd_axi_busy, rd_axi_done, rd_axi_error, tx_busy;
    udp_rd_master  u_rd (
        .clk(clk),
        .rst_n(rst_n),
        .start(rd_start),
        .addr(op_addr),
        .length(op_length),
        .busy(rd_axi_busy),
        .done(rd_axi_done),
        .error(rd_axi_error),
        .dbg_state(),
        .m_tdata(rd_tdata),
        .m_tvalid(rd_tvalid),
        .m_tready(rd_tready),
        .m_tlast(),
        .m_arid(rd_arid),
        .m_araddr(rd_araddr),
        .m_arlen(rd_arlen),
        .m_arsize(rd_arsize),
        .m_arburst(rd_arburst),
        .m_arvalid(rd_arvalid),
        .m_arready(rd_arready),
        .m_rid(rd_rid),
        .m_rdata(rd_rdata),
        .m_rresp(rd_rresp),
        .m_rlast(rd_rlast),
        .m_rvalid(rd_rvalid),
        .m_rready(rd_rready)
    );
    assign wr_arid = '0;
    assign wr_araddr = '0;
    assign wr_arlen = '0;
    assign wr_arsize = '0;
    assign wr_arburst = '0;
    assign wr_arvalid = '0;
    assign wr_rready = '0;
    assign rd_awid = '0;
    assign rd_awaddr = '0;
    assign rd_awlen = '0;
    assign rd_awsize = '0;
    assign rd_awburst = '0;
    assign rd_awvalid = '0;
    assign rd_wdata = '0;
    assign rd_wstrb = '0;
    assign rd_wlast = '0;
    assign rd_wvalid = '0;
    assign rd_bready = '0;
    axi_dwidth_128to256  u_wr_gear (
        .clk(clk),
        .rst_n(rst_n),
        .protocol_error(wr_protocol_error),
        .s_awid(wr_awid),
        .s_awaddr(wr_awaddr),
        .s_awlen(wr_awlen),
        .s_awsize(wr_awsize),
        .s_awburst(wr_awburst),
        .s_awvalid(wr_awvalid),
        .s_awready(wr_awready),
        .s_wdata(wr_wdata),
        .s_wstrb(wr_wstrb),
        .s_wlast(wr_wlast),
        .s_wvalid(wr_wvalid),
        .s_wready(wr_wready),
        .s_bid(wr_bid),
        .s_bresp(wr_bresp),
        .s_bvalid(wr_bvalid),
        .s_bready(wr_bready),
        .s_arid(wr_arid),
        .s_araddr(wr_araddr),
        .s_arlen(wr_arlen),
        .s_arsize(wr_arsize),
        .s_arburst(wr_arburst),
        .s_arvalid(wr_arvalid),
        .s_arready(wr_arready),
        .s_rid(wr_rid),
        .s_rdata(wr_rdata),
        .s_rresp(wr_rresp),
        .s_rlast(wr_rlast),
        .s_rvalid(wr_rvalid),
        .s_rready(wr_rready),
        .m_awid(uw_awid),
        .m_awaddr(uw_awaddr),
        .m_awlen(uw_awlen),
        .m_awsize(uw_awsize),
        .m_awburst(uw_awburst),
        .m_awvalid(uw_awvalid),
        .m_awready(uw_awready),
        .m_wdata(uw_wdata),
        .m_wstrb(uw_wstrb),
        .m_wlast(uw_wlast),
        .m_wvalid(uw_wvalid),
        .m_wready(uw_wready),
        .m_bid(uw_bid),
        .m_bresp(uw_bresp),
        .m_bvalid(uw_bvalid),
        .m_bready(uw_bready),
        .m_arid(uw_arid),
        .m_araddr(uw_araddr),
        .m_arlen(uw_arlen),
        .m_arsize(uw_arsize),
        .m_arburst(uw_arburst),
        .m_arvalid(uw_arvalid),
        .m_arready(uw_arready),
        .m_rid(uw_rid),
        .m_rdata(uw_rdata),
        .m_rresp(uw_rresp),
        .m_rlast(uw_rlast),
        .m_rvalid(uw_rvalid),
        .m_rready(uw_rready)
    );
    axi_dwidth_128to256  u_rd_gear (
        .clk(clk),
        .rst_n(rst_n),
        .protocol_error(rd_protocol_error),
        .s_awid(rd_awid),
        .s_awaddr(rd_awaddr),
        .s_awlen(rd_awlen),
        .s_awsize(rd_awsize),
        .s_awburst(rd_awburst),
        .s_awvalid(rd_awvalid),
        .s_awready(rd_awready),
        .s_wdata(rd_wdata),
        .s_wstrb(rd_wstrb),
        .s_wlast(rd_wlast),
        .s_wvalid(rd_wvalid),
        .s_wready(rd_wready),
        .s_bid(rd_bid),
        .s_bresp(rd_bresp),
        .s_bvalid(rd_bvalid),
        .s_bready(rd_bready),
        .s_arid(rd_arid),
        .s_araddr(rd_araddr),
        .s_arlen(rd_arlen),
        .s_arsize(rd_arsize),
        .s_arburst(rd_arburst),
        .s_arvalid(rd_arvalid),
        .s_arready(rd_arready),
        .s_rid(rd_rid),
        .s_rdata(rd_rdata),
        .s_rresp(rd_rresp),
        .s_rlast(rd_rlast),
        .s_rvalid(rd_rvalid),
        .s_rready(rd_rready),
        .m_awid(ur_awid),
        .m_awaddr(ur_awaddr),
        .m_awlen(ur_awlen),
        .m_awsize(ur_awsize),
        .m_awburst(ur_awburst),
        .m_awvalid(ur_awvalid),
        .m_awready(ur_awready),
        .m_wdata(ur_wdata),
        .m_wstrb(ur_wstrb),
        .m_wlast(ur_wlast),
        .m_wvalid(ur_wvalid),
        .m_wready(ur_wready),
        .m_bid(ur_bid),
        .m_bresp(ur_bresp),
        .m_bvalid(ur_bvalid),
        .m_bready(ur_bready),
        .m_arid(ur_arid),
        .m_araddr(ur_araddr),
        .m_arlen(ur_arlen),
        .m_arsize(ur_arsize),
        .m_arburst(ur_arburst),
        .m_arvalid(ur_arvalid),
        .m_arready(ur_arready),
        .m_rid(ur_rid),
        .m_rdata(ur_rdata),
        .m_rresp(ur_rresp),
        .m_rlast(ur_rlast),
        .m_rvalid(ur_rvalid),
        .m_rready(ur_rready)
    );
    // ---- the CS datapath: one master, the seven-pass chain ----
    //
    // Slots for a standalone FFT pass, inverse pass and transpose used to sit
    // here.  They were how each segment was brought up and checked against the
    // golden model on the board, one at a time, and that job is done -- the
    // evidence is in results/k7_port.  They are gone now because they were
    // duplicate hardware: csa_cs_app carries its own four coefficient pass
    // engines and its own transpose, and an injection campaign has to hit
    // those, since they are the ones that produce OUT.  Keeping a second set
    // outside the chain cost 144 DSP and 80 block RAM tiles and could not be
    // injected into usefully.
    //
    // With a single CS master there is no CS-side mux either; the chain drives
    // the outer 3-to-1 mux's slot 1 directly.
    wire [3:0]        fp_awid, fp_bid, fp_arid, fp_rid;
    wire [ADDR_W-1:0] fp_awaddr, fp_araddr;
    wire [7:0]        fp_awlen, fp_arlen;
    wire [2:0]        fp_awsize, fp_arsize;
    wire [1:0]        fp_awburst, fp_bresp, fp_arburst, fp_rresp;
    wire              fp_awvalid, fp_awready, fp_wlast, fp_wvalid, fp_wready;
    wire              fp_bvalid, fp_bready;
    wire [255:0]      fp_wdata, fp_rdata;
    wire [31:0]       fp_wstrb;
    wire              fp_arvalid, fp_arready, fp_rlast, fp_rvalid, fp_rready;

`ifdef CS_CLK_SPLIT
    // ---- rpr_iso fallback: csa_cs_app on cs_clk ----------------------------------
    // Control and status cross in cs_xclk; the app's AXI master (ap_*, cs_clk)
    // crosses into fp_* (clk) in a second instance of the AXI clock converter.
    wire cs_rst, cs_xclk_lost;
    wire a_start, a_clear, a_init_calib_complete, a_cs_param_we, a_seed_wr_en, a_seed_rb_dbg;
    wire [3:0]  a_cs_param_idx;
    wire [31:0] a_cs_param_w0, a_cs_param_w1;
    wire [1:0]  a_seed_wr_sel, a_seed_wr_tgt, a_seed_rb_sel;
    wire [FFT_LOGN-1:0] a_seed_wr_idx, a_seed_rb_idx;
    wire [95:0] a_seed_wr_data, a_seed_rb_data;
    wire a_cs_build_busy, a_cs_build_done, a_cs_build_error, a_params_dirty, a_calib_sync,
         a_preload_done, a_busy, a_done, a_error, a_protocol_error, a_overflow_sticky,
         a_hash_ok, a_image_done;
    wire [2:0]  a_active_pass_id, a_fsm_state;
    wire [63:0] a_fold;
    cs_xclk #(.SIDE_LOG2(FFT_LOGN)) u_cs_xclk (
        .clk(clk), .cs_clk(cs_clk), .rst(rst), .cs_rst(cs_rst),
        .start(cs_build_start), .clear(!rst_n || cs_clear), .init_calib_complete(calib_done && (CS_ENABLE != 0)),
        .cs_param_we(cs_param_we), .cs_param_idx(cs_param_idx),
        .cs_param_w0(cs_param_w0), .cs_param_w1(cs_param_w1),
        .seed_wr_en(cs_row_we), .seed_wr_sel(cs_row_sel), .seed_wr_idx(cs_row_idx),
        .seed_wr_data(cs_row_data), .seed_wr_tgt(cs_row_tgt),
        .seed_rb_sel(cs_rb_sel), .seed_rb_idx(cs_rb_idx), .seed_rb_dbg(cs_rb_dbg),
        .cs_build_busy(cs_bld_busy), .cs_build_done(cs_bld_done), .cs_build_error(cs_bld_error),
        .params_dirty(cs_params_dirty), .seed_rb_data(seed_rb_data_w),
        .calib_sync(cs_app_calib_sync), .preload_done(cs_app_preload_done),
        .busy(cs_app_busy), .busy_ran(cs_app_busy_ran), .done(cs_app_done), .error(cs_app_error),
        .protocol_error(cs_app_protocol_error), .overflow_sticky(cs_app_overflow_sticky),
        .active_pass_id(cs_app_pass_id), .hash_ok(cs_app_hash_ok), .fold(cs_app_fold),
        .image_done(cs_app_image_done), .fsm_state(cs_app_fsm_state), .lost(cs_xclk_lost),
        .a_start(a_start), .a_clear(a_clear), .a_init_calib_complete(a_init_calib_complete),
        .a_cs_param_we(a_cs_param_we), .a_cs_param_idx(a_cs_param_idx),
        .a_cs_param_w0(a_cs_param_w0), .a_cs_param_w1(a_cs_param_w1),
        .a_seed_wr_en(a_seed_wr_en), .a_seed_wr_sel(a_seed_wr_sel), .a_seed_wr_idx(a_seed_wr_idx),
        .a_seed_wr_data(a_seed_wr_data), .a_seed_wr_tgt(a_seed_wr_tgt),
        .a_seed_rb_sel(a_seed_rb_sel), .a_seed_rb_idx(a_seed_rb_idx), .a_seed_rb_dbg(a_seed_rb_dbg),
        .a_cs_build_busy(a_cs_build_busy), .a_cs_build_done(a_cs_build_done),
        .a_cs_build_error(a_cs_build_error), .a_params_dirty(a_params_dirty),
        .a_seed_rb_data(a_seed_rb_data), .a_calib_sync(a_calib_sync), .a_preload_done(a_preload_done),
        .a_busy(a_busy), .a_done(a_done), .a_error(a_error), .a_protocol_error(a_protocol_error),
        .a_overflow_sticky(a_overflow_sticky), .a_active_pass_id(a_active_pass_id),
        .a_hash_ok(a_hash_ok), .a_fold(a_fold), .a_image_done(a_image_done), .a_fsm_state(a_fsm_state));
    wire [3:0]        ap_awid, ap_bid, ap_arid, ap_rid;
    wire [ADDR_W-1:0] ap_awaddr, ap_araddr;
    wire [7:0]        ap_awlen, ap_arlen;
    wire [2:0]        ap_awsize, ap_arsize;
    wire [1:0]        ap_awburst, ap_bresp, ap_arburst, ap_rresp;
    wire              ap_awvalid, ap_awready, ap_wlast, ap_wvalid, ap_wready;
    wire              ap_bvalid, ap_bready;
    wire [255:0]      ap_wdata, ap_rdata;
    wire [31:0]       ap_wstrb;
    wire              ap_arvalid, ap_arready, ap_rlast, ap_rvalid, ap_rready;
    k7_axi_clock_converter u_cs_axi_cdc (
        .s_axi_aclk(cs_clk), .s_axi_aresetn(!cs_rst),
        .m_axi_aclk(clk), .m_axi_aresetn(rst_n),
        .s_axi_awid(ap_awid), .s_axi_awaddr(ap_awaddr), .s_axi_awlen(ap_awlen),
        .s_axi_awsize(ap_awsize), .s_axi_awburst(ap_awburst), .s_axi_awvalid(ap_awvalid),
        .s_axi_awready(ap_awready), .s_axi_wdata(ap_wdata), .s_axi_wstrb(ap_wstrb),
        .s_axi_wlast(ap_wlast), .s_axi_wvalid(ap_wvalid), .s_axi_wready(ap_wready),
        .s_axi_bid(ap_bid), .s_axi_bresp(ap_bresp), .s_axi_bvalid(ap_bvalid), .s_axi_bready(ap_bready),
        .s_axi_arid(ap_arid), .s_axi_araddr(ap_araddr), .s_axi_arlen(ap_arlen),
        .s_axi_arsize(ap_arsize), .s_axi_arburst(ap_arburst), .s_axi_arvalid(ap_arvalid),
        .s_axi_arready(ap_arready), .s_axi_rid(ap_rid), .s_axi_rdata(ap_rdata),
        .s_axi_rresp(ap_rresp), .s_axi_rlast(ap_rlast), .s_axi_rvalid(ap_rvalid), .s_axi_rready(ap_rready),
        .m_axi_awid(fp_awid), .m_axi_awaddr(fp_awaddr), .m_axi_awlen(fp_awlen),
        .m_axi_awsize(fp_awsize), .m_axi_awburst(fp_awburst), .m_axi_awvalid(fp_awvalid),
        .m_axi_awready(fp_awready), .m_axi_wdata(fp_wdata), .m_axi_wstrb(fp_wstrb),
        .m_axi_wlast(fp_wlast), .m_axi_wvalid(fp_wvalid), .m_axi_wready(fp_wready),
        .m_axi_bid(fp_bid), .m_axi_bresp(fp_bresp), .m_axi_bvalid(fp_bvalid), .m_axi_bready(fp_bready),
        .m_axi_arid(fp_arid), .m_axi_araddr(fp_araddr), .m_axi_arlen(fp_arlen),
        .m_axi_arsize(fp_arsize), .m_axi_arburst(fp_arburst), .m_axi_arvalid(fp_arvalid),
        .m_axi_arready(fp_arready), .m_axi_rid(fp_rid), .m_axi_rdata(fp_rdata),
        .m_axi_rresp(fp_rresp), .m_axi_rlast(fp_rlast), .m_axi_rvalid(fp_rvalid), .m_axi_rready(fp_rready),
        .s_axi_awlock(1'b0), .m_axi_awlock(), .s_axi_awcache(4'b0011), .m_axi_awcache(),
        .s_axi_awprot(3'b0), .m_axi_awprot(), .s_axi_awqos(4'b0), .m_axi_awqos(),
        .s_axi_arlock(1'b0), .m_axi_arlock(), .s_axi_arcache(4'b0011), .m_axi_arcache(),
        .s_axi_arprot(3'b0), .m_axi_arprot(), .s_axi_arqos(4'b0), .m_axi_arqos());
`endif

    // csa_cs_app is what both csa_cs256_board_top and csa_cs4096_board_top
    // instantiate; this follows them rather than assembling the chain here.
    // It brings its own four coefficient pass engines, its own transpose, the
    // seed and window driver, the raw preload and the output scoreboard, and
    // presents one AXI master.
    //
    // Production input path: the host writes B0 over UDP, then CS_PARAM
    // flags[7] runs the chain on it.  The fabric pattern preload of the
    // verification era was removed on 2026-09-11.
    localparam integer CS_SIDE_LOG2 = FFT_LOGN;
    localparam [ADDR_W-1:0] CS_FB = {{(ADDR_W-1-2*CS_SIDE_LOG2-3){1'b0}},
                                     1'b1, {(2*CS_SIDE_LOG2+3){1'b0}}};
    localparam [ADDR_W-1:0] CS_BASE_B0  = {ADDR_W{1'b0}};
    localparam [ADDR_W-1:0] CS_BASE_B1  = CS_FB;
    localparam [ADDR_W-1:0] CS_BASE_B2  = CS_FB * 2;
    localparam [ADDR_W-1:0] CS_BASE_B3  = CS_FB * 3;
    localparam [ADDR_W-1:0] CS_BASE_OUT = CS_FB * 4;

    // K7_MOD_KIND is unset on the production seven-pass build, so the
    // hierarchy stays u_cs_app = csa_cs_app (baseline_v1 unchanged).
    // Module campaigns set verilog_define K7_MOD_KIND=1|2|3.
`ifdef K7_MOD_KIND
    k7_mod_app #(
        .DUT_KIND(`K7_MOD_KIND),
        .SIDE_LOG2(CS_SIDE_LOG2), .AZ_SCALE_MASK(FFT_AZ_SCALE_MASK), .RG_SCALE_MASK(FFT_RG_SCALE_MASK),
        .ADDR_W(ADDR_W), .ID_W(4),
        .BASE_B0(CS_BASE_B0), .BASE_B1(CS_BASE_B1), .BASE_B2(CS_BASE_B2),
        .BASE_B3(CS_BASE_B3), .BASE_OUT(CS_BASE_OUT),
        .EXPECTED_FOLD(CS_EXPECTED_FOLD),
        .H1_MEM("cs2048_h1_seeds.mem"), .H2_MEM("cs2048_h2_seeds.mem"),
        .H3_MEM("cs2048_h3_seeds.mem"), .C4_MEM("cs2048_c4_seeds.mem"),
        .SYNC_SEED_ROM(1)
    ) u_cs_app
`else
    csa_cs_app #(
        .SIDE_LOG2(CS_SIDE_LOG2), .AZ_SCALE_MASK(FFT_AZ_SCALE_MASK), .RG_SCALE_MASK(FFT_RG_SCALE_MASK),
        .RG_IFFT_SCALE_MASK(FFT_RG_IFFT_SCALE_MASK), .AZ_IFFT_SCALE_MASK(FFT_AZ_IFFT_SCALE_MASK),
        .ADDR_W(ADDR_W), .ID_W(4),
        .BASE_B0(CS_BASE_B0), .BASE_B1(CS_BASE_B1), .BASE_B2(CS_BASE_B2),
        .BASE_B3(CS_BASE_B3), .BASE_OUT(CS_BASE_OUT),
        .EXPECTED_FOLD(CS_EXPECTED_FOLD),
        .H1_MEM("cs2048_h1_seeds.mem"), .H2_MEM("cs2048_h2_seeds.mem"),
        .H3_MEM("cs2048_h3_seeds.mem"), .C4_MEM("cs2048_c4_seeds.mem"),
        .SYNC_SEED_ROM(1)
    ) u_cs_app
`endif
`ifdef CS_CLK_SPLIT
    (
        .clk(cs_clk), .rst(cs_rst), .init_calib_complete(a_init_calib_complete),
        .start(a_start), .clear(a_clear),
        .cs_param_we(a_cs_param_we), .cs_param_idx(a_cs_param_idx),
        .cs_param_w0(a_cs_param_w0), .cs_param_w1(a_cs_param_w1),
        .cs_build_busy(a_cs_build_busy), .cs_build_done(a_cs_build_done),
        .cs_build_error(a_cs_build_error), .params_dirty(a_params_dirty),
        .seed_wr_en(a_seed_wr_en), .seed_wr_sel(a_seed_wr_sel),
        .seed_wr_idx(a_seed_wr_idx), .seed_wr_data(a_seed_wr_data),
        .seed_wr_tgt(a_seed_wr_tgt),
        .seed_rb_sel(a_seed_rb_sel), .seed_rb_idx(a_seed_rb_idx),
        .seed_rb_dbg(a_seed_rb_dbg),
        .seed_rb_data(a_seed_rb_data),
        .calib_sync(a_calib_sync), .preload_done(a_preload_done),
        .busy(a_busy), .done(a_done), .error(a_error),
        .protocol_error(a_protocol_error),
        .overflow_sticky(a_overflow_sticky),
        .active_pass_id(a_active_pass_id), .hash_ok(a_hash_ok),
        .fold(a_fold), .image_done(a_image_done),
        .fsm_state(a_fsm_state),
        .m_axi_awid(ap_awid),
        .m_axi_awaddr(ap_awaddr),
        .m_axi_awlen(ap_awlen),
        .m_axi_awsize(ap_awsize),
        .m_axi_awburst(ap_awburst),
        .m_axi_awvalid(ap_awvalid), .m_axi_awready(ap_awready),
        .m_axi_wdata(ap_wdata),
        .m_axi_wstrb(ap_wstrb),
        .m_axi_wlast(ap_wlast),
        .m_axi_wvalid(ap_wvalid), .m_axi_wready(ap_wready),
        .m_axi_bid(ap_bid),
        .m_axi_bresp(ap_bresp),
        .m_axi_bvalid(ap_bvalid), .m_axi_bready(ap_bready),
        .m_axi_arid(ap_arid),
        .m_axi_araddr(ap_araddr),
        .m_axi_arlen(ap_arlen),
        .m_axi_arsize(ap_arsize),
        .m_axi_arburst(ap_arburst),
        .m_axi_arvalid(ap_arvalid), .m_axi_arready(ap_arready),
        .m_axi_rid(ap_rid),
        .m_axi_rdata(ap_rdata),
        .m_axi_rresp(ap_rresp),
        .m_axi_rlast(ap_rlast),
        .m_axi_rvalid(ap_rvalid), .m_axi_rready(ap_rready)
    );
`else
    (
        .clk(clk), .rst(rst), .init_calib_complete(calib_done && (CS_ENABLE != 0)),
        .start(cs_build_start), .clear(!rst_n || cs_clear),
        .cs_param_we(cs_param_we), .cs_param_idx(cs_param_idx),
        .cs_param_w0(cs_param_w0), .cs_param_w1(cs_param_w1),
        .cs_build_busy(cs_bld_busy), .cs_build_done(cs_bld_done),
        .cs_build_error(cs_bld_error), .params_dirty(cs_params_dirty),
        .seed_wr_en(cs_row_we), .seed_wr_sel(cs_row_sel),
        .seed_wr_idx(cs_row_idx), .seed_wr_data(cs_row_data),
        .seed_wr_tgt(cs_row_tgt),
        .seed_rb_sel(cs_rb_sel), .seed_rb_idx(cs_rb_idx),
        .seed_rb_dbg(cs_rb_dbg),
        .seed_rb_data(seed_rb_data_w),
        .calib_sync(cs_app_calib_sync), .preload_done(cs_app_preload_done),
        .busy(cs_app_busy), .done(cs_app_done), .error(cs_app_error),
        .protocol_error(cs_app_protocol_error),
        .overflow_sticky(cs_app_overflow_sticky),
        .active_pass_id(cs_app_pass_id), .hash_ok(cs_app_hash_ok),
        .fold(cs_app_fold), .image_done(cs_app_image_done),
        .fsm_state(cs_app_fsm_state),
        .m_axi_awid(fp_awid),
        .m_axi_awaddr(fp_awaddr),
        .m_axi_awlen(fp_awlen),
        .m_axi_awsize(fp_awsize),
        .m_axi_awburst(fp_awburst),
        .m_axi_awvalid(fp_awvalid), .m_axi_awready(fp_awready),
        .m_axi_wdata(fp_wdata),
        .m_axi_wstrb(fp_wstrb),
        .m_axi_wlast(fp_wlast),
        .m_axi_wvalid(fp_wvalid), .m_axi_wready(fp_wready),
        .m_axi_bid(fp_bid),
        .m_axi_bresp(fp_bresp),
        .m_axi_bvalid(fp_bvalid), .m_axi_bready(fp_bready),
        .m_axi_arid(fp_arid),
        .m_axi_araddr(fp_araddr),
        .m_axi_arlen(fp_arlen),
        .m_axi_arsize(fp_arsize),
        .m_axi_arburst(fp_arburst),
        .m_axi_arvalid(fp_arvalid), .m_axi_arready(fp_arready),
        .m_axi_rid(fp_rid),
        .m_axi_rdata(fp_rdata),
        .m_axi_rresp(fp_rresp),
        .m_axi_rlast(fp_rlast),
        .m_axi_rvalid(fp_rvalid), .m_axi_rready(fp_rready)
    );
`endif

    // the mux belongs to the memory interface (zone 1, pblock pb_mem); its cells
    // keep the u_mux/ prefix so the pblock and the zone check find them
    (* keep_hierarchy = "yes" *)
    axi_mux_3to1_256  u_mux (
        .clk(clk),
        .rst_n(rst_n),
        .sel(bus_sel),
        .s0_awid(uw_awid),
        .s0_awaddr(uw_awaddr),
        .s0_awlen(uw_awlen),
        .s0_awsize(uw_awsize),
        .s0_awburst(uw_awburst),
        .s0_awvalid(uw_awvalid),
        .s0_awready(uw_awready),
        .s0_wdata(uw_wdata),
        .s0_wstrb(uw_wstrb),
        .s0_wlast(uw_wlast),
        .s0_wvalid(uw_wvalid),
        .s0_wready(uw_wready),
        .s0_bid(uw_bid),
        .s0_bresp(uw_bresp),
        .s0_bvalid(uw_bvalid),
        .s0_bready(uw_bready),
        .s0_arid(uw_arid),
        .s0_araddr(uw_araddr),
        .s0_arlen(uw_arlen),
        .s0_arsize(uw_arsize),
        .s0_arburst(uw_arburst),
        .s0_arvalid(uw_arvalid),
        .s0_arready(uw_arready),
        .s0_rid(uw_rid),
        .s0_rdata(uw_rdata),
        .s0_rresp(uw_rresp),
        .s0_rlast(uw_rlast),
        .s0_rvalid(uw_rvalid),
        .s0_rready(uw_rready),
        .s1_awid(fp_awid),
        .s1_awaddr(fp_awaddr),
        .s1_awlen(fp_awlen),
        .s1_awsize(fp_awsize),
        .s1_awburst(fp_awburst),
        .s1_awvalid(fp_awvalid),
        .s1_awready(fp_awready),
        .s1_wdata(fp_wdata),
        .s1_wstrb(fp_wstrb),
        .s1_wlast(fp_wlast),
        .s1_wvalid(fp_wvalid),
        .s1_wready(fp_wready),
        .s1_bid(fp_bid),
        .s1_bresp(fp_bresp),
        .s1_bvalid(fp_bvalid),
        .s1_bready(fp_bready),
        .s1_arid(fp_arid),
        .s1_araddr(fp_araddr),
        .s1_arlen(fp_arlen),
        .s1_arsize(fp_arsize),
        .s1_arburst(fp_arburst),
        .s1_arvalid(fp_arvalid),
        .s1_arready(fp_arready),
        .s1_rid(fp_rid),
        .s1_rdata(fp_rdata),
        .s1_rresp(fp_rresp),
        .s1_rlast(fp_rlast),
        .s1_rvalid(fp_rvalid),
        .s1_rready(fp_rready),
        .s2_awid(ur_awid),
        .s2_awaddr(ur_awaddr),
        .s2_awlen(ur_awlen),
        .s2_awsize(ur_awsize),
        .s2_awburst(ur_awburst),
        .s2_awvalid(ur_awvalid),
        .s2_awready(ur_awready),
        .s2_wdata(ur_wdata),
        .s2_wstrb(ur_wstrb),
        .s2_wlast(ur_wlast),
        .s2_wvalid(ur_wvalid),
        .s2_wready(ur_wready),
        .s2_bid(ur_bid),
        .s2_bresp(ur_bresp),
        .s2_bvalid(ur_bvalid),
        .s2_bready(ur_bready),
        .s2_arid(ur_arid),
        .s2_araddr(ur_araddr),
        .s2_arlen(ur_arlen),
        .s2_arsize(ur_arsize),
        .s2_arburst(ur_arburst),
        .s2_arvalid(ur_arvalid),
        .s2_arready(ur_arready),
        .s2_rid(ur_rid),
        .s2_rdata(ur_rdata),
        .s2_rresp(ur_rresp),
        .s2_rlast(ur_rlast),
        .s2_rvalid(ur_rvalid),
        .s2_rready(ur_rready),
        .m_awid(m_awid),
        .m_awaddr(m_awaddr),
        .m_awlen(m_awlen),
        .m_awsize(m_awsize),
        .m_awburst(m_awburst),
        .m_awvalid(m_awvalid),
        .m_awready(m_awready),
        .m_wdata(m_wdata),
        .m_wstrb(m_wstrb),
        .m_wlast(m_wlast),
        .m_wvalid(m_wvalid),
        .m_wready(m_wready),
        .m_bid(m_bid),
        .m_bresp(m_bresp),
        .m_bvalid(m_bvalid),
        .m_bready(m_bready),
        .m_arid(m_arid),
        .m_araddr(m_araddr),
        .m_arlen(m_arlen),
        .m_arsize(m_arsize),
        .m_arburst(m_arburst),
        .m_arvalid(m_arvalid),
        .m_arready(m_arready),
        .m_rid(m_rid),
        .m_rdata(m_rdata),
        .m_rresp(m_rresp),
        .m_rlast(m_rlast),
        .m_rvalid(m_rvalid),
        .m_rready(m_rready)
    );

    wire fifo_full, fifo_valid;
    wire [7:0] tx_byte;
    wire tx_ready;
    FWFT_sync_fifo #(.WRITE_DATA_WIDTH(128), .READ_DATA_WIDTH(8),
        .FIFO_WRITE_DEPTH(128)) u_tx_payload (
        .clk(clk), .rst(!rst_n), .din(rd_tdata),
        .wr_en(rd_tvalid && rd_tready), .full(fifo_full),
        .rd_en(tx_ready && fifo_valid), .dout(tx_byte), .data_valid(fifo_valid)
    );
    assign rd_tready = !fifo_full;
    assign rd_busy = rd_axi_busy | tx_busy;
    assign rd_error = rd_axi_error | rd_protocol_error;
    wire [128:0] tx_status;
    wire tx_fil_valid;
    wire [7:0] tx_fil_data;
    udp_tx_engine u_tx_engine (
        .clk(clk), .rst(!rst_n), .start(rd_start), .length(op_length),
        .pc_mac(peer[79:32]), .pc_ip(peer[31:0]), .busy(tx_busy), .done(rd_done),
        .pld_tdata(tx_byte), .pld_tvalid(fifo_valid), .pld_tready(tx_ready),
        .status(tx_status), .fil_data_vld(tx_fil_valid), .fil_data(tx_fil_data)
    );
    udp_eth_tx u_eth_tx (
        .clk(clk), .rst(!rst_n), .status(tx_status),
        .fil_data_vld(tx_fil_valid), .fil_data(tx_fil_data),
        .po_data_vld(gmii_tx_dv), .po_data(gmii_td)
    );

    // ===================== SEM (PG036) CRAM injection channel ==============
    // Same arrangement as the A7 campaign build: sem_0 + ICAPE2 in
    // sem_wrapper, ASCII monitor commands relayed by sem_monitor_bridge.
    // The difference on K7 is the clock split: cmd_seq is in the 125 MHz
    // processing domain while ICAPE2 must run at <= 100 MHz, so the whole
    // SEM subsystem sits on icap_clk and only the request/response bundle
    // crosses, through sem_cmd_cdc's toggle handshake.
    wire        status_heartbeat, status_initialization, status_observation;
    wire        status_correction, status_classification, status_essential;
    wire        status_uncorrectable, status_idle, status_injection;
    wire [7:0]  mon_rxdata, mon_txdata;
    wire        mon_rxread, mon_rxempty, mon_txwrite, mon_txfull;
    wire        s_sem_start, s_sem_done, s_sem_err;
    wire [7:0]  s_sem_flags, s_sem_resp_len;
    wire [31:0] s_sem_timeout, s_sem_len;
    wire [127:0] s_sem_payload, s_sem_resp;

    sem_cmd_cdc u_sem_cdc (
        .c_clk(clk), .c_rst_n(rst_n),
        .c_start(sem_start), .c_flags(flags),
        .c_timeout_ms(cmd_addr), .c_cmd_len(cmd_length),
        .c_cmd_payload(rx_payload),
        .c_done(sem_done), .c_err(sem_err), .c_resp_len(sem_resp_len),
        .c_resp_payload(sem_resp_payload),
        .s_clk(icap_clk), .s_rst_n(icap_rst_n),
        .s_start(s_sem_start), .s_flags(s_sem_flags),
        .s_timeout_ms(s_sem_timeout), .s_cmd_len(s_sem_len),
        .s_cmd_payload(s_sem_payload),
        .s_done(s_sem_done), .s_err(s_sem_err),
        .s_resp_len(s_sem_resp_len), .s_resp_payload(s_sem_resp)
    );

    sem_monitor_bridge #(.CLK_HZ(100_000_000)) u_sem_bridge (
        .clk(icap_clk), .rst_n(icap_rst_n),
        .start(s_sem_start), .flags(s_sem_flags), .timeout_ms(s_sem_timeout),
        .cmd_len(s_sem_len), .cmd_payload(s_sem_payload),
        .status_idle(status_idle), .status_injection(status_injection),
        .monitor_rxdata(mon_rxdata), .monitor_rxread(mon_rxread),
        .monitor_rxempty(mon_rxempty),
        .monitor_txdata(mon_txdata), .monitor_txwrite(mon_txwrite),
        .monitor_txfull(mon_txfull),
        .done(s_sem_done), .err(s_sem_err),
        .resp_len(s_sem_resp_len), .resp_payload(s_sem_resp)
    );

    sem_wrapper u_sem_wrapper (
        .icap_clk(icap_clk), .icap_resetn(icap_rst_n),
        .status_heartbeat(status_heartbeat),
        .status_initialization(status_initialization),
        .status_observation(status_observation),
        .status_correction(status_correction),
        .status_classification(status_classification),
        .status_injection(status_injection),
        .status_essential(status_essential),
        .status_uncorrectable(status_uncorrectable),
        .status_idle(status_idle),
        .icap_request(sem_icap_request), .icap_grant(sem_icap_grant),
        .monitor_txdata(mon_txdata), .monitor_txwrite(mon_txwrite),
        .monitor_txfull(mon_txfull),
        .monitor_rxdata(mon_rxdata), .monitor_rxread(mon_rxread),
        .monitor_rxempty(mon_rxempty)
    );

    // SEM status is quasi-static; two-flop each bit into the command domain
    // for the SEM_ACK stat16 field.
    (* ASYNC_REG = "TRUE" *) reg [10:0] sem_stat_m, sem_stat_s;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sem_stat_m <= 11'd0;
            sem_stat_s <= 11'd0;
        end else begin
            sem_stat_m <= {status_heartbeat, status_idle, status_uncorrectable,
                           status_essential, status_injection,
                           status_classification, status_correction,
                           status_observation, status_initialization,
                           sem_icap_request, sem_icap_grant};
            sem_stat_s <= sem_stat_m;
        end
    end
    assign {sem_stat_heartbeat, sem_status_idle_c, sem_stat_uncorrectable,
            sem_stat_essential, sem_status_injection_c, sem_stat_classification,
            sem_stat_correction, sem_stat_observation,
            sem_stat_initialization, sem_icap_request_c, sem_icap_grant_c}
        = sem_stat_s;
endmodule
