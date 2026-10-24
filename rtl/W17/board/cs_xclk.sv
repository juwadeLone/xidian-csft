`timescale 1ns/1ps

// rpr_iso clock-split fallback (`CS_CLK_SPLIT, 2026-10-02): csa_cs_app on its
// own clock cs_clk (100 or 80 MHz) while the network, UART, SEM, cmd_seq and
// the AXI mux stay on clk (125 MHz; RGMII at 1 Gb/s needs it).  This module
// carries every control and status signal between cmd_seq's side (clk) and
// the app's side (cs_clk); the AXI master crosses in an AXI clock converter
// beside it (k7_udp_ddr_core).  Used only when timing at 125 MHz does not
// close; the clock is a board limit, not part of the study.
//
//   start                       one-cycle pulse        xpm_cdc_pulse
//   cs_param_we + idx/w0/w1     one-cycle write        xpm_cdc_handshake
//   seed_wr_en + sel/tgt/idx/data  one-cycle write     xpm_cdc_handshake
//   clear, init_calib_complete  levels                 xpm_cdc_single
//   seed_rb_sel/idx/dbg         quasi-static (set long before the UART reply
//                               reads seed_rb_data)    xpm_cdc_array_single
//   status levels and buses     levels / quasi-static  xpm_cdc_single / _array_single
//
// A write that arrives while the previous one is still crossing (a few cs_clk
// cycles) is not accepted; cmd_seq issues them per UART command, milliseconds
// apart.  `lost` latches if that ever happens.
//
// busy / done: the app's done is a level (state == idle).  The two cross in
// separate synchronisers, so busy_s is delayed by three more clk cycles before
// it may mark the app as having run (cs_app_ran in k7_udp_ddr_core): by then
// done_s has followed the app out of idle.
module cs_xclk #(
    parameter integer SIDE_LOG2 = 11
) (
    input  wire                     clk,
    input  wire                     cs_clk,
    input  wire                     rst,            // clk domain, active high
    output wire                     cs_rst,         // cs_clk domain
    // clk side (cmd_seq)
    input  wire                     start,
    input  wire                     clear,
    input  wire                     init_calib_complete,
    input  wire                     cs_param_we,
    input  wire [3:0]               cs_param_idx,
    input  wire [31:0]              cs_param_w0,
    input  wire [31:0]              cs_param_w1,
    input  wire                     seed_wr_en,
    input  wire [1:0]               seed_wr_sel,
    input  wire [SIDE_LOG2-1:0]     seed_wr_idx,
    input  wire [95:0]              seed_wr_data,
    input  wire [1:0]               seed_wr_tgt,
    input  wire [1:0]               seed_rb_sel,
    input  wire [SIDE_LOG2-1:0]     seed_rb_idx,
    input  wire                     seed_rb_dbg,
    output wire                     cs_build_busy,
    output wire                     cs_build_done,
    output wire                     cs_build_error,
    output wire                     params_dirty,
    output wire [95:0]              seed_rb_data,
    output wire                     calib_sync,
    output wire                     preload_done,
    output wire                     busy,
    output wire                     busy_ran,       // busy, three cycles later (cs_app_ran)
    output wire                     done,
    output wire                     error,
    output wire                     protocol_error,
    output wire                     overflow_sticky,
    output wire [2:0]               active_pass_id,
    output wire                     hash_ok,
    output wire [63:0]              fold,
    output wire                     image_done,
    output wire [2:0]               fsm_state,
    output reg                      lost,
    // cs_clk side (the app)
    output wire                     a_start,
    output wire                     a_clear,
    output wire                     a_init_calib_complete,
    output wire                     a_cs_param_we,
    output wire [3:0]               a_cs_param_idx,
    output wire [31:0]              a_cs_param_w0,
    output wire [31:0]              a_cs_param_w1,
    output wire                     a_seed_wr_en,
    output wire [1:0]               a_seed_wr_sel,
    output wire [SIDE_LOG2-1:0]     a_seed_wr_idx,
    output wire [95:0]              a_seed_wr_data,
    output wire [1:0]               a_seed_wr_tgt,
    output wire [1:0]               a_seed_rb_sel,
    output wire [SIDE_LOG2-1:0]     a_seed_rb_idx,
    output wire                     a_seed_rb_dbg,
    input  wire                     a_cs_build_busy,
    input  wire                     a_cs_build_done,
    input  wire                     a_cs_build_error,
    input  wire                     a_params_dirty,
    input  wire [95:0]              a_seed_rb_data,
    input  wire                     a_calib_sync,
    input  wire                     a_preload_done,
    input  wire                     a_busy,
    input  wire                     a_done,
    input  wire                     a_error,
    input  wire                     a_protocol_error,
    input  wire                     a_overflow_sticky,
    input  wire [2:0]               a_active_pass_id,
    input  wire                     a_hash_ok,
    input  wire [63:0]              a_fold,
    input  wire                     a_image_done,
    input  wire [2:0]               a_fsm_state
);
    // ---- reset into cs_clk ------------------------------------------------------
    xpm_cdc_async_rst #(.DEST_SYNC_FF(4), .RST_ACTIVE_HIGH(1)) u_rst (
        .src_arst(rst), .dest_clk(cs_clk), .dest_arst(cs_rst));

    // ---- start pulse ------------------------------------------------------------
    xpm_cdc_pulse #(.DEST_SYNC_FF(4), .REG_OUTPUT(1), .RST_USED(1)) u_start (
        .src_clk(clk), .src_rst(rst), .src_pulse(start),
        .dest_clk(cs_clk), .dest_rst(cs_rst), .dest_pulse(a_start));

    // ---- levels and quasi-static inputs -----------------------------------------
    localparam integer LV_IN = 2 + 2 + SIDE_LOG2 + 1;
    wire [LV_IN-1:0] lv_in_s;
    xpm_cdc_array_single #(.DEST_SYNC_FF(4), .WIDTH(LV_IN), .SRC_INPUT_REG(0)) u_lv_in (
        .src_clk(clk), .src_in({clear, init_calib_complete, seed_rb_sel, seed_rb_idx, seed_rb_dbg}),
        .dest_clk(cs_clk), .dest_out(lv_in_s));
    assign {a_clear, a_init_calib_complete, a_seed_rb_sel, a_seed_rb_idx, a_seed_rb_dbg} = lv_in_s;

    // ---- one-cycle writes with data ---------------------------------------------
    localparam integer PW = 4 + 32 + 32;
    localparam integer RW = 2 + 2 + SIDE_LOG2 + 96;
    reg  p_send, r_send;
    reg  [PW-1:0] p_data;
    reg  [RW-1:0] r_data;
    wire p_rcv, r_rcv, p_req, r_req;
    wire [PW-1:0] p_out;
    wire [RW-1:0] r_out;
    always @(posedge clk) begin
        if (rst) begin
            p_send <= 1'b0; r_send <= 1'b0; lost <= 1'b0;
        end else begin
            if (cs_param_we) begin
                if (!p_send && !p_rcv) begin
                    p_send <= 1'b1; p_data <= {cs_param_idx, cs_param_w0, cs_param_w1};
                end else lost <= 1'b1;
            end else if (p_send && p_rcv) p_send <= 1'b0;
            if (seed_wr_en) begin
                if (!r_send && !r_rcv) begin
                    r_send <= 1'b1; r_data <= {seed_wr_sel, seed_wr_tgt, seed_wr_idx, seed_wr_data};
                end else lost <= 1'b1;
            end else if (r_send && r_rcv) r_send <= 1'b0;
        end
    end
    xpm_cdc_handshake #(.DEST_EXT_HSK(0), .DEST_SYNC_FF(4), .SRC_SYNC_FF(4), .WIDTH(PW)) u_param (
        .src_clk(clk), .src_in(p_data), .src_send(p_send), .src_rcv(p_rcv),
        .dest_clk(cs_clk), .dest_req(p_req), .dest_ack(1'b0), .dest_out(p_out));
    xpm_cdc_handshake #(.DEST_EXT_HSK(0), .DEST_SYNC_FF(4), .SRC_SYNC_FF(4), .WIDTH(RW)) u_row (
        .src_clk(clk), .src_in(r_data), .src_send(r_send), .src_rcv(r_rcv),
        .dest_clk(cs_clk), .dest_req(r_req), .dest_ack(1'b0), .dest_out(r_out));
    // dest_req stays high for the whole handshake: one write per rising edge
    reg p_req_q, r_req_q;
    always @(posedge cs_clk) begin
        if (cs_rst) begin p_req_q <= 1'b0; r_req_q <= 1'b0; end
        else begin p_req_q <= p_req; r_req_q <= r_req; end
    end
    assign a_cs_param_we = p_req && !p_req_q;
    assign {a_cs_param_idx, a_cs_param_w0, a_cs_param_w1} = p_out;
    assign a_seed_wr_en = r_req && !r_req_q;
    assign {a_seed_wr_sel, a_seed_wr_tgt, a_seed_wr_idx, a_seed_wr_data} = r_out;

    // ---- status back to clk ---------------------------------------------------------
    localparam integer LV_OUT = 13;
    wire [LV_OUT-1:0] lv_out_s;
    xpm_cdc_array_single #(.DEST_SYNC_FF(4), .WIDTH(LV_OUT), .SRC_INPUT_REG(0)) u_lv_out (
        .src_clk(cs_clk),
        .src_in({a_cs_build_busy, a_cs_build_done, a_cs_build_error, a_params_dirty, a_calib_sync,
                 a_preload_done, a_busy, a_done, a_error, a_protocol_error, a_overflow_sticky,
                 a_hash_ok, a_image_done}),
        .dest_clk(clk), .dest_out(lv_out_s));
    assign {cs_build_busy, cs_build_done, cs_build_error, params_dirty, calib_sync,
            preload_done, busy, done, error, protocol_error, overflow_sticky,
            hash_ok, image_done} = lv_out_s;
    localparam integer BUS_OUT = 96 + 3 + 64 + 3;
    wire [BUS_OUT-1:0] bus_s;
    xpm_cdc_array_single #(.DEST_SYNC_FF(4), .WIDTH(BUS_OUT), .SRC_INPUT_REG(0)) u_bus_out (
        .src_clk(cs_clk), .src_in({a_seed_rb_data, a_active_pass_id, a_fold, a_fsm_state}),
        .dest_clk(clk), .dest_out(bus_s));
    assign {seed_rb_data, active_pass_id, fold, fsm_state} = bus_s;
    reg [2:0] busy_d;
    always @(posedge clk) busy_d <= {busy_d[1:0], busy};
    assign busy_ran = busy_d[2];
endmodule
