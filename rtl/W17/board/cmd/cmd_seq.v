`timescale 1ns / 1ps
`include "udp_cmd_hdr.vh"

// UART command dispatcher: ARM_WR / TRANSPOSE / RD_BACK, exclusive mux select.
module cmd_seq #(
    parameter integer ADDR_W   = 31,
    parameter integer SIDE_LOG2 = 11,
    // ub16 (port of rpr_base): a frame run (CS_PARAM with flags[7]) that has
    // not finished 2^CS_RUN_TMO_LOG2 clocks after it was started is answered
    // with a NAK whose ack_flags[0] is 1 ("run time-out"), and the sequencer
    // goes back to idle, instead of waiting without end and never answering.
    // 0 = no limit.  At 125 MHz, 31 gives 17.2 s (a 2048 x 2048 frame takes
    // about a second).
    parameter integer CS_RUN_TMO_LOG2 = 31
) (
    input                      clk,
    input                      rst_n,
    input                      calib_done,

    input                      cmd_valid,
    output                     cmd_ready,
    input      [7:0]           cmd,
    input      [7:0]           flags,
    input      [15:0]          seq,
    input      [31:0]          addr,
    input      [31:0]          length,
    input                      hdr_ok,
    input                      cmd_is_wr,
    input                      cmd_is_rd,
    input                      cmd_is_xpose,

    output reg                 ack_start,
    output reg [7:0]           ack_cmd,
    output     [7:0]           ack_flags,
    output     [15:0]          ack_seq,
    output     [31:0]          ack_addr,
    output     [31:0]          ack_length,
    input                      ack_busy,

    output reg [1:0]           mux_sel,
    output     [ADDR_W-1:0]    op_addr,
    output     [31:0]          op_length,
    output reg                 wr_err_sticky,

    output reg                 wr_start,
    input                      wr_busy,
    input                      wr_done,
    input                      wr_error,

    output reg                 swap_start,
    input                      swap_busy,
    input                      swap_done,
    input                      swap_error,

    output reg                 rd_start,
    input                      rd_busy,
    input                      rd_done,
    input                      rd_error,

    output     [1:0]           fft_mode,
    output reg                 fft_start,
    input                      fft_busy,
    input                      fft_done,
    input                      fft_error,
    output     [2:0]           debug_state,

    output reg signed [15:0]   cs_a_hcs,
    output reg signed [15:0]   cs_b_hcs,
    output reg signed [15:0]   cs_c_hcs,
    output reg signed [15:0]   cs_a_hr,
    output reg signed [15:0]   cs_b_hr,
    output reg signed [15:0]   cs_c_hr,
    output reg signed [15:0]   cs_a_ha,
    output reg signed [15:0]   cs_b_ha,
    output reg signed [15:0]   cs_c_ha,
    output reg                 cs_tab_we,
    output reg [1:0]           cs_tab_sel,
    output reg [7:0]           cs_tab_fa,
    output reg signed [15:0]   cs_tab_a,
    output reg signed [15:0]   cs_tab_b,
    output reg signed [15:0]   cs_tab_c,
    output reg                 cs_param_we,
    output     [3:0]           cs_param_idx,
    output     [31:0]          cs_param_w0,
    output     [31:0]          cs_param_w1,
    output reg                 cs_build_start,
    input                      cs_build_busy,
    input                      cs_build_done,
    // Every other wait state has one; this one did not, so a build that failed
    // left the sequencer in S_WAIT_B forever.  csa_cs_app's ST_ERROR drives
    // both busy and done low, so there was nothing else to notice it by.
    input                      cs_build_error,
    // Application clear (2026-10-13).  CS_CLEAR, accepted only while the
    // application is not busy, holds cs_clear high for 32 clocks -- long
    // enough to cross into the application's clock as a level -- and is
    // answered with CS_CLEAR_ACK.  The application takes the rising edge:
    // its run control returns to idle (also out of the error state, which
    // until now only a reset left) and the chain and the result check are
    // cleared.  While the application is busy the command is answered NAK
    // and nothing is cleared: a chain that still has memory transfers
    // under way is not cut off.
    output                     cs_clear,

    output                     sem_busy,
    output reg                 sem_start,
    input                      sem_done,
    input                      sem_err,
    input      [7:0]           sem_resp_len,

    // ---- seed-table row write / read-back (host-built tables) ----
    input      [127:0]         rx_payload,
    output reg                 cs_row_we,
    output reg [1:0]           cs_row_sel,
    output reg [1:0]           cs_row_tgt,   // protection_cmp: flags[5:4]
    output reg [SIDE_LOG2-1:0] cs_row_idx,
    output reg [95:0]          cs_row_data,
    output reg [1:0]           cs_rb_sel,
    output reg [SIDE_LOG2-1:0] cs_rb_idx,
    output reg                 cs_rb_dbg
);

localparam S_IDLE     = 3'd0;
localparam S_ACK      = 3'd1;
localparam S_WAIT_WR  = 3'd2;
localparam S_WAIT_X   = 3'd3;
localparam S_WAIT_RD  = 3'd4;
localparam S_WAIT_F   = 3'd5;
localparam S_WAIT_B   = 3'd6;
localparam S_WAIT_SEM = 3'd7;

reg [2:0]  state;
reg [7:0]  cmd_q;
reg [7:0]  flags_q;
reg [15:0] seq_q;
reg [31:0] addr_q;
reg [31:0] len_q;
reg        pending_ack;
reg        wr_done_seen;
reg [32:0] run_tmo_ctr;
reg        run_tmo_q;        // the last answer is a run time-out
reg [5:0]  clr_cnt;

assign cmd_ready   = (state == S_IDLE) && calib_done;
assign debug_state = state;
assign ack_flags   = {7'd0, run_tmo_q};
assign ack_seq     = seq_q;
assign ack_addr    = addr_q;
assign ack_length  = len_q;
assign op_addr     = addr_q[ADDR_W-1:0];
assign op_length   = len_q;
assign fft_mode     = flags_q[1:0];
assign cs_param_idx = flags[3:0];
assign cs_param_w0  = addr;
assign cs_param_w1  = length;
assign sem_busy     = (state == S_WAIT_SEM);
assign cs_clear     = (clr_cnt != 6'd0);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state       <= S_IDLE;
        cmd_q       <= 8'd0;
        flags_q     <= 8'd0;
        seq_q       <= 16'd0;
        addr_q      <= 32'd0;
        len_q       <= 32'd0;
        mux_sel     <= 2'd0;
        wr_start    <= 1'b0;
        swap_start  <= 1'b0;
        rd_start    <= 1'b0;
        fft_start   <= 1'b0;
        ack_start   <= 1'b0;
        ack_cmd     <= 8'd0;
        pending_ack <= 1'b0;
        wr_err_sticky <= 1'b0;
        wr_done_seen  <= 1'b0;
        cs_a_hcs      <= 16'sd0;
        cs_b_hcs      <= 16'sd0;
        cs_c_hcs      <= 16'sd0;
        cs_a_hr       <= 16'sd0;
        cs_b_hr       <= 16'sd0;
        cs_c_hr       <= 16'sd0;
        cs_a_ha       <= 16'sd0;
        cs_b_ha       <= 16'sd0;
        cs_c_ha       <= 16'sd0;
        cs_tab_we     <= 1'b0;
        cs_tab_sel    <= 2'd0;
        cs_tab_fa     <= 8'd0;
        cs_tab_a      <= 16'sd0;
        cs_tab_b      <= 16'sd0;
        cs_tab_c      <= 16'sd0;
        cs_param_we   <= 1'b0;
        cs_build_start<= 1'b0;
        sem_start     <= 1'b0;
        cs_row_we     <= 1'b0;
        cs_row_sel    <= 2'd0;
        cs_row_tgt    <= 2'd0;
        cs_row_idx    <= {SIDE_LOG2{1'b0}};
        cs_row_data   <= 96'd0;
        cs_rb_sel     <= 2'd0;
        cs_rb_idx     <= {SIDE_LOG2{1'b0}};
        cs_rb_dbg     <= 1'b0;
        run_tmo_ctr   <= 33'd0;
        run_tmo_q     <= 1'b0;
        clr_cnt       <= 6'd0;
    end else begin
        wr_start   <= 1'b0;
        swap_start <= 1'b0;
        rd_start   <= 1'b0;
        fft_start  <= 1'b0;
        ack_start  <= 1'b0;
        wr_done_seen <= 1'b0;
        cs_tab_we  <= 1'b0;
        cs_param_we <= 1'b0;
        cs_build_start <= 1'b0;
        sem_start      <= 1'b0;
        cs_row_we      <= 1'b0;
        if (clr_cnt != 6'd0)
            clr_cnt <= clr_cnt - 6'd1;

        case (state)
            S_IDLE: begin
                if (cmd_valid && cmd_ready) begin
                    cmd_q   <= cmd;
                    flags_q <= flags;
                    seq_q   <= seq;
                    addr_q <= addr;
                    len_q  <= length;
                    run_tmo_q <= 1'b0;
                    run_tmo_ctr <= 33'd0;
                    if (!hdr_ok) begin
                        ack_cmd     <= `UDP_CMD_NAK;
                        pending_ack <= 1'b1;
                        state       <= S_ACK;
                    end else if (cs_build_busy &&
                                 (cmd_is_wr || cmd_is_rd ||
                                  ((cmd == `UDP_CMD_CS_PARAM) && flags[7]))) begin
                        // The application is busy and owns the memory bus
                        // (the board core gives the bus to it whenever it
                        // is busy).  That happens with the sequencer idle
                        // after a run that was answered with an error or a
                        // time-out while the chain was still working.  A
                        // write or read started now would wait for the bus
                        // without end, and a second run cannot start:
                        // refuse them.
                        ack_cmd     <= `UDP_CMD_NAK;
                        pending_ack <= 1'b1;
                        state       <= S_ACK;
                    end else if (cmd_is_wr) begin
                        mux_sel       <= 2'd0;
                        wr_start      <= 1'b1;
                        wr_err_sticky <= 1'b0;
                        ack_cmd       <= `UDP_CMD_WR_ACK;
                        pending_ack   <= 1'b1;
                        state         <= S_ACK;
                    end else if (cmd_is_xpose) begin
                        mux_sel    <= 2'd1;
                        swap_start <= 1'b1;
                        state      <= S_WAIT_X;
                    end else if (cmd_is_rd) begin
                        mux_sel  <= 2'd2;
                        rd_start <= 1'b1;
                        state    <= S_WAIT_RD;
                    end else if (cmd == `UDP_CMD_FFT2D) begin
                        if (cs_build_busy) begin
                            ack_cmd     <= `UDP_CMD_NAK;
                            pending_ack <= 1'b1;
                            state       <= S_ACK;
                        end else begin
                            mux_sel   <= 2'd1;
                            fft_start <= 1'b1;
                            state     <= S_WAIT_F;
                        end
                    end else if (cmd == `UDP_CMD_CS_ROW) begin
                        // One synchronous cycle writes one 96-bit seed row
                        // into the table RAMs: sel = flags[1:0], row =
                        // addr[10:0], the 12 payload bytes carry the row.
                        cs_row_we   <= 1'b1;
                        cs_row_sel  <= flags[1:0];
                        // protection_cmp write target: 00/11 both,
                        // 01 main table only, 10 rb copy only
                        cs_row_tgt  <= flags[5:4];
                        cs_row_idx  <= addr[SIDE_LOG2-1:0];
                        cs_row_data <= rx_payload[95:0];
                        ack_cmd     <= `UDP_CMD_CS_ROW_ACK;
                        pending_ack <= 1'b1;
                        state       <= S_ACK;
                    end else if (cmd == `UDP_CMD_CS_RB) begin
                        // Read-back.  The address is registered and stays put
                        // until the next command, and the top drives
                        // tx_payload straight from the table RAM output, so
                        // the ACK carries the live row.
                        cs_rb_sel   <= flags[1:0];
                        cs_rb_idx   <= addr[SIDE_LOG2-1:0];
                        cs_rb_dbg   <= flags[3];
                        ack_cmd     <= `UDP_CMD_CS_RB_ACK;
                        pending_ack <= 1'b1;
                        state       <= S_ACK;
                    end else if (cmd == `UDP_CMD_CS_PARAM) begin
                        if (flags[7]) begin
                            // The CS engine sits on mux slot 1, the same as
                            // TRANSPOSE and FFT2D.  This branch never set it,
                            // so the build ran with the mux still pointing at
                            // the UDP write master and its AXI traffic never
                            // reached memory: the board sat in S_WAIT_B with
                            // mux=WR until the command timed out.  The path had
                            // never been exercised -- every board top before
                            // this one left cs_param_we unconnected.
                            mux_sel        <= 2'd1;
                            cs_build_start <= 1'b1;
                            state          <= S_WAIT_B;
                        end else begin
                            cs_param_we <= 1'b1;
                            ack_cmd     <= `UDP_CMD_CS_PARAM_ACK;
                            pending_ack <= 1'b1;
                            state       <= S_ACK;
                        end
                    end else if (cmd == `UDP_CMD_CS_COEFF) begin
                        if (flags[2]) begin
                            cs_tab_we  <= 1'b1;
                            cs_tab_sel <= flags[1:0];
                            cs_tab_fa  <= addr[7:0];
                            cs_tab_a   <= addr[31:16];
                            cs_tab_b   <= length[15:0];
                            cs_tab_c   <= length[31:16];
                        end else if (flags[1:0] == 2'd3) begin
                            cs_a_hcs <= 16'sd0; cs_b_hcs <= 16'sd0; cs_c_hcs <= 16'sd0;
                            cs_a_hr  <= 16'sd0; cs_b_hr  <= 16'sd0; cs_c_hr  <= 16'sd0;
                            cs_a_ha  <= 16'sd0; cs_b_ha  <= 16'sd0; cs_c_ha  <= 16'sd0;
                        end else if (flags[1:0] == 2'd0) begin
                            cs_a_hcs <= addr[15:0];
                            cs_b_hcs <= addr[31:16];
                            cs_c_hcs <= length[15:0];
                        end else if (flags[1:0] == 2'd1) begin
                            cs_a_hr <= addr[15:0];
                            cs_b_hr <= addr[31:16];
                            cs_c_hr <= length[15:0];
                        end else begin
                            cs_a_ha <= addr[15:0];
                            cs_b_ha <= addr[31:16];
                            cs_c_ha <= length[15:0];
                        end
                        ack_cmd     <= `UDP_CMD_CS_COEFF_ACK;
                        pending_ack <= 1'b1;
                        state       <= S_ACK;
                    end else if (cmd == `UDP_CMD_CS_CLEAR) begin
                        if (cs_build_busy) begin
                            ack_cmd     <= `UDP_CMD_NAK;
                        end else begin
                            clr_cnt     <= 6'd32;
                            ack_cmd     <= `UDP_CMD_CS_CLEAR_ACK;
                        end
                        pending_ack <= 1'b1;
                        state       <= S_ACK;
                    end else if (cmd == `UDP_CMD_SEM) begin
                        sem_start <= 1'b1;
                        state     <= S_WAIT_SEM;
                    end else begin
                        ack_cmd     <= `UDP_CMD_NAK;
                        pending_ack <= 1'b1;
                        state       <= S_ACK;
                    end
                end
            end
            S_ACK: begin
                if (pending_ack && !ack_busy) begin
                    ack_start   <= 1'b1;
                    pending_ack <= 1'b0;
                end else if (!pending_ack && !ack_busy) begin
                    if (ack_cmd == `UDP_CMD_WR_ACK)
                        state <= S_WAIT_WR;
                    else
                        state <= S_IDLE;
                end
            end
            S_WAIT_WR: begin
                // WR_ACK went out at arm time; the write itself is verified
                // here: completion with wr_error produces an unsolicited NAK
                // so the host learns the write failed (sticky flag feeds
                // telemetry until the next ARM_WR).
                if (wr_done)
                    wr_done_seen <= 1'b1;
                if (wr_done_seen) begin
                    if (wr_error) begin
                        ack_cmd       <= `UDP_CMD_NAK;
                        pending_ack   <= 1'b1;
                        wr_err_sticky <= 1'b1;
                        state         <= S_ACK;
                    end else begin
                        state <= S_IDLE;
                    end
                end
            end
            S_WAIT_X: begin
                if (swap_done && !swap_busy) begin
                    ack_cmd     <= swap_error ? `UDP_CMD_NAK : `UDP_CMD_XPOSE_ACK;
                    pending_ack <= 1'b1;
                    state       <= S_ACK;
                end
            end
            S_WAIT_RD: begin
                if (rd_done) begin
                    ack_cmd     <= rd_error ? `UDP_CMD_NAK : `UDP_CMD_RD_ACK;
                    pending_ack <= 1'b1;
                    state       <= S_ACK;
                end
            end
            S_WAIT_F: begin
                if (fft_done && !fft_busy) begin
                    ack_cmd     <= fft_error ? `UDP_CMD_NAK : `UDP_CMD_FFT2D_ACK;
                    pending_ack <= 1'b1;
                    state       <= S_ACK;
                end
            end
            S_WAIT_B: begin
                // An error is answered at once.  Since 2026-10-13 the
                // application stays busy in its error state for as long as
                // the chain is still working, so waiting for "not busy"
                // here would delay the answer by the rest of a pass, or up
                // to the run time-out when the chain never comes to rest.
                // The host sees the busy level in the telemetry and in the
                // answers to its next commands (S_IDLE above).
                if (cs_build_error) begin
                    ack_cmd     <= `UDP_CMD_NAK;
                    pending_ack <= 1'b1;
                    state       <= S_ACK;
                end else if (cs_build_done && !cs_build_busy) begin
                    ack_cmd     <= `UDP_CMD_CS_PARAM_ACK;
                    pending_ack <= 1'b1;
                    state       <= S_ACK;
                end else if (CS_RUN_TMO_LOG2 != 0 && run_tmo_ctr[CS_RUN_TMO_LOG2]) begin
                    // run time-out: answer, and take commands again
                    run_tmo_q   <= 1'b1;
                    ack_cmd     <= `UDP_CMD_NAK;
                    pending_ack <= 1'b1;
                    state       <= S_ACK;
                end else begin
                    run_tmo_ctr <= run_tmo_ctr + 33'd1;
                end
            end
            S_WAIT_SEM: begin
                if (sem_done) begin
                    len_q       <= {24'd0, sem_resp_len};
                    ack_cmd     <= `UDP_CMD_SEM_ACK;
                    pending_ack <= 1'b1;
                    state       <= S_ACK;
                end
            end
            default: state <= S_IDLE;
        endcase
    end
end

endmodule
