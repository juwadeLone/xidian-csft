`timescale 1ns/1ps

module axi_burst_reader_256 #(
    parameter integer ADDR_W = 36,
    parameter integer ID_W = 4,
    parameter [ID_W-1:0] AXI_ID = 0
) (
    input  wire                     clk,
    input  wire                     rst,
    input  wire                     clear,
    input  wire                     start,
    input  wire [ADDR_W-1:0]        base_addr,
    input  wire [31:0]              total_beats,
    output wire                     busy,
    output reg                      done,
    output reg                      error,
    output wire [255:0]             m_data,
    output wire                     m_valid,
    input  wire                     m_ready,
    output wire [ID_W-1:0]          m_axi_arid,
    output wire [ADDR_W-1:0]        m_axi_araddr,
    output wire [7:0]               m_axi_arlen,
    output wire [2:0]               m_axi_arsize,
    output wire [1:0]               m_axi_arburst,
    output wire                     m_axi_arvalid,
    input  wire                     m_axi_arready,
    input  wire [ID_W-1:0]          m_axi_rid,
    input  wire [255:0]             m_axi_rdata,
    input  wire [1:0]               m_axi_rresp,
    input  wire                     m_axi_rlast,
    input  wire                     m_axi_rvalid,
    output wire                     m_axi_rready
);
    localparam [1:0] ST_IDLE=2'd0, ST_READ=2'd1, ST_RECOVER=2'd2,
                     ST_POISONED=2'd3;
    localparam [7:0] DRAIN_TIMEOUT_CYCLES=8'd64;
    localparam integer END_W = (ADDR_W + 1 > 38) ? ADDR_W + 1 : 38;
    localparam [END_W-1:0] ADDRESS_SPACE_END =
        ({{(END_W-1){1'b0}},1'b1} << ADDR_W);
    reg [1:0] state;
    reg [ADDR_W-1:0] addr_q;
    reg [31:0] total_beats_q;
    reg [31:0] beats_received;
    reg [4:0] burst_beat;
    reg ar_pending;
    reg [31:0] pad_remaining;
    reg drain_needed;
    reg [7:0] drain_timer;
    reg drain_timed_out;

    wire [END_W-1:0] base_addr_ext =
        {{(END_W-ADDR_W){1'b0}},base_addr};
    wire [END_W-1:0] transfer_bytes_ext =
        ({{(END_W-32){1'b0}},total_beats} << 5);
    wire [END_W-1:0] exclusive_end_ext =
        base_addr_ext + transfer_bytes_ext;
    wire address_range_error = exclusive_end_ext > ADDRESS_SPACE_END;
    wire recover_pad_done_now = (pad_remaining == 0) ||
                                ((pad_remaining == 1) && m_ready);
    wire recover_drain_last_now = (state == ST_RECOVER) && drain_needed &&
                                  m_axi_rvalid && m_axi_rlast;
    wire recover_drain_timeout_now = (state == ST_RECOVER) && drain_needed &&
                                     !recover_drain_last_now &&
                                     (drain_timer <= 1);
    wire recover_drain_done_now = !drain_needed || recover_drain_last_now ||
                                  recover_drain_timeout_now;

    assign busy = state != ST_IDLE;
    assign m_axi_arid = AXI_ID;
    assign m_axi_araddr = addr_q;
    assign m_axi_arlen = 8'd15;
    assign m_axi_arsize = 3'd5;
    assign m_axi_arburst = 2'b01;
    assign m_axi_arvalid = state == ST_READ && ar_pending;
    // A malformed RLAST terminates the real read stream.  Remaining beats are
    // synthesized as zero so an already-started downstream writer can finish
    // its accepted burst(s) instead of being abandoned mid-transaction.
    assign m_data = state == ST_RECOVER ? 256'd0 : m_axi_rdata;
    assign m_valid = (state == ST_READ && !ar_pending && m_axi_rvalid) ||
                     (state == ST_RECOVER && pad_remaining != 0);
    // For a missing/late RLAST, late data is discarded for a bounded interval;
    // it is never forwarded after the declared 16th beat.
    assign m_axi_rready = (state == ST_READ && !ar_pending && m_ready) ||
                          (state == ST_RECOVER && drain_needed);

    always @(posedge clk) begin
        if (rst || clear) begin
            state <= ST_IDLE;
            addr_q <= '0;
            total_beats_q <= '0;
            beats_received <= '0;
            burst_beat <= '0;
            ar_pending <= 1'b0;
            pad_remaining <= '0;
            drain_needed <= 1'b0;
            drain_timer <= '0;
            drain_timed_out <= 1'b0;
            done <= 1'b0;
            error <= 1'b0;
        end else begin
            done <= 1'b0;
            if (state == ST_IDLE) begin
                if (start) begin
                    addr_q <= base_addr;
                    total_beats_q <= total_beats;
                    beats_received <= 0;
                    burst_beat <= 0;
                    pad_remaining <= 0;
                    drain_needed <= 1'b0;
                    drain_timer <= 0;
                    drain_timed_out <= 1'b0;
                    if ((total_beats == 0) || (total_beats[3:0] != 0) ||
                        (base_addr[8:0] != 0) || address_range_error) begin
                        state <= ST_IDLE;
                        ar_pending <= 1'b0;
                        done <= 1'b1;
                        error <= 1'b1;
                    end else begin
                        state <= ST_READ;
                        ar_pending <= 1'b1;
                        error <= 1'b0;
                    end
                end
            end else if (state == ST_READ) begin
                if (ar_pending && m_axi_arready)
                    ar_pending <= 1'b0;
                if (!ar_pending && m_axi_rvalid && m_ready) begin
                    beats_received <= beats_received + 1'b1;
                    if (m_axi_rresp != 2'b00 || m_axi_rid != AXI_ID)
                        error <= 1'b1;
                    if ((burst_beat == 15) != m_axi_rlast) begin
                        error <= 1'b1;
                        ar_pending <= 1'b0;
                        pad_remaining <= total_beats_q - (beats_received + 1'b1);
                        // Early RLAST has already closed the malformed burst.
                        // A missing RLAST is drained, if it arrives late, for a
                        // bounded time before system-level recovery takes over.
                        drain_needed <= (burst_beat == 15) && !m_axi_rlast;
                        drain_timer <= ((burst_beat == 15) && !m_axi_rlast) ?
                                       DRAIN_TIMEOUT_CYCLES : 8'd0;
                        drain_timed_out <= 1'b0;
                        state <= ST_RECOVER;
                    end else if (burst_beat == 15) begin
                        burst_beat <= 0;
                        if (beats_received + 1'b1 == total_beats_q) begin
                            state <= ST_IDLE;
                            done <= 1'b1;
                        end else begin
                            addr_q <= addr_q + {{(ADDR_W-10){1'b0}},10'd512};
                            ar_pending <= 1'b1;
                        end
                    end else begin
                        burst_beat <= burst_beat + 1'b1;
                    end
                end
                if (start)
                    error <= 1'b1;
            end else if (state == ST_RECOVER) begin
                // Consume synthetic output independently from any late AXI R
                // beats being discarded.  Completion requires both activities
                // to reach a finite terminal condition.
                if ((pad_remaining != 0) && m_ready)
                    pad_remaining <= pad_remaining - 1'b1;

                if (drain_needed) begin
                    if (recover_drain_last_now) begin
                        drain_needed <= 1'b0;
                    end else if (recover_drain_timeout_now) begin
                        drain_needed <= 1'b0;
                        drain_timer <= 0;
                        drain_timed_out <= 1'b1;
                    end else begin
                        drain_timer <= drain_timer - 1'b1;
                    end
                end

                if (recover_pad_done_now && recover_drain_done_now) begin
                    // EARLY RLAST, or LATE RLAST observed before timeout, has
                    // closed the malformed AXI transaction and is reusable.
                    // A true timeout is not reusable: done pulses once so the
                    // parent can observe failure, while busy remains asserted
                    // in ST_POISONED until a coordinated system clear/reset.
                    state <= (drain_timed_out || recover_drain_timeout_now) ?
                             ST_POISONED : ST_IDLE;
                    done <= 1'b1;
                    error <= 1'b1;
                end
                if (start)
                    error <= 1'b1;
            end else begin
                // NEEDS_CLEAR.  No AR or synthetic data can be issued here.
                // Local clear only releases this RTL state; the system recovery
                // controller must pair it with AXI/MIG quiesce or channel reset.
                ar_pending <= 1'b0;
                drain_needed <= 1'b0;
                error <= 1'b1;
            end
        end
    end
endmodule
