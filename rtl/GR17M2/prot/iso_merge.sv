`timescale 1ns/1ps

// ub16 support primitives, ported from baseline/experiments/rpr_base
// (sdf_iso.sv) with the width-generic bodies unchanged.
//
// fft_iso_ctrl is ONE COPY of the SDF core control (busy / flush / clear /
// sample counter / frames in flight / protocol check).  Three copies run
// under dont_touch in sdf_fft_core_tk; each copy's accept decision uses the
// majority of the three ready flags, so the copies take the same samples.
//
// iso_merge is the place three control copies (or, in a full-replica build,
// three data domains) meet: a bitwise majority, combinational.

module fft_iso_ctrl #(
    parameter integer LOGN = 11
) (
    input  wire         clk,
    input  wire         rst,
    input  wire         clear,
    input  wire         s_valid,
    input  wire         s_sof,
    input  wire         s_eof,
    input  wire [2:0]   rdy,          // the three copies' ready flags
    input  wire         m_valid,      // this copy's core output tick
    input  wire         m_eof,
    output wire         s_ready_own,
    output wire         ready_maj,    // this copy's majority of the ready flags
    output wire         accept,
    output wire         engine_ce,
    output wire         stage_clear,
    output wire         busy,
    output reg          protocol_error
);
    localparam integer N = 1 << LOGN;
    localparam integer COUNT_W = LOGN;
    reg busy_q;
    reg flush_mode;
    reg await_next;
    reg need_clear;
    reg [COUNT_W-1:0] sample_count;
    reg [15:0] frames_inflight;
    wire rdy_maj = (rdy[0] & rdy[1]) | (rdy[0] & rdy[2]) | (rdy[1] & rdy[2]);
    assign busy = busy_q;
    assign s_ready_own = !flush_mode && !need_clear;
    assign accept = s_valid && rdy_maj;
    assign ready_maj = rdy_maj;
    assign engine_ce = busy_q || accept;
    assign stage_clear = clear || need_clear;

    always @(posedge clk) begin
        if (rst || clear) begin
            busy_q <= 1'b0;
            flush_mode <= 1'b0;
            await_next <= 1'b0;
            need_clear <= 1'b0;
            sample_count <= '0;
            frames_inflight <= '0;
            protocol_error <= 1'b0;
        end else if (need_clear) begin
            need_clear <= 1'b0;
            flush_mode <= 1'b0;
            await_next <= 1'b0;
            sample_count <= '0;
            frames_inflight <= '0;
        end else begin
            if (accept) begin
                busy_q <= 1'b1;
                if ((sample_count == 0 && !s_sof) ||
                    (sample_count != 0 && s_sof) ||
                    (sample_count == N-1 && !s_eof) ||
                    (sample_count != N-1 && s_eof))
                    protocol_error <= 1'b1;

                if (sample_count == N-1) begin
                    sample_count <= '0;
                    frames_inflight <= frames_inflight + 1'b1;
                    await_next <= 1'b1;
                end else begin
                    sample_count <= sample_count + 1'b1;
                    if (await_next)
                        await_next <= 1'b0;
                end
            end else if (busy_q && sample_count != 0) begin
                protocol_error <= 1'b1;
            end

            if (await_next && !accept) begin
                flush_mode <= 1'b1;
                await_next <= 1'b0;
            end

            if (m_valid && m_eof) begin
                if (frames_inflight != 0)
                    frames_inflight <= frames_inflight - 1'b1;
                if (flush_mode && frames_inflight == 1) begin
                    busy_q <= 1'b0;
                    flush_mode <= 1'b0;
                    need_clear <= 1'b1;
                end
            end
        end
    end
endmodule

// ---------------------------------------------------------------------------
module iso_merge #(
    parameter integer W = 67
) (
    input  wire [W-1:0] a,
    input  wire [W-1:0] b,
    input  wire [W-1:0] c,
    output wire [W-1:0] o
);
    assign o = (a & b) | (a & c) | (b & c);
endmodule
