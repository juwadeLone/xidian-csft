`timescale 1ns/1ps

module sdf_fft_core #(
    parameter integer LOGN = 11,
    parameter integer DATA_W = 11,
    parameter integer INVERSE = 0,
    parameter [11:0] SCALE_MASK = 12'h000,
    parameter [3:0] CORE_ID = 4'd0
) (
    input  wire                         clk,
    input  wire                         rst,
    input  wire                         clear,
    input  wire                         s_valid,
    output wire                         s_ready,
    input  wire                         s_sof,
    input  wire                         s_eof,
    input  wire signed [DATA_W-1:0]     s_re,
    input  wire signed [DATA_W-1:0]     s_im,
    output wire                         m_valid,
    output wire                         m_sof,
    output wire                         m_eof,
    output wire signed [DATA_W-1:0]     m_re,
    output wire signed [DATA_W-1:0]     m_im,
    output wire                         busy,
    output reg                          protocol_error,
    output wire                         overflow_sticky,
    output wire [LOGN-1:0]              stage_overflow,
    output wire [LOGN*12-1:0]           debug_delay_addr,
    output wire [LOGN*12-1:0]           debug_twiddle_addr,
    output wire [LOGN-1:0]              debug_stage_valid,
    output wire [3:0]                   core_id
);

    localparam integer N = 1 << LOGN;
    localparam integer COUNT_W = LOGN;

    reg busy_q;
    reg flush_mode;
    reg await_next;
    reg need_clear;
    reg [COUNT_W-1:0] sample_count;
    reg [15:0] frames_inflight;

    assign busy = busy_q;
    assign core_id = CORE_ID;
    assign s_ready = !flush_mode && !need_clear;
    wire accept = s_valid && s_ready;
    wire engine_ce = busy_q || accept;
    wire stage_clear = clear || need_clear;

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

    wire signed [DATA_W-1:0] re_pipe [0:LOGN];
    wire signed [DATA_W-1:0] im_pipe [0:LOGN];
    wire [LOGN:0] valid_pipe;
    wire [LOGN:0] sof_pipe;
    wire [LOGN:0] eof_pipe;
    wire [LOGN-1:0] sample_overflow_pipe;
    wire [LOGN-1:0] sticky_overflow_pipe;
    reg  [LOGN-1:0] stage_overflow_sticky_q;

    always @(posedge clk) begin
        if (rst || clear)
            stage_overflow_sticky_q <= '0;
        else
            stage_overflow_sticky_q <= stage_overflow_sticky_q |
                                       sample_overflow_pipe;
    end

    assign re_pipe[0] = accept ? s_re : '0;
    assign im_pipe[0] = accept ? s_im : '0;
    assign valid_pipe[0] = accept;
    assign sof_pipe[0] = accept && s_sof;
    assign eof_pipe[0] = accept && s_eof;

    genvar g;
    generate
        for (g = 0; g < LOGN; g = g + 1) begin : g_sdf_stage
            sdf_stage #(
                .LOGN(LOGN),
                .STAGE(g),
                .MAX_LOGN(11),
                .DATA_W(DATA_W),
                .INVERSE(INVERSE),
                .SCALE(SCALE_MASK[g])
            ) u_stage (
                .clk(clk),
                .rst(rst),
                .clear(stage_clear),
                .ce(engine_ce),
                .i_valid(valid_pipe[g]),
                .i_sof(sof_pipe[g]),
                .i_eof(eof_pipe[g]),
                .i_re(re_pipe[g]),
                .i_im(im_pipe[g]),
                .o_valid(valid_pipe[g+1]),
                .o_sof(sof_pipe[g+1]),
                .o_eof(eof_pipe[g+1]),
                .o_re(re_pipe[g+1]),
                .o_im(im_pipe[g+1]),
                .o_sample_overflow(sample_overflow_pipe[g]),
                .o_overflow_re(),
                .o_overflow_im(),
                .o_overflow_sticky(sticky_overflow_pipe[g]),
                .o_debug_delay_addr(debug_delay_addr[g*12 +: 12]),
                .o_debug_twiddle_addr(debug_twiddle_addr[g*12 +: 12])
            );
        end
    endgenerate

    assign m_valid = valid_pipe[LOGN];
    assign m_sof = sof_pipe[LOGN];
    assign m_eof = eof_pipe[LOGN];
    assign m_re = re_pipe[LOGN];
    assign m_im = im_pipe[LOGN];
    assign stage_overflow = stage_overflow_sticky_q;
    assign overflow_sticky = |stage_overflow_sticky_q;
    assign debug_stage_valid = valid_pipe[LOGN-1:0];

endmodule
