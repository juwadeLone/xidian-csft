`timescale 1ns/1ps

module bitrev_pingpong #(
    parameter integer LOGN = 12,
    parameter integer DATA_W = 64,
    // rpr_noroll: 1 = read the natural-order bank with the index MSB
    // inverted, i.e. emit the spectrum fftshift-ed (replaces the (-1)^n
    // centring roll in front of the FFT)
    parameter integer OUT_SHIFT = 0
) (
    input  wire                     clk,
    input  wire                     rst,
    input  wire                     clear,
    input  wire [DATA_W-1:0]        s_data,
    input  wire                     s_valid,
    output wire                     s_ready,
    input  wire                     s_sof,
    input  wire                     s_eof,
    output wire [DATA_W-1:0]        m_data,
    output wire                     m_valid,
    input  wire                     m_ready,
    output wire                     m_sof,
    output wire                     m_eof,
    output wire                     occupied,
    output reg                      protocol_error,
    output reg                      overflow_error
);
    localparam integer N = 1 << LOGN;
`ifndef CSA_USE_XILINX_IP
    (* ram_style = "block" *) reg [DATA_W-1:0] bank0 [0:N-1];
    (* ram_style = "block" *) reg [DATA_W-1:0] bank1 [0:N-1];
`endif
    reg wr_bank;
    reg [LOGN-1:0] wr_index;
    reg [1:0] bank_ready;
    reg next_read_bank;
    reg read_active;
    reg read_bank;
    reg [LOGN-1:0] read_index;
    reg [DATA_W-1:0] read_data_q;
    reg read_valid_q, read_sof_q, read_eof_q;
    wire input_fire = s_valid && s_ready;
    wire input_completes_bank = input_fire && (wr_index == N-1);
    wire output_can_advance = !read_valid_q || m_ready;
    // In reserved-output mode at the lane level, the next transformed vector
    // reaches this writer on the same clock that the reader fetches the last
    // word from the bank being recycled.  The read address is N-1 while the
    // new frame writes bit_reverse(0), so a dual-port RAM can accept both.
    // m_ready remains part of the condition: a stalled final read never frees
    // the bank optimistically.
    localparam [LOGN-1:0] RD_XOR = (OUT_SHIFT != 0) ? (1 << (LOGN-1)) : 0;
    wire [LOGN-1:0] read_addr = read_index ^ RD_XOR;
    wire write_bank_released_now = read_active && output_can_advance &&
                                   (read_index == N-1) &&
                                   (read_bank == wr_bank);

    function [LOGN-1:0] reverse_bits;
        input [LOGN-1:0] value;
        integer k;
        begin
            for (k = 0; k < LOGN; k = k + 1)
                reverse_bits[k] = value[LOGN-1-k];
        end
    endfunction

    assign s_ready = !bank_ready[wr_bank] || write_bank_released_now;
`ifdef CSA_USE_XILINX_IP
    wire wr_en0 = input_fire && (wr_bank == 1'b0);
    wire wr_en1 = input_fire && (wr_bank == 1'b1);
    wire rd_en  = output_can_advance && read_active;
    wire [LOGN-1:0] wr_addr = reverse_bits(wr_index);
    wire [DATA_W-1:0] rdata0;
    wire [DATA_W-1:0] rdata1;
    reg data_bank;
    always @(posedge clk) begin
        if (rst || clear)
            data_bank <= 1'b0;
        else if (rd_en)
            data_bank <= read_bank;
    end
    xpm_memory_sdpram #(
        .MEMORY_SIZE(N * DATA_W),
        .MEMORY_PRIMITIVE("block"),
        .CLOCKING_MODE("common_clock"),
        .MEMORY_INIT_FILE("none"),
        .USE_MEM_INIT(0),
        .WRITE_DATA_WIDTH_A(DATA_W),
        .BYTE_WRITE_WIDTH_A(DATA_W),
        .ADDR_WIDTH_A(LOGN),
        .READ_DATA_WIDTH_B(DATA_W),
        .ADDR_WIDTH_B(LOGN),
        .READ_LATENCY_B(1),
        .WRITE_MODE_B("read_first")
    ) u_bank0 (
        .sleep(1'b0), .clka(clk), .ena(1'b1), .wea(wr_en0),
        .addra(wr_addr), .dina(s_data),
        .injectsbiterra(1'b0), .injectdbiterra(1'b0),
        .clkb(clk), .rstb(1'b0), .enb(rd_en && (read_bank == 1'b0)),
        .regceb(1'b1), .addrb(read_addr), .doutb(rdata0),
        .sbiterrb(), .dbiterrb()
    );
    xpm_memory_sdpram #(
        .MEMORY_SIZE(N * DATA_W),
        .MEMORY_PRIMITIVE("block"),
        .CLOCKING_MODE("common_clock"),
        .MEMORY_INIT_FILE("none"),
        .USE_MEM_INIT(0),
        .WRITE_DATA_WIDTH_A(DATA_W),
        .BYTE_WRITE_WIDTH_A(DATA_W),
        .ADDR_WIDTH_A(LOGN),
        .READ_DATA_WIDTH_B(DATA_W),
        .ADDR_WIDTH_B(LOGN),
        .READ_LATENCY_B(1),
        .WRITE_MODE_B("read_first")
    ) u_bank1 (
        .sleep(1'b0), .clka(clk), .ena(1'b1), .wea(wr_en1),
        .addra(wr_addr), .dina(s_data),
        .injectsbiterra(1'b0), .injectdbiterra(1'b0),
        .clkb(clk), .rstb(1'b0), .enb(rd_en && (read_bank == 1'b1)),
        .regceb(1'b1), .addrb(read_addr), .doutb(rdata1),
        .sbiterrb(), .dbiterrb()
    );
    assign m_data = data_bank ? rdata1 : rdata0;
`else
    assign m_data = read_data_q;
`endif
    assign m_valid = read_valid_q;
    assign m_sof = read_sof_q;
    assign m_eof = read_eof_q;
    assign occupied = (wr_index != '0) || (bank_ready != 0) ||
                      read_active || read_valid_q;
    always @(posedge clk) begin
        if (rst || clear) begin
            wr_bank <= 1'b0;
            wr_index <= '0;
            bank_ready <= 2'b00;
            next_read_bank <= 1'b0;
            read_active <= 1'b0;
            read_bank <= 1'b0;
            read_index <= '0;
            read_data_q <= '0;
            read_valid_q <= 1'b0;
            read_sof_q <= 1'b0;
            read_eof_q <= 1'b0;
            protocol_error <= 1'b0;
            overflow_error <= 1'b0;
        end else begin
            // Backpressure is legal on this ready/valid interface.  An
            // unstallable-source contract violation is detected by the lane,
            // not by treating a held valid as a buffer overflow here.
            if (input_fire) begin
                if ((wr_index == 0 && !s_sof) || (wr_index != 0 && s_sof) ||
                    (wr_index == N-1 && !s_eof) || (wr_index != N-1 && s_eof))
                    protocol_error <= 1'b1;
`ifndef CSA_USE_XILINX_IP
                if (wr_bank == 1'b0)
                    bank0[reverse_bits(wr_index)] <= s_data;
                else
                    bank1[reverse_bits(wr_index)] <= s_data;
`endif
                if (wr_index == N-1) begin
                    bank_ready[wr_bank] <= 1'b1;
                    wr_bank <= ~wr_bank;
                    wr_index <= '0;
                end else begin
                    wr_index <= wr_index + 1'b1;
                end
            end

            if (!read_active && bank_ready[next_read_bank]) begin
                read_active <= 1'b1;
                read_bank <= next_read_bank;
                read_index <= '0;
            end

            if (output_can_advance) begin
                if (read_active) begin
`ifndef CSA_USE_XILINX_IP
                    if (read_bank == 1'b0)
                        read_data_q <= bank0[read_addr];
                    else
                        read_data_q <= bank1[read_addr];
`endif
                    read_valid_q <= 1'b1;
                    read_sof_q <= read_index == 0;
                    read_eof_q <= read_index == N-1;
                    if (read_index == N-1) begin
                        bank_ready[read_bank] <= 1'b0;
                        next_read_bank <= ~next_read_bank;
                        read_index <= '0;
                        if (bank_ready[~read_bank] ||
                            (input_completes_bank && wr_bank == ~read_bank)) begin
                            read_active <= 1'b1;
                            read_bank <= ~read_bank;
                        end else begin
                            read_active <= 1'b0;
                        end
                    end else begin
                        read_index <= read_index + 1'b1;
                    end
                end else begin
                    read_valid_q <= 1'b0;
                    read_sof_q <= 1'b0;
                    read_eof_q <= 1'b0;
                end
            end
        end
    end
endmodule
