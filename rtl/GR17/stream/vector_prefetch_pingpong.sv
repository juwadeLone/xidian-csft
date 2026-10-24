`timescale 1ns/1ps

// Two complete-vector banks decouple AXI jitter from an SDF core.  Input may
// pause arbitrarily.  Once a full vector is launched, output valid is asserted
// for exactly N consecutive clocks; the core-facing side has no ready signal
// by design.
module vector_prefetch_pingpong #(
    parameter integer LOGN = 12,
    parameter integer DATA_W = 64,
    // rpr_noroll: 1 = launch each vector with the index MSB inverted, i.e.
    // feed the core the ifftshift of the stored vector (replaces the (-1)^n
    // un-roll after the IFFT).  Sample 0 of the launched vector is stored
    // word RD_XOR, captured at its write like word 0 is without the shift.
    parameter integer IN_SHIFT = 0
) (
    input  wire                     clk,
    input  wire                     rst,
    input  wire                     clear,
    input  wire [DATA_W-1:0]        s_data,
    input  wire                     s_valid,
    output wire                     s_ready,
    input  wire                     s_sof,
    input  wire                     s_eof,
    input  wire                     launch_enable,
    output wire [DATA_W-1:0]        m_data,
    output reg                      m_valid,
    output reg                      m_sof,
    output reg                      m_eof,
    output wire [1:0]               bank_full,
    output wire                     occupied,
    output reg                      protocol_error,
    output reg                      overflow_error
);
    localparam integer N = 1 << LOGN;
    localparam [LOGN-1:0] RD_XOR = (IN_SHIFT != 0) ? (1 << (LOGN-1)) : 0;

`ifndef CSA_USE_XILINX_IP
    (* ram_style="block" *) reg [DATA_W-1:0] bank0 [0:N-1];
    (* ram_style="block" *) reg [DATA_W-1:0] bank1 [0:N-1];
`endif
    reg wr_bank;
    reg [LOGN-1:0] wr_index;
    reg [1:0] full_q;
    reg next_read_bank;
    reg read_bank;
    reg read_active;
    reg [LOGN-1:0] read_index;
    reg [DATA_W-1:0] m_data_r;
    wire input_fire = s_valid && s_ready;
    wire input_completes_bank = input_fire && (wr_index == N-1);
    wire launching = !read_active && launch_enable &&
                     (full_q[next_read_bank] ||
                      (input_completes_bank && wr_bank == next_read_bank));
    wire continue_now = read_active && (read_index == N-1) && launch_enable &&
                        (full_q[~read_bank] ||
                         (input_completes_bank && wr_bank == ~read_bank));

    assign bank_full = full_q;
    assign s_ready = !full_q[wr_bank];
    assign occupied = (wr_index != '0) || (full_q != 0) ||
                      read_active || m_valid;

`ifdef CSA_USE_XILINX_IP
    // Sample 0 is captured on the write of index 0 so launch stays
    // same-cycle.  Later samples come from XPM READ_LATENCY_B=1, with
    // the address issued one cycle early.
    reg [DATA_W-1:0] word0_0;
    reg [DATA_W-1:0] word0_1;
    wire wr_en0 = input_fire && !wr_bank;
    wire wr_en1 = input_fire && wr_bank;
    // Address = read_index, with no special cases.
    //
    // READ_LATENCY_B is 1, so the address presented on cycle k produces data on
    // k+1, and sample 0 is not read at all -- it comes from the word0 bypass.
    // The cycle on which read_index is 1 therefore carries m_data sample 0 and
    // must present address 1, so that sample 1 arrives on the next cycle: the
    // address to present is read_index itself.
    //
    // The previous version added one, and forced address 1 on the launch and
    // continuation cycles.  Both were wrong.  The +1 handed the core index k+1
    // where it wanted k, and the forced 1 replaced the old vector's last
    // address (N-1) with the new vector's, dropping the final sample of every
    // vector.  The launch and continuation cycles need no special address:
    // read_index is 0 on both, and both are don't-cares because the sample that
    // lands next comes from word0.
    //
    // read_bank is the right bank throughout, including on the continuation
    // cycle -- it still holds the old bank there and flips at the end of it,
    // which is exactly when the old vector's last address is still needed.
    wire [LOGN-1:0] xpm_addr = read_active ? (read_index ^ RD_XOR) : {LOGN{1'b0}};
    wire xpm_en   = read_active;
    wire xpm_bank = read_bank;
    wire [DATA_W-1:0] rdata0;
    wire [DATA_W-1:0] rdata1;
    reg use_word0;
    reg word0_bank;
    reg xpm_data_bank;
    always @(posedge clk) begin
        if (rst || clear) begin
            word0_0 <= {DATA_W{1'b0}};
            word0_1 <= {DATA_W{1'b0}};
            use_word0 <= 1'b0;
            word0_bank <= 1'b0;
            xpm_data_bank <= 1'b0;
        end else begin
            if (input_fire && (wr_index == RD_XOR)) begin
                if (!wr_bank)
                    word0_0 <= s_data;
                else
                    word0_1 <= s_data;
            end
            if (launching) begin
                use_word0 <= 1'b1;
                word0_bank <= next_read_bank;
            end else if (read_active && (read_index == {LOGN{1'b0}})) begin
                use_word0 <= 1'b1;
                word0_bank <= read_bank;
            end else if (read_active || launching)
                use_word0 <= 1'b0;
            if (xpm_en)
                xpm_data_bank <= xpm_bank;
        end
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
        .addra(wr_index), .dina(s_data),
        .injectsbiterra(1'b0), .injectdbiterra(1'b0),
        .clkb(clk), .rstb(1'b0), .enb(xpm_en && !xpm_bank),
        .regceb(1'b1), .addrb(xpm_addr), .doutb(rdata0),
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
        .addra(wr_index), .dina(s_data),
        .injectsbiterra(1'b0), .injectdbiterra(1'b0),
        .clkb(clk), .rstb(1'b0), .enb(xpm_en && xpm_bank),
        .regceb(1'b1), .addrb(xpm_addr), .doutb(rdata1),
        .sbiterrb(), .dbiterrb()
    );
    assign m_data = use_word0 ?
                    (word0_bank ? word0_1 : word0_0) :
                    (xpm_data_bank ? rdata1 : rdata0);
`else
    assign m_data = m_data_r;
`endif

    always @(posedge clk) begin
        if (rst || clear) begin
            wr_bank <= 1'b0;
            wr_index <= '0;
            full_q <= 2'b00;
            next_read_bank <= 1'b0;
            read_bank <= 1'b0;
            read_active <= 1'b0;
            read_index <= '0;
            m_data_r <= '0;
            m_valid <= 1'b0;
            m_sof <= 1'b0;
            m_eof <= 1'b0;
            protocol_error <= 1'b0;
            overflow_error <= 1'b0;
        end else begin
            m_valid <= 1'b0;
            m_sof <= 1'b0;
            m_eof <= 1'b0;

            // valid may remain asserted while ready is low.  Since writes are
            // handshake-qualified, legal ready/valid backpressure cannot
            // overflow either bank.
            if (input_fire) begin
                if ((wr_index == 0 && !s_sof) ||
                    (wr_index != 0 && s_sof) ||
                    (wr_index == N-1 && !s_eof) ||
                    (wr_index != N-1 && s_eof))
                    protocol_error <= 1'b1;
`ifndef CSA_USE_XILINX_IP
                if (!wr_bank)
                    bank0[wr_index] <= s_data;
                else
                    bank1[wr_index] <= s_data;
`endif
                if (wr_index == N-1) begin
                    full_q[wr_bank] <= 1'b1;
                    wr_bank <= ~wr_bank;
                    wr_index <= '0;
                end else begin
                    wr_index <= wr_index + 1'b1;
                end
            end

            if (!read_active && launch_enable &&
                (full_q[next_read_bank] ||
                 (input_completes_bank && wr_bank == next_read_bank))) begin
                read_bank <= next_read_bank;
`ifndef CSA_USE_XILINX_IP
                if (!next_read_bank)
                    m_data_r <= bank0[RD_XOR];
                else
                    m_data_r <= bank1[RD_XOR];
`endif
                m_valid <= 1'b1;
                m_sof <= 1'b1;
                m_eof <= N == 1;
                if (N == 1) begin
                    read_active <= 1'b0;
                    full_q[next_read_bank] <= 1'b0;
                    next_read_bank <= ~next_read_bank;
                    read_index <= '0;
                end else begin
                    read_active <= 1'b1;
                    read_index <= 1;
                end
            end

            if (read_active) begin
`ifndef CSA_USE_XILINX_IP
                if (!read_bank)
                    m_data_r <= bank0[read_index ^ RD_XOR];
                else
                    m_data_r <= bank1[read_index ^ RD_XOR];
`endif
                m_valid <= 1'b1;
                m_sof <= read_index == 0;
                m_eof <= read_index == N-1;
                if (read_index == N-1) begin
                    full_q[read_bank] <= 1'b0;
                    next_read_bank <= ~next_read_bank;
                    read_index <= '0;
                    // If the other complete bank has credit, make it the
                    // active bank immediately.  The next clock then emits its
                    // sample zero while m_valid stays continuously asserted.
                    if (launch_enable &&
                        (full_q[~read_bank] ||
                         (input_completes_bank && wr_bank == ~read_bank))) begin
                        read_active <= 1'b1;
                        read_bank <= ~read_bank;
                    end else begin
                        read_active <= 1'b0;
                    end
                end else begin
                    read_index <= read_index + 1'b1;
                end
            end
        end
    end
endmodule
