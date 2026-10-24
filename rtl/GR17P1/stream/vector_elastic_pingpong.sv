`timescale 1ns/1ps

// Natural-order, two-bank, complete-vector elastic FIFO.  Input samples may
// contain arbitrary gaps.  Output samples use a conventional ready/valid
// interface and therefore remain stable for an arbitrarily long sink stall.
//
// A vector consumes credit when its SOF sample is accepted and releases that
// credit only when its EOF sample is accepted at the output.  Consequently an
// EOF held in the output register still counts as an in-flight vector and a
// third vector cannot enter merely because its RAM bank is safe to overwrite.
module vector_elastic_pingpong #(
    parameter integer LOGN   = 12,
    parameter integer DATA_W = 64
) (
    input  wire                  clk,
    input  wire                  rst,
    input  wire                  clear,

    input  wire [DATA_W-1:0]     s_data,
    input  wire                  s_valid,
    output wire                  s_ready,
    input  wire                  s_sof,
    input  wire                  s_eof,

    output wire [DATA_W-1:0]     m_data,
    output wire                  m_valid,
    input  wire                  m_ready,
    output wire                  m_sof,
    output wire                  m_eof,

    output wire                  occupied,
    output reg                   protocol_error,
    output reg                   overflow_error
);
    localparam integer N = 1 << LOGN;

    initial begin
        if (LOGN < 1)
            $error("vector_elastic_pingpong requires LOGN >= 1");
    end

`ifndef CSA_USE_XILINX_IP
    (* ram_style = "block" *) reg [DATA_W-1:0] bank0 [0:N-1];
    (* ram_style = "block" *) reg [DATA_W-1:0] bank1 [0:N-1];
`endif

    reg                         wr_bank;
    reg [LOGN-1:0]              wr_index;
    reg [1:0]                   bank_ready;
    reg                         next_read_bank;
    reg                         read_active;
    reg                         read_bank;
    reg [LOGN-1:0]              read_index;

    reg [DATA_W-1:0]            read_data_q;
    reg                         read_valid_q;
    reg                         read_sof_q;
    reg                         read_eof_q;
    reg [1:0]                   vector_count;

    wire output_fire     = read_valid_q && m_ready;
    wire output_eof_fire = output_fire && read_eof_q;
    wire bank_available  = !bank_ready[wr_bank];
    wire vector_credit   = (wr_index != {LOGN{1'b0}}) ||
                           (vector_count < 2) || output_eof_fire;
    wire input_fire      = s_valid && s_ready;
    wire input_sof_fire  = input_fire && (wr_index == {LOGN{1'b0}});
    wire input_completes_bank = input_fire && (wr_index == N-1);
    wire output_can_advance = !read_valid_q || m_ready;

    assign s_ready = bank_available && vector_credit;
`ifdef CSA_USE_XILINX_IP
    // XPM READ_LATENCY_B=1 matches the inferred registered read.  Bank
    // select is delayed with the same enable so the last word of a bank
    // is not muxed away on the recycle edge.
    wire wr_en0 = input_fire && !wr_bank;
    wire wr_en1 = input_fire && wr_bank;
    wire rd_en  = output_can_advance && read_active;
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
        .addra(wr_index), .dina(s_data),
        .injectsbiterra(1'b0), .injectdbiterra(1'b0),
        .clkb(clk), .rstb(1'b0), .enb(rd_en && !read_bank),
        .regceb(1'b1), .addrb(read_index), .doutb(rdata0),
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
        .clkb(clk), .rstb(1'b0), .enb(rd_en && read_bank),
        .regceb(1'b1), .addrb(read_index), .doutb(rdata1),
        .sbiterrb(), .dbiterrb()
    );
    assign m_data = data_bank ? rdata1 : rdata0;
`else
    assign m_data  = read_data_q;
`endif
    assign m_valid = read_valid_q;
    assign m_sof   = read_sof_q;
    assign m_eof   = read_eof_q;
    assign occupied = (vector_count != 0) ||
                      (wr_index != {LOGN{1'b0}}) ||
                      (bank_ready != 0) || read_active || read_valid_q;

    always @(posedge clk) begin
        if (rst || clear) begin
            wr_bank        <= 1'b0;
            wr_index       <= {LOGN{1'b0}};
            bank_ready     <= 2'b00;
            next_read_bank <= 1'b0;
            read_active    <= 1'b0;
            read_bank      <= 1'b0;
            read_index     <= {LOGN{1'b0}};
            read_data_q    <= {DATA_W{1'b0}};
            read_valid_q   <= 1'b0;
            read_sof_q     <= 1'b0;
            read_eof_q     <= 1'b0;
            vector_count   <= 2'd0;
            protocol_error <= 1'b0;
            overflow_error <= 1'b0;
        end else begin
            // A held VALID while READY is low is legal and does not indicate
            // overflow.  State changes below are handshake-qualified.
            if (input_fire) begin
                if ((wr_index == 0 && !s_sof) ||
                    (wr_index != 0 && s_sof) ||
                    (wr_index == N-1 && !s_eof) ||
                    (wr_index != N-1 && s_eof))
                    protocol_error <= 1'b1;

                // These conditions are unreachable when the credit and bank
                // equations above are obeyed; keep a sticky hardware witness
                // for a future integration error rather than treating normal
                // backpressure as overflow.
                if (bank_ready[wr_bank] || vector_count > 2)
                    overflow_error <= 1'b1;

`ifndef CSA_USE_XILINX_IP
                if (!wr_bank)
                    bank0[wr_index] <= s_data;
                else
                    bank1[wr_index] <= s_data;
`endif

                if (wr_index == N-1) begin
                    bank_ready[wr_bank] <= 1'b1;
                    wr_bank  <= ~wr_bank;
                    wr_index <= {LOGN{1'b0}};
                end else begin
                    wr_index <= wr_index + 1'b1;
                end
            end

            // Count logical vectors, not merely occupied RAM banks.  The
            // simultaneous case transfers one credit without changing count.
            case ({input_sof_fire, output_eof_fire})
                2'b10: begin
                    if (vector_count == 2)
                        overflow_error <= 1'b1;
                    else
                        vector_count <= vector_count + 1'b1;
                end
                2'b01: begin
                    if (vector_count == 0)
                        protocol_error <= 1'b1;
                    else
                        vector_count <= vector_count - 1'b1;
                end
                default: vector_count <= vector_count;
            endcase

            // A bank is exposed only after all N input samples have arrived.
            if (!read_active && bank_ready[next_read_bank]) begin
                read_active <= 1'b1;
                read_bank   <= next_read_bank;
                read_index  <= {LOGN{1'b0}};
            end

            // The registered output changes only when empty or accepted.
            if (output_can_advance) begin
                if (read_active) begin
`ifndef CSA_USE_XILINX_IP
                    if (!read_bank)
                        read_data_q <= bank0[read_index];
                    else
                        read_data_q <= bank1[read_index];
`endif
                    read_valid_q <= 1'b1;
                    read_sof_q   <= (read_index == 0);
                    read_eof_q   <= (read_index == N-1);

                    if (read_index == N-1) begin
                        // The last word is now in the output register, so the
                        // RAM bank is safe.  Logical vector credit is retained
                        // until the registered EOF is actually accepted.
                        bank_ready[read_bank] <= 1'b0;
                        next_read_bank <= ~next_read_bank;
                        read_index <= {LOGN{1'b0}};
                        if (bank_ready[~read_bank] ||
                            (input_completes_bank &&
                             (wr_bank == ~read_bank))) begin
                            read_active <= 1'b1;
                            read_bank   <= ~read_bank;
                        end else begin
                            read_active <= 1'b0;
                        end
                    end else begin
                        read_index <= read_index + 1'b1;
                    end
                end else begin
                    read_valid_q <= 1'b0;
                    read_sof_q   <= 1'b0;
                    read_eof_q   <= 1'b0;
                end
            end
        end
    end
endmodule
