`timescale 1ns/1ps

// Reusable DDR-to-DDR tile-transpose pass.  Every memory access is performed
// through the AXI reader/writer; the transpose operates only on their streams.
module axi_tile_transpose_pass_engine #(
    parameter integer ADDR_W = 36,
    parameter integer ID_W = 4,
    parameter integer TILE_LOG2 = 6
) (
    input  wire                     clk,
    input  wire                     rst,
    input  wire                     clear,
    input  wire                     start,
    input  wire [ADDR_W-1:0]        source_addr,
    input  wire [ADDR_W-1:0]        destination_addr,
    input  wire [31:0]              tile_count,
    output wire                     busy,
    output reg                      done,
    output reg                      error,

    output wire [ID_W-1:0]          m_axi_awid,
    output wire [ADDR_W-1:0]        m_axi_awaddr,
    output wire [7:0]               m_axi_awlen,
    output wire [2:0]               m_axi_awsize,
    output wire [1:0]               m_axi_awburst,
    output wire                     m_axi_awvalid,
    input  wire                     m_axi_awready,
    output wire [255:0]             m_axi_wdata,
    output wire [31:0]              m_axi_wstrb,
    output wire                     m_axi_wlast,
    output wire                     m_axi_wvalid,
    input  wire                     m_axi_wready,
    input  wire [ID_W-1:0]          m_axi_bid,
    input  wire [1:0]               m_axi_bresp,
    input  wire                     m_axi_bvalid,
    output wire                     m_axi_bready,

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
    localparam integer TILE_INDEX_W = 2*TILE_LOG2;

    // One beat contains four 64-bit samples.  Use a wide launch expression so
    // an unrepresentable 32-bit child count is rejected instead of truncated.
    wire [63:0] start_total_beats_wide =
        {32'd0,tile_count} << (TILE_INDEX_W-2);
    wire start_contract_error =
        (tile_count == 0) ||
        (start_total_beats_wide[63:32] != 0) ||
        (start_total_beats_wide[3:0] != 0) ||
        (source_addr[8:0] != 0) ||
        (destination_addr[8:0] != 0);

    reg active;
    reg child_start;
    reg reader_seen;
    reg writer_seen;
    reg [ADDR_W-1:0] source_addr_q;
    reg [ADDR_W-1:0] destination_addr_q;
    reg [31:0] tile_count_q;
    reg [31:0] total_beats_q;

    wire reader_busy;
    wire reader_done;
    wire reader_error;
    wire writer_busy;
    wire writer_done;
    wire writer_error;

    wire [255:0] read_data;
    wire read_valid;
    wire read_ready;
    wire [63:0] sample_data;
    wire sample_valid;
    wire sample_ready;

    reg [TILE_INDEX_W-1:0] tile_sample_index;
    wire tile_sof = tile_sample_index == {TILE_INDEX_W{1'b0}};
    wire tile_eof = tile_sample_index == {TILE_INDEX_W{1'b1}};

    wire [63:0] transposed_data;
    wire transposed_valid;
    wire transposed_ready;
    wire transposed_sof;
    wire transposed_eof;
    wire transpose_protocol_error;
    wire transpose_overflow_error;

    wire [255:0] packed_data;
    wire packed_valid;
    wire packed_ready;

    initial begin
        if (TILE_LOG2 < 1)
            $fatal(1,"TILE_LOG2 must be at least 1");
        if (ADDR_W < 9)
            $fatal(1,"ADDR_W must be at least 9 for 512-byte alignment");
    end

    axi_burst_reader_256 #(
        .ADDR_W(ADDR_W),
        .ID_W(ID_W),
        .AXI_ID(1)
    ) u_reader (
        .clk(clk),
        .rst(rst),
        .clear(clear),
        .start(child_start),
        .base_addr(source_addr_q),
        .total_beats(total_beats_q),
        .busy(reader_busy),
        .done(reader_done),
        .error(reader_error),
        .m_data(read_data),
        .m_valid(read_valid),
        .m_ready(read_ready),
        .m_axi_arid(m_axi_arid),
        .m_axi_araddr(m_axi_araddr),
        .m_axi_arlen(m_axi_arlen),
        .m_axi_arsize(m_axi_arsize),
        .m_axi_arburst(m_axi_arburst),
        .m_axi_arvalid(m_axi_arvalid),
        .m_axi_arready(m_axi_arready),
        .m_axi_rid(m_axi_rid),
        .m_axi_rdata(m_axi_rdata),
        .m_axi_rresp(m_axi_rresp),
        .m_axi_rlast(m_axi_rlast),
        .m_axi_rvalid(m_axi_rvalid),
        .m_axi_rready(m_axi_rready)
    );

    stream_256_to_64 u_unpack (
        .clk(clk),
        .rst(rst),
        .clear(clear),
        .s_data(read_data),
        .s_valid(read_valid),
        .s_ready(read_ready),
        .m_data(sample_data),
        .m_valid(sample_valid),
        .m_ready(sample_ready)
    );

    tile_transpose_pingpong #(
        .TILE_LOG2(TILE_LOG2),
        .DATA_W(64)
    ) u_transpose (
        .clk(clk),
        .rst(rst),
        .clear(clear),
        .s_data(sample_data),
        .s_valid(sample_valid),
        .s_ready(sample_ready),
        .s_sof(tile_sof),
        .s_eof(tile_eof),
        .m_data(transposed_data),
        .m_valid(transposed_valid),
        .m_ready(transposed_ready),
        .m_sof(transposed_sof),
        .m_eof(transposed_eof),
        .protocol_error(transpose_protocol_error),
        .overflow_error(transpose_overflow_error)
    );

    stream_64_to_256 u_pack (
        .clk(clk),
        .rst(rst),
        .clear(clear),
        .s_data(transposed_data),
        .s_valid(transposed_valid),
        .s_ready(transposed_ready),
        .m_data(packed_data),
        .m_valid(packed_valid),
        .m_ready(packed_ready)
    );

    axi_burst_writer_256 #(
        .ADDR_W(ADDR_W),
        .ID_W(ID_W),
        .AXI_ID(2)
    ) u_writer (
        .clk(clk),
        .rst(rst),
        .clear(clear),
        .start(child_start),
        .base_addr(destination_addr_q),
        .total_beats(total_beats_q),
        .busy(writer_busy),
        .done(writer_done),
        .error(writer_error),
        .s_data(packed_data),
        .s_valid(packed_valid),
        .s_ready(packed_ready),
        .m_axi_awid(m_axi_awid),
        .m_axi_awaddr(m_axi_awaddr),
        .m_axi_awlen(m_axi_awlen),
        .m_axi_awsize(m_axi_awsize),
        .m_axi_awburst(m_axi_awburst),
        .m_axi_awvalid(m_axi_awvalid),
        .m_axi_awready(m_axi_awready),
        .m_axi_wdata(m_axi_wdata),
        .m_axi_wstrb(m_axi_wstrb),
        .m_axi_wlast(m_axi_wlast),
        .m_axi_wvalid(m_axi_wvalid),
        .m_axi_wready(m_axi_wready),
        .m_axi_bid(m_axi_bid),
        .m_axi_bresp(m_axi_bresp),
        .m_axi_bvalid(m_axi_bvalid),
        .m_axi_bready(m_axi_bready)
    );

    assign busy = active || reader_busy || writer_busy;

    // SOF/EOF track accepted 64-bit input samples, never wall-clock cycles.
    always @(posedge clk) begin
        if (rst || clear) begin
            tile_sample_index <= '0;
        end else if (child_start) begin
            tile_sample_index <= '0;
        end else if (sample_valid && sample_ready) begin
            if (tile_eof)
                tile_sample_index <= '0;
            else
                tile_sample_index <= tile_sample_index + 1'b1;
        end
    end

    always @(posedge clk) begin
        if (rst || clear) begin
            active <= 1'b0;
            child_start <= 1'b0;
            reader_seen <= 1'b0;
            writer_seen <= 1'b0;
            source_addr_q <= '0;
            destination_addr_q <= '0;
            tile_count_q <= '0;
            total_beats_q <= '0;
            done <= 1'b0;
            error <= 1'b0;
        end else begin
            done <= 1'b0;
            child_start <= 1'b0;

            if (start) begin
                if (busy) begin
                    // Sticky for the current transaction; latched launch
                    // parameters and all child state remain untouched.
                    error <= 1'b1;
                end else if (start_contract_error) begin
                    active <= 1'b0;
                    reader_seen <= 1'b0;
                    writer_seen <= 1'b0;
                    done <= 1'b1;
                    error <= 1'b1;
                end else begin
                    active <= 1'b1;
                    child_start <= 1'b1;
                    reader_seen <= 1'b0;
                    writer_seen <= 1'b0;
                    source_addr_q <= source_addr;
                    destination_addr_q <= destination_addr;
                    tile_count_q <= tile_count;
                    total_beats_q <= start_total_beats_wide[31:0];
                    error <= 1'b0;
                end
            end

            if (reader_done)
                reader_seen <= 1'b1;
            if (writer_done)
                writer_seen <= 1'b1;

            if (active && !child_start &&
                (reader_error || writer_error || transpose_protocol_error ||
                 transpose_overflow_error))
                error <= 1'b1;

            if (active &&
                (reader_seen || reader_done) &&
                (writer_seen || writer_done)) begin
                active <= 1'b0;
                done <= 1'b1;
            end
        end
    end

    // These markers are consumed by the transpose-to-pack stream boundary;
    // naming them here also makes their preservation visible in simulation.
    wire unused_transposed_markers = transposed_sof ^ transposed_eof;
    wire unused_latched_tile_count = ^tile_count_q;

endmodule
