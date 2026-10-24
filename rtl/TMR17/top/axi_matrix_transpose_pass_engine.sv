`timescale 1ns/1ps

// Row-major square-matrix transpose pass.
//
// The production contract is a 64x64 tile.  A source tile at (tile_row,
// tile_col) is read as 64 independent, contiguous 512-byte AXI bursts.  The
// locally transposed tile is then written as 64 independent bursts at the
// globally exchanged tile coordinate (tile_col,tile_row).  Consequently this
// module transposes the complete row-major matrix; it does not treat a linear
// list of tiles as though that list were a matrix.
//
// Source and destination hold one complete N x N cpx32 matrix each and must be
// disjoint.  Every memory transaction is made through the AXI five channels.
module axi_matrix_transpose_pass_engine #(
    parameter integer ADDR_W = 36,
    parameter integer ID_W = 4,
    parameter integer MATRIX_LOG2 = 12,
    parameter integer TILE_LOG2 = 6
) (
    input  wire                     clk,
    input  wire                     rst,
    input  wire                     clear,
    input  wire                     start,
    input  wire [ADDR_W-1:0]        source_addr,
    input  wire [ADDR_W-1:0]        destination_addr,
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
    localparam integer MATRIX_BYTES_LOG2 = 2*MATRIX_LOG2 + 3;
    localparam integer OFFSET_W = MATRIX_BYTES_LOG2;
    localparam integer GRID_LOG2 = MATRIX_LOG2 - TILE_LOG2;
    localparam integer TILE_COORD_W = (GRID_LOG2 > 0) ? GRID_LOG2 : 1;
    localparam integer TILE_GRID = 1 << GRID_LOG2;
    localparam integer TILE_INDEX_W = 2*TILE_LOG2;
    localparam [31:0] ROW_BEATS = 32'd16;

    localparam [2:0] ST_IDLE        = 3'd0;
    localparam [2:0] ST_READ_START  = 3'd1;
    localparam [2:0] ST_READ_WAIT   = 3'd2;
    localparam [2:0] ST_WRITE_START = 3'd3;
    localparam [2:0] ST_WRITE_WAIT  = 3'd4;
    localparam [2:0] ST_ERROR_HOLD  = 3'd5;

    // One extra bit makes range wrap and overlap checks explicit.  One cpx32
    // sample occupies eight bytes, hence N*N*8 bytes per matrix.
    localparam [ADDR_W:0] MATRIX_BYTES_EXT =
        ({{ADDR_W{1'b0}},1'b1} << MATRIX_BYTES_LOG2);
    localparam [ADDR_W:0] ADDRESS_SPACE_EXT =
        {1'b1,{ADDR_W{1'b0}}};
    wire [ADDR_W:0] source_begin_ext = {1'b0,source_addr};
    wire [ADDR_W:0] destination_begin_ext = {1'b0,destination_addr};
    wire [ADDR_W:0] source_end_ext = source_begin_ext + MATRIX_BYTES_EXT;
    wire [ADDR_W:0] destination_end_ext =
        destination_begin_ext + MATRIX_BYTES_EXT;
    wire start_ranges_overlap =
        (source_begin_ext < destination_end_ext) &&
        (destination_begin_ext < source_end_ext);
    wire start_contract_error =
        (source_addr[8:0] != 0) ||
        (destination_addr[8:0] != 0) ||
        (source_end_ext > ADDRESS_SPACE_EXT) ||
        (destination_end_ext > ADDRESS_SPACE_EXT) ||
        start_ranges_overlap;

    reg [2:0] state;
    reg reader_start;
    reg writer_start;
    reg [ADDR_W-1:0] source_addr_q;
    reg [ADDR_W-1:0] destination_addr_q;
    reg [TILE_COORD_W-1:0] tile_row_q;
    reg [TILE_COORD_W-1:0] tile_col_q;
    reg [TILE_LOG2-1:0] read_row_q;
    reg [TILE_LOG2-1:0] write_row_q;

    wire [MATRIX_LOG2-1:0] tile_row_ext =
        {{(MATRIX_LOG2-TILE_COORD_W){1'b0}},tile_row_q};
    wire [MATRIX_LOG2-1:0] tile_col_ext =
        {{(MATRIX_LOG2-TILE_COORD_W){1'b0}},tile_col_q};
    wire [MATRIX_LOG2-1:0] read_row_ext =
        {{(MATRIX_LOG2-TILE_LOG2){1'b0}},read_row_q};
    wire [MATRIX_LOG2-1:0] write_row_ext =
        {{(MATRIX_LOG2-TILE_LOG2){1'b0}},write_row_q};

    // Source tile (r,c): global row r*T+local_row, column base c*T.
    wire [MATRIX_LOG2-1:0] source_global_row =
        (tile_row_ext << TILE_LOG2) | read_row_ext;
    wire [MATRIX_LOG2-1:0] source_global_col =
        (tile_col_ext << TILE_LOG2);

    // Destination tile coordinate is exchanged to (c,r).  Each output row is
    // therefore global row c*T+local_row at column base r*T.
    wire [MATRIX_LOG2-1:0] destination_global_row =
        (tile_col_ext << TILE_LOG2) | write_row_ext;
    wire [MATRIX_LOG2-1:0] destination_global_col =
        (tile_row_ext << TILE_LOG2);

    wire [OFFSET_W-1:0] source_row_offset =
        {source_global_row,source_global_col,3'b000};
    wire [OFFSET_W-1:0] destination_row_offset =
        {destination_global_row,destination_global_col,3'b000};
    wire [ADDR_W-1:0] reader_row_addr = source_addr_q +
        {{(ADDR_W-OFFSET_W){1'b0}},source_row_offset};
    wire [ADDR_W-1:0] writer_row_addr = destination_addr_q +
        {{(ADDR_W-OFFSET_W){1'b0}},destination_row_offset};

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
        if (TILE_LOG2 != 6)
            $fatal(1,"Production TILE_LOG2 is fixed at 6 (64x64)");
        if (MATRIX_LOG2 < TILE_LOG2)
            $fatal(1,"MATRIX_LOG2 must be at least TILE_LOG2");
        if (MATRIX_BYTES_LOG2 >= ADDR_W)
            $fatal(1,"ADDR_W cannot represent a complete matrix address range");
        if (ADDR_W < 9)
            $fatal(1,"ADDR_W must support 512-byte alignment");
    end

    axi_burst_reader_256 #(
        .ADDR_W(ADDR_W),
        .ID_W(ID_W),
        .AXI_ID(1)
    ) u_reader (
        .clk(clk),
        .rst(rst),
        .clear(clear),
        .start(reader_start),
        .base_addr(reader_row_addr),
        .total_beats(ROW_BEATS),
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
        .start(writer_start),
        .base_addr(writer_row_addr),
        .total_beats(ROW_BEATS),
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

    // ST_ERROR_HOLD is a quiescent, externally observable terminal state.
    // A clear is required before another pass may start, but busy is low once
    // every already-issued AXI transaction has completed.
    assign busy = ((state != ST_IDLE) && (state != ST_ERROR_HOLD)) ||
                  reader_busy || writer_busy;

    // SOF/EOF are derived solely from accepted 64-bit tile samples.  AXI
    // latency and backpressure therefore cannot move a tile boundary.
    always @(posedge clk) begin
        if (rst || clear) begin
            tile_sample_index <= '0;
        end else if (state == ST_IDLE && start && !start_contract_error) begin
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
            state <= ST_IDLE;
            reader_start <= 1'b0;
            writer_start <= 1'b0;
            source_addr_q <= '0;
            destination_addr_q <= '0;
            tile_row_q <= '0;
            tile_col_q <= '0;
            read_row_q <= '0;
            write_row_q <= '0;
            done <= 1'b0;
            error <= 1'b0;
        end else begin
            done <= 1'b0;
            reader_start <= 1'b0;
            writer_start <= 1'b0;

            // A repeated launch cannot alter any latched address or counter.
            if (start && busy)
                error <= 1'b1;

            if (state != ST_IDLE &&
                (reader_error || writer_error || transpose_protocol_error ||
                 transpose_overflow_error))
                error <= 1'b1;

            case (state)
                ST_IDLE: begin
                    if (start) begin
                        if (start_contract_error) begin
                            done <= 1'b1;
                            error <= 1'b1;
                        end else begin
                            source_addr_q <= source_addr;
                            destination_addr_q <= destination_addr;
                            tile_row_q <= '0;
                            tile_col_q <= '0;
                            read_row_q <= '0;
                            write_row_q <= '0;
                            error <= 1'b0;
                            state <= ST_READ_START;
                        end
                    end
                end

                ST_READ_START: begin
                    reader_start <= 1'b1;
                    state <= ST_READ_WAIT;
                end

                ST_READ_WAIT: begin
                    if (reader_done) begin
                        if (reader_error) begin
                            // Never expose a tile assembled from a failed row
                            // to the writer, and never issue another row/tile.
                            error <= 1'b1;
                            done <= 1'b1;
                            state <= ST_ERROR_HOLD;
                        end else if (read_row_q == {TILE_LOG2{1'b1}}) begin
                            write_row_q <= '0;
                            state <= ST_WRITE_START;
                        end else begin
                            read_row_q <= read_row_q + 1'b1;
                            state <= ST_READ_START;
                        end
                    end
                end

                ST_WRITE_START: begin
                    writer_start <= 1'b1;
                    state <= ST_WRITE_WAIT;
                end

                ST_WRITE_WAIT: begin
                    if (writer_done) begin
                        if (writer_error) begin
                            // Prior rows/tiles may already be in DDR and cannot
                            // be rolled back here.  The system-level recovery
                            // controller must invalidate checkpoint-valid
                            // metadata before retrying the complete pass.
                            error <= 1'b1;
                            done <= 1'b1;
                            state <= ST_ERROR_HOLD;
                        end else if (write_row_q == {TILE_LOG2{1'b1}}) begin
                            read_row_q <= '0;
                            write_row_q <= '0;
                            if ((tile_row_q == TILE_GRID-1) &&
                                (tile_col_q == TILE_GRID-1)) begin
                                state <= ST_IDLE;
                                done <= 1'b1;
                            end else begin
                                if (tile_col_q == TILE_GRID-1) begin
                                    tile_col_q <= '0;
                                    tile_row_q <= tile_row_q + 1'b1;
                                end else begin
                                    tile_col_q <= tile_col_q + 1'b1;
                                end
                                state <= ST_READ_START;
                            end
                        end else begin
                            write_row_q <= write_row_q + 1'b1;
                            state <= ST_WRITE_START;
                        end
                    end
                end

                ST_ERROR_HOLD: begin
                    // Fail closed until clear.  In particular, ignore start
                    // even though busy is low and issue no further AXI work.
                    error <= 1'b1;
                end

                default: begin
                    state <= ST_IDLE;
                    error <= 1'b1;
                end
            endcase
        end
    end

    // Marker consumption is structural: the tile transposer and packer share
    // the same ready/valid boundary.  Keep this reduction visible to lint.
    wire unused_transposed_markers = transposed_sof ^ transposed_eof;

endmodule
