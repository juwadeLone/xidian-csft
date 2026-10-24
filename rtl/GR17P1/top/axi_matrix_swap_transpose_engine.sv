`timescale 1ns/1ps

// In-place square-matrix transpose by tiled pair exchange.
//
// The matrix is a row-major grid of 64x64 tiles.  Only the upper triangle of
// the tile grid is visited.  An off-diagonal pair (r,c)/(c,r) with r<c is
// read completely (tile A into bank A, tile B into bank B); each tile is
// locally transposed by the bank read-address permutation and written back
// into the exchanged position.  A diagonal tile (r==c) is read, locally
// transposed (its own upper/lower triangle exchange), and written back to
// its own position.  A single DDR region therefore holds the complete
// transposed matrix when done pulses: no second frame buffer is required.
//
// All data movement uses the shared production blocks: 16-beat 512-byte AXI
// bursts, 256<->64 bit width conversion.  The two 4096x64b banks match the
// BRAM footprint of the streaming tile_transpose_pingpong.

module axi_matrix_swap_transpose_engine #(
    parameter integer ADDR_W = 36,
    parameter integer ID_W = 4,
    parameter integer MATRIX_LOG2 = 12,
    parameter integer TILE_LOG2 = 6
) (
    input  wire                     clk,
    input  wire                     rst,
    input  wire                     clear,
    input  wire                     start,
    input  wire [ADDR_W-1:0]        base_addr,
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
    localparam integer TILE_SAMPLES = 1 << TILE_INDEX_W;
    localparam [31:0] ROW_BEATS = 32'd16;

    localparam [3:0] ST_IDLE         = 4'd0;
    localparam [3:0] ST_READ_START   = 4'd1;
    localparam [3:0] ST_READ_WAIT    = 4'd2;
    localparam [3:0] ST_WRITE_START  = 4'd3;
    localparam [3:0] ST_WRITE_WAIT   = 4'd4;
    localparam [3:0] ST_ERROR_HOLD   = 4'd5;

    // One extra bit makes the range check wrap-free.  One cpx32 sample
    // occupies eight bytes, hence N*N*8 bytes for the frame.
    localparam [ADDR_W:0] MATRIX_BYTES_EXT =
        ({{ADDR_W{1'b0}},1'b1} << MATRIX_BYTES_LOG2);
    localparam [ADDR_W:0] ADDRESS_SPACE_EXT = {1'b1,{ADDR_W{1'b0}}};
    wire [ADDR_W:0] base_begin_ext = {1'b0,base_addr};
    wire [ADDR_W:0] base_end_ext = base_begin_ext + MATRIX_BYTES_EXT;
    wire start_contract_error = (base_addr[8:0] != 0) ||
                                (base_end_ext > ADDRESS_SPACE_EXT);

    reg [3:0] state;
    reg reader_start;
    reg writer_start;
    reg [ADDR_W-1:0] base_addr_q;
    reg [TILE_COORD_W-1:0] pair_row_q;
    reg [TILE_COORD_W-1:0] pair_col_q;
    reg [TILE_LOG2-1:0] read_row_q;
    reg [TILE_LOG2-1:0] write_row_q;
    reg read_tile_q;          // read phase: 0 = tile A (r,c), 1 = tile B (c,r)
    reg write_tile_q;         // write phase: 0 = write back tile A, 1 = tile B

    wire [MATRIX_LOG2-1:0] pair_row_ext =
        {{(MATRIX_LOG2-TILE_COORD_W){1'b0}},pair_row_q};
    wire [MATRIX_LOG2-1:0] pair_col_ext =
        {{(MATRIX_LOG2-TILE_COORD_W){1'b0}},pair_col_q};
    wire [MATRIX_LOG2-1:0] read_row_ext =
        {{(MATRIX_LOG2-TILE_LOG2){1'b0}},read_row_q};
    wire [MATRIX_LOG2-1:0] write_row_ext =
        {{(MATRIX_LOG2-TILE_LOG2){1'b0}},write_row_q};

    // Active source tile is (r,c) while read_tile_q=0 and (c,r) while 1.
    wire [MATRIX_LOG2-1:0] src_tile_row =
        read_tile_q ? pair_col_ext : pair_row_ext;
    wire [MATRIX_LOG2-1:0] src_tile_col =
        read_tile_q ? pair_row_ext : pair_col_ext;
    wire [MATRIX_LOG2-1:0] source_global_row =
        (src_tile_row << TILE_LOG2) | read_row_ext;
    wire [MATRIX_LOG2-1:0] source_global_col = (src_tile_col << TILE_LOG2);
    wire [OFFSET_W-1:0] source_row_offset =
        {source_global_row,source_global_col,3'b000};
    wire [ADDR_W-1:0] reader_row_addr = base_addr_q +
        {{(ADDR_W-OFFSET_W){1'b0}},source_row_offset};

    // The transposed source tile lands at the exchanged tile coordinate:
    // tile (r,c) transposed is written at tile (c,r); row m of the source
    // becomes column m of the written tile.  Writing back tile A uses the
    // exchanged coordinate (c,r); writing back tile B (read from (c,r))
    // uses (r,c).  For a diagonal tile the exchanged coordinate is itself.
    wire [MATRIX_LOG2-1:0] dst_tile_row =
        write_tile_q ? pair_row_ext : pair_col_ext;
    wire [MATRIX_LOG2-1:0] dst_tile_col =
        write_tile_q ? pair_col_ext : pair_row_ext;
    wire [MATRIX_LOG2-1:0] destination_global_row =
        (dst_tile_row << TILE_LOG2) | write_row_ext;
    wire [MATRIX_LOG2-1:0] destination_global_col = (dst_tile_col << TILE_LOG2);
    wire [OFFSET_W-1:0] destination_row_offset =
        {destination_global_row,destination_global_col,3'b000};
    wire [ADDR_W-1:0] writer_row_addr = base_addr_q +
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
    wire [255:0] packed_data;
    wire packed_valid;
    wire packed_ready;

    reg [TILE_INDEX_W-1:0] load_index;
    reg load_run;
    reg load_valid_q;
    wire [63:0] load_data_q;
    wire load_ready;
    wire [63:0] pk_data;
    wire        pk_valid, pk_ready;

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

    stream_64_to_256 u_pack (
        .clk(clk),
        .rst(rst),
        .clear(clear),
        .s_data(pk_data),
        .s_valid(pk_valid),
        .s_ready(pk_ready),
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

    // The tile store is a plain BRAM write port: every incoming 64-bit
    // sample of the exactly-64-row read phase is accepted unconditionally.
    // Overflow is impossible by construction (one burst row per launch).
    assign sample_ready = 1'b1;

    // ---- dual tile store: linear write, permuted read ----
    // Bank A holds tile (r,c) while bank B holds tile (c,r).  A diagonal
    // pair uses bank A only.  The read-side index permutation {col,row}
    // performs the local 64x64 transpose itself.
    //
    // Board synth (CSA_USE_XILINX_IP) uses xpm_memory_sdpram so the 4096x64
    // banks stay in RAMB36.  Icarus keeps an equivalent inferred model.
    wire [63:0] rdata_a;
    wire [63:0] rdata_b;
    reg [TILE_INDEX_W-1:0] store_addr;
    reg tile_stored;
    wire store_we = sample_valid && sample_ready;
    wire store_we_a = store_we && !read_tile_q;
    wire store_we_b = store_we && read_tile_q;
    // The first reader launch of a tile restarts the linear store; it always
    // precedes the first data beat by at least the AR handshake latency.
    wire store_reset = reader_start && (read_row_q == {TILE_LOG2{1'b0}});

    // Load side: streams the active bank in permuted (transposed) order into
    // the packer.  Armed once per tile by the first writer launch, then pure
    // ready/valid flow; the writer consumes exactly TILE_SAMPLES samples.
    wire load_advance = !load_valid_q || load_ready;
    wire [TILE_INDEX_W-1:0] load_permuted =
        {load_index[TILE_LOG2-1:0],load_index[TILE_INDEX_W-1:TILE_LOG2]};
    wire load_start = writer_start && (write_row_q == {TILE_LOG2{1'b0}});

    assign load_data_q = write_tile_q ? rdata_b : rdata_a;
    assign pk_data  = load_data_q;
    assign pk_valid = load_valid_q;
    assign load_ready = pk_ready;

`ifdef CSA_USE_XILINX_IP
    xpm_memory_sdpram #(
        .MEMORY_SIZE(TILE_SAMPLES * 64),
        .MEMORY_PRIMITIVE("block"),
        .CLOCKING_MODE("common_clock"),
        .MEMORY_INIT_FILE("none"),
        .USE_MEM_INIT(0),
        .WRITE_DATA_WIDTH_A(64),
        .BYTE_WRITE_WIDTH_A(64),
        .ADDR_WIDTH_A(TILE_INDEX_W),
        .READ_DATA_WIDTH_B(64),
        .ADDR_WIDTH_B(TILE_INDEX_W),
        .READ_LATENCY_B(1),
        .WRITE_MODE_B("read_first")
    ) u_tile_bank_a (
        .sleep(1'b0),
        .clka(clk),
        .ena(1'b1),
        .wea(store_we_a),
        .addra(store_addr),
        .dina(sample_data),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .clkb(clk),
        .rstb(1'b0),
        .enb(load_advance),
        .regceb(1'b1),
        .addrb(load_permuted),
        .doutb(rdata_a),
        .sbiterrb(),
        .dbiterrb()
    );
    xpm_memory_sdpram #(
        .MEMORY_SIZE(TILE_SAMPLES * 64),
        .MEMORY_PRIMITIVE("block"),
        .CLOCKING_MODE("common_clock"),
        .MEMORY_INIT_FILE("none"),
        .USE_MEM_INIT(0),
        .WRITE_DATA_WIDTH_A(64),
        .BYTE_WRITE_WIDTH_A(64),
        .ADDR_WIDTH_A(TILE_INDEX_W),
        .READ_DATA_WIDTH_B(64),
        .ADDR_WIDTH_B(TILE_INDEX_W),
        .READ_LATENCY_B(1),
        .WRITE_MODE_B("read_first")
    ) u_tile_bank_b (
        .sleep(1'b0),
        .clka(clk),
        .ena(1'b1),
        .wea(store_we_b),
        .addra(store_addr),
        .dina(sample_data),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .clkb(clk),
        .rstb(1'b0),
        .enb(load_advance),
        .regceb(1'b1),
        .addrb(load_permuted),
        .doutb(rdata_b),
        .sbiterrb(),
        .dbiterrb()
    );
`else
    (* ram_style = "block" *) reg [63:0] tile_bank_a [0:TILE_SAMPLES-1];
    (* ram_style = "block" *) reg [63:0] tile_bank_b [0:TILE_SAMPLES-1];
    reg [63:0] rdata_a_q;
    reg [63:0] rdata_b_q;
    assign rdata_a = rdata_a_q;
    assign rdata_b = rdata_b_q;

    always @(posedge clk) begin
        if (store_we_a)
            tile_bank_a[store_addr] <= sample_data;
        if (load_advance)
            rdata_a_q <= tile_bank_a[load_permuted];
    end

    always @(posedge clk) begin
        if (store_we_b)
            tile_bank_b[store_addr] <= sample_data;
        if (load_advance)
            rdata_b_q <= tile_bank_b[load_permuted];
    end
`endif

    always @(posedge clk) begin
        if (rst || clear || store_reset) begin
            store_addr <= '0;
            tile_stored <= 1'b0;
        end else if (store_we) begin
            if (store_addr == {TILE_INDEX_W{1'b1}})
                tile_stored <= 1'b1;
            store_addr <= store_addr + 1'b1;
        end
    end

    always @(posedge clk) begin
        if (rst || clear) begin
            load_run <= 1'b0;
            load_index <= '0;
            load_valid_q <= 1'b0;
        end else if (load_start) begin
            load_run <= 1'b1;
            load_index <= '0;
            load_valid_q <= 1'b0;
        end else if (load_advance) begin
            if (load_run) begin
                load_valid_q <= 1'b1;
                if (load_index == {TILE_INDEX_W{1'b1}})
                    load_run <= 1'b0;
                else
                    load_index <= load_index + 1'b1;
            end else begin
                load_valid_q <= 1'b0;
            end
        end
    end

    assign busy = ((state != ST_IDLE) && (state != ST_ERROR_HOLD)) ||
                  reader_busy || writer_busy;

    always @(posedge clk) begin
        if (rst || clear) begin
            state <= ST_IDLE;
            reader_start <= 1'b0;
            writer_start <= 1'b0;
            base_addr_q <= '0;
            pair_row_q <= '0;
            pair_col_q <= '0;
            read_row_q <= '0;
            write_row_q <= '0;
            read_tile_q <= 1'b0;
            write_tile_q <= 1'b0;
            done <= 1'b0;
            error <= 1'b0;
        end else begin
            done <= 1'b0;
            reader_start <= 1'b0;
            writer_start <= 1'b0;

            if (start && busy)
                error <= 1'b1;

            if (state != ST_IDLE &&
                (reader_error || writer_error))
                error <= 1'b1;

            case (state)
                ST_IDLE: begin
                    if (start) begin
                        if (start_contract_error) begin
                            done <= 1'b1;
                            error <= 1'b1;
                        end else begin
                            base_addr_q <= base_addr;
                            pair_row_q <= '0;
                            pair_col_q <= '0;
                            read_row_q <= '0;
                            write_row_q <= '0;
                            read_tile_q <= 1'b0;
                            write_tile_q <= 1'b0;
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
                    if (reader_error) begin
                        error <= 1'b1;
                        done <= 1'b1;
                        state <= ST_ERROR_HOLD;
                    end else if (read_row_q != {TILE_LOG2{1'b1}}) begin
                        if (reader_done) begin
                            read_row_q <= read_row_q + 1'b1;
                            state <= ST_READ_START;
                        end
                    end else if (tile_stored) begin
                        // One complete tile is stored in its bank.  For an
                        // off-diagonal pair, tile B must be read BEFORE any
                        // write-back: writing the transposed tile A lands on
                        // tile B's original position and would destroy its
                        // not-yet-buffered data (the classic in-place swap
                        // hazard).  Both banks are therefore filled first.
                        write_row_q <= '0;
                        if (!read_tile_q && (pair_row_q != pair_col_q)) begin
                            read_tile_q <= 1'b1;
                            read_row_q <= '0;
                            state <= ST_READ_START;
                        end else begin
                            write_tile_q <= 1'b0;
                            state <= ST_WRITE_START;
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
                            error <= 1'b1;
                            done <= 1'b1;
                            state <= ST_ERROR_HOLD;
                        end else if (write_row_q == {TILE_LOG2{1'b1}}) begin
                            read_row_q <= '0;
                            write_row_q <= '0;
                            if (!write_tile_q &&
                                (pair_row_q != pair_col_q)) begin
                                // Second write-back of an off-diagonal pair;
                                // its source tile is already safe in bank B.
                                write_tile_q <= 1'b1;
                                state <= ST_WRITE_START;
                            end else begin
                                // Diagonal tile or second write-back done.
                                read_tile_q <= 1'b0;
                                write_tile_q <= 1'b0;
                                if (pair_row_q == TILE_GRID-1) begin
                                    // last pair is the (GRID-1,GRID-1)
                                    // diagonal of the upper triangle
                                    pair_row_q <= '0;
                                    pair_col_q <= '0;
                                    state <= ST_IDLE;
                                    done <= 1'b1;
                                end else if (pair_col_q == TILE_GRID-1) begin
                                    pair_row_q <= pair_row_q + 1'b1;
                                    pair_col_q <= pair_row_q + 1'b1;
                                    state <= ST_READ_START;
                                end else begin
                                    pair_col_q <= pair_col_q + 1'b1;
                                    state <= ST_READ_START;
                                end
                            end
                        end else begin
                            write_row_q <= write_row_q + 1'b1;
                            state <= ST_WRITE_START;
                        end
                    end
                end

                ST_ERROR_HOLD: begin
                    // Fail closed until clear.  Already-written tiles cannot
                    // be rolled back; the system recovery controller owns
                    // the retry of the complete in-place transpose pass.
                    error <= 1'b1;
                end

                default: begin
                    state <= ST_IDLE;
                    error <= 1'b1;
                end
            endcase
        end
    end
endmodule
