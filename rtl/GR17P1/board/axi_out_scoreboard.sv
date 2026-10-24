`timescale 1ns/1ps

// Read OUT after the chain finishes and XOR-fold every 64-bit word.
// hash_ok is the on-chip green light; host JTAG can still dump OUT.
module axi_out_scoreboard #(
    parameter integer SIDE_LOG2 = 8,
    parameter integer ADDR_W = 36,
    parameter integer ID_W = 4,
    parameter [ID_W-1:0] AXI_ID = 4'h2,
    parameter [63:0] EXPECTED_FOLD = 64'd0
) (
    input  wire                     clk,
    input  wire                     rst,
    input  wire                     clear,
    input  wire                     start,
    input  wire [ADDR_W-1:0]        base_out,
    output wire                     busy,
    output wire                     done,
    output wire                     error,
    output reg                      hash_ok,
    output reg  [63:0]              fold,

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
    localparam integer SIDE = 1 << SIDE_LOG2;
    localparam integer BEATS = (SIDE * SIDE) / 4;

    wire [255:0] m_data;
    wire m_valid;
    wire reader_done;
    wire [63:0] beat_xor = m_data[63:0] ^ m_data[127:64] ^
                           m_data[191:128] ^ m_data[255:192];

    // Declared before use: SystemVerilog has no implicit nets, so the earlier
    // ordering (assign busy = reader_busy above the declaration) is a hard
    // error -- VRFC 10-3380.  It survived because nothing compiled this file:
    // there is no scoreboard testbench, and the one check that would have
    // caught it, vivado_xvlog_all_rtl, had been skipped with -SkipVivado.
    wire reader_busy;

    assign busy = reader_busy;
    assign done = reader_done;

    axi_burst_reader_256 #(
        .ADDR_W(ADDR_W), .ID_W(ID_W), .AXI_ID(AXI_ID)
    ) u_reader (
        .clk(clk), .rst(rst), .clear(clear), .start(start),
        .base_addr(base_out), .total_beats(BEATS),
        .busy(reader_busy), .done(reader_done), .error(error),
        .m_data(m_data), .m_valid(m_valid), .m_ready(1'b1),
        .m_axi_arid(m_axi_arid), .m_axi_araddr(m_axi_araddr),
        .m_axi_arlen(m_axi_arlen), .m_axi_arsize(m_axi_arsize),
        .m_axi_arburst(m_axi_arburst), .m_axi_arvalid(m_axi_arvalid),
        .m_axi_arready(m_axi_arready), .m_axi_rid(m_axi_rid),
        .m_axi_rdata(m_axi_rdata), .m_axi_rresp(m_axi_rresp),
        .m_axi_rlast(m_axi_rlast), .m_axi_rvalid(m_axi_rvalid),
        .m_axi_rready(m_axi_rready)
    );

    always @(posedge clk) begin
        if (rst || clear || start) begin
            fold <= 64'd0;
            hash_ok <= 1'b0;
        end else begin
            if (m_valid)
                fold <= fold ^ beat_xor;
            if (reader_done)
                hash_ok <= ((m_valid ? (fold ^ beat_xor) : fold) ==
                            EXPECTED_FOLD);
        end
    end
endmodule
