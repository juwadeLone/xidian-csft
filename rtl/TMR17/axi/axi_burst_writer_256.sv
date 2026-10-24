`timescale 1ns/1ps

module axi_burst_writer_256 #(
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
    input  wire [255:0]             s_data,
    input  wire                     s_valid,
    output wire                     s_ready,
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
    output wire                     m_axi_bready
);
    localparam [1:0] ST_IDLE=0, ST_AW=1, ST_W=2, ST_B=3;
    localparam integer END_W = (ADDR_W + 1 > 38) ? ADDR_W + 1 : 38;
    localparam [END_W-1:0] ADDRESS_SPACE_END =
        ({{(END_W-1){1'b0}},1'b1} << ADDR_W);
    reg [1:0] state;
    reg [ADDR_W-1:0] addr_q;
    reg [31:0] total_beats_q;
    reg [31:0] beats_sent;
    reg [4:0] burst_beat;

    wire [END_W-1:0] base_addr_ext =
        {{(END_W-ADDR_W){1'b0}},base_addr};
    wire [END_W-1:0] transfer_bytes_ext =
        ({{(END_W-32){1'b0}},total_beats} << 5);
    wire [END_W-1:0] exclusive_end_ext =
        base_addr_ext + transfer_bytes_ext;
    wire address_range_error = exclusive_end_ext > ADDRESS_SPACE_END;

    assign busy = state != ST_IDLE;
    assign m_axi_awid = AXI_ID;
    assign m_axi_awaddr = addr_q;
    assign m_axi_awlen = 8'd15;
    assign m_axi_awsize = 3'd5;
    assign m_axi_awburst = 2'b01;
    assign m_axi_awvalid = state == ST_AW;
    assign m_axi_wdata = s_data;
    assign m_axi_wstrb = 32'hffffffff;
    assign m_axi_wlast = burst_beat == 5'd15;
    assign m_axi_wvalid = state == ST_W && s_valid;
    assign s_ready = state == ST_W && m_axi_wready;
    assign m_axi_bready = state == ST_B;

    always @(posedge clk) begin
        if (rst || clear) begin
            state <= ST_IDLE;
            addr_q <= '0;
            total_beats_q <= '0;
            beats_sent <= '0;
            burst_beat <= '0;
            done <= 1'b0;
            error <= 1'b0;
        end else begin
            done <= 1'b0;
            case (state)
                ST_IDLE: if (start) begin
                    addr_q <= base_addr;
                    total_beats_q <= total_beats;
                    beats_sent <= 0;
                    burst_beat <= 0;
                    if ((total_beats == 0) || (total_beats[3:0] != 0) ||
                        (base_addr[8:0] != 0) || address_range_error) begin
                        error <= 1'b1;
                        done <= 1'b1;
                        state <= ST_IDLE;
                    end else begin
                        error <= 1'b0;
                        state <= ST_AW;
                    end
                end
                ST_AW: if (m_axi_awready) begin
                    burst_beat <= 0;
                    state <= ST_W;
                end
                ST_W: if (s_valid && m_axi_wready) begin
                    beats_sent <= beats_sent + 1'b1;
                    if (burst_beat == 15) begin
                        burst_beat <= 0;
                        state <= ST_B;
                    end else begin
                        burst_beat <= burst_beat + 1'b1;
                    end
                end
                ST_B: if (m_axi_bvalid) begin
                    if (m_axi_bresp != 2'b00 || m_axi_bid != AXI_ID)
                        error <= 1'b1;
                    if (beats_sent == total_beats_q) begin
                        done <= 1'b1;
                        state <= ST_IDLE;
                    end else begin
                        addr_q <= addr_q + {{(ADDR_W-10){1'b0}},10'd512};
                        state <= ST_AW;
                    end
                end
            endcase
            if ((state != ST_IDLE) && start)
                error <= 1'b1;
        end
    end
endmodule
