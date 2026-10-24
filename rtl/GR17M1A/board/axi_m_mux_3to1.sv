`timescale 1ns/1ps

// Exclusive three-to-one AXI4 master mux on the processing clock.
// select must stay stable for the whole granted transaction
// (preload / CS chain / scoreboard).
module axi_m_mux_3to1 #(
    parameter integer ADDR_W = 36,
    parameter integer ID_W = 4
) (
    input  wire [1:0]                   select,

    input  wire [3*ID_W-1:0]            s_axi_awid,
    input  wire [3*ADDR_W-1:0]          s_axi_awaddr,
    input  wire [3*8-1:0]               s_axi_awlen,
    input  wire [3*3-1:0]               s_axi_awsize,
    input  wire [3*2-1:0]               s_axi_awburst,
    input  wire [2:0]                   s_axi_awvalid,
    output reg  [2:0]                   s_axi_awready,
    input  wire [3*256-1:0]             s_axi_wdata,
    input  wire [3*32-1:0]              s_axi_wstrb,
    input  wire [2:0]                   s_axi_wlast,
    input  wire [2:0]                   s_axi_wvalid,
    output reg  [2:0]                   s_axi_wready,
    output wire [3*ID_W-1:0]            s_axi_bid,
    output wire [3*2-1:0]               s_axi_bresp,
    output reg  [2:0]                   s_axi_bvalid,
    input  wire [2:0]                   s_axi_bready,

    input  wire [3*ID_W-1:0]            s_axi_arid,
    input  wire [3*ADDR_W-1:0]          s_axi_araddr,
    input  wire [3*8-1:0]               s_axi_arlen,
    input  wire [3*3-1:0]               s_axi_arsize,
    input  wire [3*2-1:0]               s_axi_arburst,
    input  wire [2:0]                   s_axi_arvalid,
    output reg  [2:0]                   s_axi_arready,
    output wire [3*ID_W-1:0]            s_axi_rid,
    output wire [3*256-1:0]             s_axi_rdata,
    output wire [3*2-1:0]               s_axi_rresp,
    output wire [2:0]                   s_axi_rlast,
    output reg  [2:0]                   s_axi_rvalid,
    input  wire [2:0]                   s_axi_rready,

    output reg  [ID_W-1:0]              m_axi_awid,
    output reg  [ADDR_W-1:0]            m_axi_awaddr,
    output reg  [7:0]                   m_axi_awlen,
    output reg  [2:0]                   m_axi_awsize,
    output reg  [1:0]                   m_axi_awburst,
    output reg                          m_axi_awvalid,
    input  wire                         m_axi_awready,
    output reg  [255:0]                 m_axi_wdata,
    output reg  [31:0]                  m_axi_wstrb,
    output reg                          m_axi_wlast,
    output reg                          m_axi_wvalid,
    input  wire                         m_axi_wready,
    input  wire [ID_W-1:0]              m_axi_bid,
    input  wire [1:0]                   m_axi_bresp,
    input  wire                         m_axi_bvalid,
    output reg                          m_axi_bready,
    output reg  [ID_W-1:0]              m_axi_arid,
    output reg  [ADDR_W-1:0]            m_axi_araddr,
    output reg  [7:0]                   m_axi_arlen,
    output reg  [2:0]                   m_axi_arsize,
    output reg  [1:0]                   m_axi_arburst,
    output reg                          m_axi_arvalid,
    input  wire                         m_axi_arready,
    input  wire [ID_W-1:0]              m_axi_rid,
    input  wire [255:0]                 m_axi_rdata,
    input  wire [1:0]                   m_axi_rresp,
    input  wire                         m_axi_rlast,
    input  wire                         m_axi_rvalid,
    output reg                          m_axi_rready
);
    genvar g;
    generate
        for (g = 0; g < 3; g = g + 1) begin : g_resp
            assign s_axi_bid[g*ID_W +: ID_W] = m_axi_bid;
            assign s_axi_bresp[g*2 +: 2] = m_axi_bresp;
            assign s_axi_rid[g*ID_W +: ID_W] = m_axi_rid;
            assign s_axi_rdata[g*256 +: 256] = m_axi_rdata;
            assign s_axi_rresp[g*2 +: 2] = m_axi_rresp;
            assign s_axi_rlast[g] = m_axi_rlast;
        end
    endgenerate

    always @* begin
        m_axi_awid = '0;
        m_axi_awaddr = '0;
        m_axi_awlen = '0;
        m_axi_awsize = '0;
        m_axi_awburst = '0;
        m_axi_awvalid = 1'b0;
        m_axi_wdata = '0;
        m_axi_wstrb = '0;
        m_axi_wlast = 1'b0;
        m_axi_wvalid = 1'b0;
        m_axi_bready = 1'b0;
        m_axi_arid = '0;
        m_axi_araddr = '0;
        m_axi_arlen = '0;
        m_axi_arsize = '0;
        m_axi_arburst = '0;
        m_axi_arvalid = 1'b0;
        m_axi_rready = 1'b0;
        s_axi_awready = 3'b0;
        s_axi_wready = 3'b0;
        s_axi_bvalid = 3'b0;
        s_axi_arready = 3'b0;
        s_axi_rvalid = 3'b0;

        if (select <= 2'd2) begin
            m_axi_awid = s_axi_awid[select*ID_W +: ID_W];
            m_axi_awaddr = s_axi_awaddr[select*ADDR_W +: ADDR_W];
            m_axi_awlen = s_axi_awlen[select*8 +: 8];
            m_axi_awsize = s_axi_awsize[select*3 +: 3];
            m_axi_awburst = s_axi_awburst[select*2 +: 2];
            m_axi_awvalid = s_axi_awvalid[select];
            s_axi_awready[select] = m_axi_awready;
            m_axi_wdata = s_axi_wdata[select*256 +: 256];
            m_axi_wstrb = s_axi_wstrb[select*32 +: 32];
            m_axi_wlast = s_axi_wlast[select];
            m_axi_wvalid = s_axi_wvalid[select];
            s_axi_wready[select] = m_axi_wready;
            m_axi_bready = s_axi_bready[select];
            s_axi_bvalid[select] = m_axi_bvalid;
            m_axi_arid = s_axi_arid[select*ID_W +: ID_W];
            m_axi_araddr = s_axi_araddr[select*ADDR_W +: ADDR_W];
            m_axi_arlen = s_axi_arlen[select*8 +: 8];
            m_axi_arsize = s_axi_arsize[select*3 +: 3];
            m_axi_arburst = s_axi_arburst[select*2 +: 2];
            m_axi_arvalid = s_axi_arvalid[select];
            s_axi_arready[select] = m_axi_arready;
            m_axi_rready = s_axi_rready[select];
            s_axi_rvalid[select] = m_axi_rvalid;
        end
    end
endmodule
