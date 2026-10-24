`timescale 1ns/1ps
`include "cs2048_expected_fold.vh"
// Stage 3b: UDP WR -> gear -> s0, the seven-pass CS chain at s1, UDP RD ->
// gear -> s2.
// Local 125 MHz command/TX/AXI domain crosses to MIG ui_clk through AXI CDC.
// Recovered RX clock crosses only through XPM payload and peer FIFOs.
module k7_udp_ddr_top (
    input  wire        sysclk,        // AD12, 100 MHz, SSTL135
    output wire        uart_tx_o,     // Y26, LVCMOS18
    input wire uart_rx_i,
    output wire phy_rst_a_o, phy_rst_b_o,
    input wire PHYD_rgmii_rxc, PHYD_rgmii_rx_ctl,
    input wire [3:0] PHYD_rgmii_rxd,
    output wire PHYD_rgmii_txc, PHYD_rgmii_tx_ctl,
    output wire [3:0] PHYD_rgmii_txd,
    // ---- DDR3 ----
    output wire [14:0] ddr3_addr,
    output wire [2:0]  ddr3_ba,
    output wire        ddr3_cas_n,
    output wire [0:0]  ddr3_ck_n,
    output wire [0:0]  ddr3_ck_p,
    output wire [0:0]  ddr3_cke,
    output wire [0:0]  ddr3_cs_n,
    output wire [7:0]  ddr3_dm,
    inout  wire [63:0] ddr3_dq,
    inout  wire [7:0]  ddr3_dqs_n,
    inout  wire [7:0]  ddr3_dqs_p,
    output wire [0:0]  ddr3_odt,
    output wire        ddr3_ras_n,
    output wire        ddr3_reset_n,
    output wire        ddr3_we_n
);
    localparam integer ADDR_W = 31;
    localparam integer ID_W   = 4;
    // Power-on reset: no reset pin is assigned, because the board's button
    // pin is not established and guessing an IO location is worse than doing
    // without one.  Power cycling or reprogramming restarts the run.
`ifdef TMR_CS_APP
    // TMR build: these blocks live in k7_boot (as rpr_base), so they carry a name in zone 0.
    wire por_rst_n, calib_done, tap_load;
`else
    reg [15:0] por_cnt = 16'd0;
    always @(posedge sysclk)
        if (!por_cnt[15]) por_cnt <= por_cnt + 16'd1;
    wire por_rst_n = por_cnt[15];
`endif

    wire clk200, clk125, clk125_90, mmcm_locked;
    // ICAPE2 (inside the SEM subsystem) caps at 100 MHz, so it cannot share
    // clk125.  sysclk is already the right frequency; buffer it out in
    // parallel with the clk_wiz input and hand it to the core.
    wire icap_clk;
    BUFG u_icap_bufg (.I(sysclk), .O(icap_clk));
`ifdef CS_CLK_SPLIT
    // rpr_iso fallback: a fourth output for csa_cs_app (cs_xclk.sv); its
    // frequency is set where build_prot_arm.tcl generates clk_wiz_k7.
    wire cs_clk;
    clk_wiz_k7 u_clk (
        .clk_in1(sysclk),
        .clk_out1(clk200), .clk_out2(clk125), .clk_out3(clk125_90), .clk_out4(cs_clk),
        .reset(~por_rst_n),
        .locked(mmcm_locked)
    );
`else
    clk_wiz_k7 u_clk (
        .clk_in1(sysclk),
        .clk_out1(clk200), .clk_out2(clk125), .clk_out3(clk125_90),
        .reset(~por_rst_n),
        .locked(mmcm_locked)
    );
`endif

    wire ui_clk, ui_clk_sync_rst, mig_mmcm_locked, init_calib_complete;

    // Reset distribution lives in k7_reset_tree so it can be simulated;
    // see tb_k7_reset_tree.  Everything about why it is shaped this way is
    // documented there, including the five CDC-10 criticals it removes.
    wire aresetn, proc_rst_n, rx_rst_n, ref_rst_n;
    k7_reset_tree u_rst (
        .sysclk(sysclk), .por_rst_n(por_rst_n), .mmcm_locked(mmcm_locked),
        .ui_clk(ui_clk), .ui_clk_sync_rst(ui_clk_sync_rst),
        .clk125(clk125), .clk200(clk200), .rgmii_rxc(PHYD_rgmii_rxc),
        .aresetn(aresetn), .proc_rst_n(proc_rst_n),
        .rx_rst_n(rx_rst_n), .ref_rst_n(ref_rst_n)
    );
`ifdef TMR_CS_APP
    (* keep_hierarchy = "yes" *)
    k7_boot u_boot (
        .sysclk(sysclk), .por_rst_n(por_rst_n),
        .clk125(clk125), .proc_rst_n(proc_rst_n),
        .init_calib_complete(init_calib_complete), .calib_done(calib_done),
        .clk200(clk200), .ref_rst_n(ref_rst_n), .tap_load(tap_load)
    );
`else
    (* ASYNC_REG = "TRUE" *) reg [2:0] calib_sync;
    always @(posedge clk125) begin
        if (!proc_rst_n) calib_sync <= 0;
        else calib_sync <= {calib_sync[1:0],init_calib_complete};
    end
`endif
    assign phy_rst_a_o = por_rst_n;
    assign phy_rst_b_o = por_rst_n;
`ifndef TMR_CS_APP
    reg [4:0] tap_boot;
    always @(posedge clk200) begin
        if (!ref_rst_n) tap_boot <= 0;
        else if (tap_boot != 31) tap_boot <= tap_boot + 1'b1;
    end
`endif
    wire gmii_rxc, gmii_rx_dv, gmii_tx_dv;
    wire [7:0] gmii_rd, gmii_td;
    rgmii_to_gmii u_rx (
`ifdef TMR_CS_APP
        .idelay_clk(clk200), .tap_value(5'd16), .tap_load(tap_load),
`else
        .idelay_clk(clk200), .tap_value(5'd16), .tap_load(tap_boot == 16),
`endif
        .rst(!rx_rst_n), .rgmii_rxc(PHYD_rgmii_rxc),
        .rgmii_rx_ctl(PHYD_rgmii_rx_ctl), .rgmii_rd(PHYD_rgmii_rxd),
        .gmii_rxc(gmii_rxc), .gmii_rx_dv(gmii_rx_dv), .gmii_rx_err(), .gmii_rd(gmii_rd)
    );
    gmii_to_rgmii u_tx (
        .rst(!proc_rst_n), .idelay_clk(clk200), .gmii_txc(clk125),
        .txc_fwd_clk(clk125_90), .gmii_tx_dv(gmii_tx_dv),
        .gmii_tx_err(1'b0), .gmii_td(gmii_td),
        .rgmii_txc(PHYD_rgmii_txc), .rgmii_tx_ctl(PHYD_rgmii_tx_ctl),
        .rgmii_td(PHYD_rgmii_txd)
    );

    wire [ID_W-1:0]   awid, arid, bid, rid;
    wire [ADDR_W-1:0] awaddr, araddr;
    wire [7:0]        awlen, arlen;
    wire [2:0]        awsize, arsize;
    wire [1:0]        awburst, arburst, bresp, rresp;
    wire              awvalid, awready, wlast, wvalid, wready;
    wire              bvalid, bready, arvalid, arready, rlast, rvalid, rready;
    wire [255:0]      wdata, rdata;
    wire [31:0]       wstrb;

    k7_mig_axi256 u_mig (
        .ddr3_addr(ddr3_addr), .ddr3_ba(ddr3_ba), .ddr3_cas_n(ddr3_cas_n),
        .ddr3_ck_n(ddr3_ck_n), .ddr3_ck_p(ddr3_ck_p), .ddr3_cke(ddr3_cke),
        .ddr3_ras_n(ddr3_ras_n), .ddr3_reset_n(ddr3_reset_n),
        .ddr3_we_n(ddr3_we_n), .ddr3_dq(ddr3_dq),
        .ddr3_dqs_n(ddr3_dqs_n), .ddr3_dqs_p(ddr3_dqs_p),
        .ddr3_cs_n(ddr3_cs_n), .ddr3_dm(ddr3_dm), .ddr3_odt(ddr3_odt),
        .init_calib_complete(init_calib_complete),
        .ui_clk(ui_clk), .ui_clk_sync_rst(ui_clk_sync_rst),
        .mmcm_locked(mig_mmcm_locked), .aresetn(aresetn),
        .app_sr_req(1'b0), .app_ref_req(1'b0), .app_zq_req(1'b0),
        .app_sr_active(), .app_ref_ack(), .app_zq_ack(),
        .sys_clk_i(clk200), .sys_rst(por_rst_n),   // SysResetPolarity = ACTIVE LOW
        .s_axi_awid(awid), .s_axi_awaddr(awaddr), .s_axi_awlen(awlen),
        .s_axi_awsize(awsize), .s_axi_awburst(awburst),
        .s_axi_awlock(1'b0), .s_axi_awcache(4'b0011), .s_axi_awprot(3'b000),
        .s_axi_awqos(4'b0000), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(wstrb), .s_axi_wlast(wlast),
        .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bid(bid), .s_axi_bresp(bresp), .s_axi_bvalid(bvalid),
        .s_axi_bready(bready),
        .s_axi_arid(arid), .s_axi_araddr(araddr), .s_axi_arlen(arlen),
        .s_axi_arsize(arsize), .s_axi_arburst(arburst),
        .s_axi_arlock(1'b0), .s_axi_arcache(4'b0011), .s_axi_arprot(3'b000),
        .s_axi_arqos(4'b0000), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rid(rid), .s_axi_rdata(rdata), .s_axi_rresp(rresp),
        .s_axi_rlast(rlast), .s_axi_rvalid(rvalid), .s_axi_rready(rready)
    );

    wire [3:0] p_awid;
    wire [30:0] p_awaddr;
    wire [7:0] p_awlen;
    wire [2:0] p_awsize;
    wire [1:0] p_awburst;
    wire p_awvalid;
    wire p_awready;
    wire [255:0] p_wdata;
    wire [31:0] p_wstrb;
    wire p_wlast;
    wire p_wvalid;
    wire p_wready;
    wire [3:0] p_bid;
    wire [1:0] p_bresp;
    wire p_bvalid;
    wire p_bready;
    wire [3:0] p_arid;
    wire [30:0] p_araddr;
    wire [7:0] p_arlen;
    wire [2:0] p_arsize;
    wire [1:0] p_arburst;
    wire p_arvalid;
    wire p_arready;
    wire [3:0] p_rid;
    wire [255:0] p_rdata;
    wire [1:0] p_rresp;
    wire p_rlast;
    wire p_rvalid;
    wire p_rready;
    k7_udp_ddr_core #(.CS_EXPECTED_FOLD(CS2048_EXPECTED_FOLD)) u_core (
        .clk(clk125),
`ifdef CS_CLK_SPLIT
        .cs_clk(cs_clk),
`endif
        .rst_n(proc_rst_n),
        .icap_clk(icap_clk),
        .icap_rst_n(por_rst_n),
        .uart_rx_i(uart_rx_i),
        .uart_tx_o(uart_tx_o),
`ifdef TMR_CS_APP
        .calib_done(calib_done),
`else
        .calib_done(calib_sync[2]),
`endif
        .rx_clk(gmii_rxc),
        .rx_rst(!rx_rst_n),
        .gmii_rx_dv(gmii_rx_dv),
        .gmii_rd(gmii_rd),
        .gmii_tx_dv(gmii_tx_dv),
        .gmii_td(gmii_td),
        .m_awid(p_awid),
        .m_awaddr(p_awaddr),
        .m_awlen(p_awlen),
        .m_awsize(p_awsize),
        .m_awburst(p_awburst),
        .m_awvalid(p_awvalid),
        .m_awready(p_awready),
        .m_wdata(p_wdata),
        .m_wstrb(p_wstrb),
        .m_wlast(p_wlast),
        .m_wvalid(p_wvalid),
        .m_wready(p_wready),
        .m_bid(p_bid),
        .m_bresp(p_bresp),
        .m_bvalid(p_bvalid),
        .m_bready(p_bready),
        .m_arid(p_arid),
        .m_araddr(p_araddr),
        .m_arlen(p_arlen),
        .m_arsize(p_arsize),
        .m_arburst(p_arburst),
        .m_arvalid(p_arvalid),
        .m_arready(p_arready),
        .m_rid(p_rid),
        .m_rdata(p_rdata),
        .m_rresp(p_rresp),
        .m_rlast(p_rlast),
        .m_rvalid(p_rvalid),
        .m_rready(p_rready)
    );
    k7_axi_clock_converter  u_axi_cdc (
        .s_axi_aclk(clk125),
        .s_axi_aresetn(proc_rst_n),
        .m_axi_aclk(ui_clk),
        .m_axi_aresetn(aresetn),
        .s_axi_awid(p_awid),
        .s_axi_awaddr(p_awaddr),
        .s_axi_awlen(p_awlen),
        .s_axi_awsize(p_awsize),
        .s_axi_awburst(p_awburst),
        .s_axi_awvalid(p_awvalid),
        .s_axi_awready(p_awready),
        .s_axi_wdata(p_wdata),
        .s_axi_wstrb(p_wstrb),
        .s_axi_wlast(p_wlast),
        .s_axi_wvalid(p_wvalid),
        .s_axi_wready(p_wready),
        .s_axi_bid(p_bid),
        .s_axi_bresp(p_bresp),
        .s_axi_bvalid(p_bvalid),
        .s_axi_bready(p_bready),
        .s_axi_arid(p_arid),
        .s_axi_araddr(p_araddr),
        .s_axi_arlen(p_arlen),
        .s_axi_arsize(p_arsize),
        .s_axi_arburst(p_arburst),
        .s_axi_arvalid(p_arvalid),
        .s_axi_arready(p_arready),
        .s_axi_rid(p_rid),
        .s_axi_rdata(p_rdata),
        .s_axi_rresp(p_rresp),
        .s_axi_rlast(p_rlast),
        .s_axi_rvalid(p_rvalid),
        .s_axi_rready(p_rready),
        .m_axi_awid(awid),
        .m_axi_awaddr(awaddr),
        .m_axi_awlen(awlen),
        .m_axi_awsize(awsize),
        .m_axi_awburst(awburst),
        .m_axi_awvalid(awvalid),
        .m_axi_awready(awready),
        .m_axi_wdata(wdata),
        .m_axi_wstrb(wstrb),
        .m_axi_wlast(wlast),
        .m_axi_wvalid(wvalid),
        .m_axi_wready(wready),
        .m_axi_bid(bid),
        .m_axi_bresp(bresp),
        .m_axi_bvalid(bvalid),
        .m_axi_bready(bready),
        .m_axi_arid(arid),
        .m_axi_araddr(araddr),
        .m_axi_arlen(arlen),
        .m_axi_arsize(arsize),
        .m_axi_arburst(arburst),
        .m_axi_arvalid(arvalid),
        .m_axi_arready(arready),
        .m_axi_rid(rid),
        .m_axi_rdata(rdata),
        .m_axi_rresp(rresp),
        .m_axi_rlast(rlast),
        .m_axi_rvalid(rvalid),
        .m_axi_rready(rready),
        .s_axi_awlock(1'b0),
        .m_axi_awlock(),
        .s_axi_awcache(4'b0011),
        .m_axi_awcache(),
        .s_axi_awprot(3'b0),
        .m_axi_awprot(),
        .s_axi_awqos(4'b0),
        .m_axi_awqos(),
        .s_axi_arlock(1'b0),
        .m_axi_arlock(),
        .s_axi_arcache(4'b0011),
        .m_axi_arcache(),
        .s_axi_arprot(3'b0),
        .m_axi_arprot(),
        .s_axi_arqos(4'b0),
        .m_axi_arqos()
    );
endmodule
