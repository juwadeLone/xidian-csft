`timescale 1ns/1ps
// Reset distribution for k7_udp_ddr_top, factored out so it can be simulated.
//
// It lived inline in the top, which is the one module with no testbench --
// it carries MIG and RGMII primitives that will not elaborate behaviourally.
// A reset tree is the wrong thing to leave unverified: its failure mode is
// "never leaves reset", which on the board looks like a dead design and gives
// no clue where to look.
//
// Two rules the previous inline version broke, both flagged by report_cdc as
// CDC-10 (5 criticals, all landing on the AXI CDC's per-channel synchronisers):
//
//   1. Nothing combinational may sit in front of a synchroniser's async input.
//      A glitch on the gate is indistinguishable from a real reset.  The old
//      code fed `por_rst_n & mmcm_locked & !ui_clk_sync_rst` straight in.
//   2. A signal from another clock domain is not an async reset source.
//      ui_clk_sync_rst belongs to the MIG's ui_clk.  It is a release
//      condition, not an emergency assert, so it is synchronised into each
//      consumer's domain and combined there in a register.
//
// The one asynchronous source is base_rst_n, and it is a register output:
// mmcm_locked async-clears it, por_rst_n releases it synchronously to sysclk.
// sysclk is the board oscillator and runs whether or not the MMCM is locked,
// so this flop is always clocked.
module k7_reset_tree (
    input  wire sysclk,          // board oscillator, free-running
    input  wire por_rst_n,       // sysclk-domain power-on counter
    input  wire mmcm_locked,     // top MMCM, asynchronous

    input  wire ui_clk,          // MIG user clock
    input  wire ui_clk_sync_rst, // MIG reset, ui_clk domain, active high
    input  wire clk125,          // command / TX / AXI domain
    input  wire clk200,          // IDELAYCTRL reference
    input  wire rgmii_rxc,       // recovered RX clock

    output wire aresetn,         // ui_clk,    active low
    output wire proc_rst_n,      // clk125,    active low
    output wire rx_rst_n,        // rgmii_rxc, active low
    output wire ref_rst_n        // clk200,    active low
);

    // The sole asynchronous reset source, and a flop output rather than a gate.
    reg base_rst_n_r = 1'b0;
    always @(posedge sysclk or negedge mmcm_locked)
        if (!mmcm_locked) base_rst_n_r <= 1'b0;
        else              base_rst_n_r <= por_rst_n;
    wire base_rst_n = base_rst_n_r;

    // ---- ui_clk ----
    // ui_clk_sync_rst is native to this domain, so it needs no synchroniser
    // here; it is still registered rather than gated onto the reset net.
    wire aresetn_base;
    k7_reset_sync u_ui_reset (.clk(ui_clk), .arst_n(base_rst_n), .rst_n(aresetn_base));
    reg aresetn_r = 1'b0;
    always @(posedge ui_clk or negedge base_rst_n)
        if (!base_rst_n) aresetn_r <= 1'b0;
        else             aresetn_r <= aresetn_base & ~ui_clk_sync_rst;
    assign aresetn = aresetn_r;

    // ---- clk125 ----
    // Holding the command domain down until the MIG is out of reset keeps the
    // AXI masters from issuing into a slave that is not there yet.
    wire proc_rst_n_base;
    k7_reset_sync u_proc_reset (.clk(clk125), .arst_n(base_rst_n), .rst_n(proc_rst_n_base));
    (* ASYNC_REG = "TRUE" *) reg [2:0] ui_rst_s125 = 3'b111;
    always @(posedge clk125 or negedge base_rst_n)
        if (!base_rst_n) ui_rst_s125 <= 3'b111;
        else             ui_rst_s125 <= {ui_rst_s125[1:0], ui_clk_sync_rst};
    reg proc_rst_n_r = 1'b0;
    always @(posedge clk125 or negedge base_rst_n)
        if (!base_rst_n) proc_rst_n_r <= 1'b0;
        else             proc_rst_n_r <= proc_rst_n_base & ~ui_rst_s125[2];
    assign proc_rst_n = proc_rst_n_r;

    // ---- recovered RX clock ----
    // Chained off proc_rst_n (a register output, so still no gate) so the RX
    // path cannot run ahead of the domain it hands data to.  With no link
    // there is no rxc and this stays asserted, which is the behaviour we want.
    k7_reset_sync u_rx_reset (.clk(rgmii_rxc), .arst_n(proc_rst_n), .rst_n(rx_rst_n));

    // ---- IDELAYCTRL reference ----
    // Independent of the MIG on purpose: the IDELAY taps must be calibrated
    // before the link comes up, and the link is not waiting for DDR.
    k7_reset_sync u_ref_reset (.clk(clk200), .arst_n(base_rst_n), .rst_n(ref_rst_n));

endmodule
