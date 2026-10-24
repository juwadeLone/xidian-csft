`timescale 1ns/1ps
// The logic that used to sit directly in k7_udp_ddr_top -- the power-on reset
// counter, the calibration-complete synchroniser and the IDELAY tap-load
// counter.  It is in a module of its own so that its cells carry a name prefix
// (u_boot/) and are placed in the injection / observation region with the rest
// of zone 0 (vivado/zone_pblock.tcl, zone_check.tcl).  The three blocks are
// moved as they were; nothing else changes.
module k7_boot (
    // Power-on reset: no reset pin is assigned, because the board's button
    // pin is not established and guessing an IO location is worse than doing
    // without one.  Power cycling or reprogramming restarts the run.
    input  wire sysclk,
    output wire por_rst_n,
    // MIG calibration complete, into the 125 MHz domain
    input  wire clk125,
    input  wire proc_rst_n,
    input  wire init_calib_complete,
    output wire calib_done,
    // one load pulse for the RGMII input delays after the reference clock is up
    input  wire clk200,
    input  wire ref_rst_n,
    output wire tap_load
);
    reg [15:0] por_cnt = 16'd0;
    always @(posedge sysclk)
        if (!por_cnt[15]) por_cnt <= por_cnt + 16'd1;
    assign por_rst_n = por_cnt[15];

    (* ASYNC_REG = "TRUE" *) reg [2:0] calib_sync;
    always @(posedge clk125) begin
        if (!proc_rst_n) calib_sync <= 0;
        else calib_sync <= {calib_sync[1:0],init_calib_complete};
    end
    assign calib_done = calib_sync[2];

    reg [4:0] tap_boot;
    always @(posedge clk200) begin
        if (!ref_rst_n) tap_boot <= 0;
        else if (tap_boot != 31) tap_boot <= tap_boot + 1'b1;
    end
    assign tap_load = (tap_boot == 16);
endmodule
