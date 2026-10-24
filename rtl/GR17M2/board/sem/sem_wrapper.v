`timescale 1ns / 1ps
// PG036 7-series wrapper: match official sem_0_sem_cfg (direct ICAPE2, no bitswap).
// Injection only via monitor N (inject_strobe tied off). Sole ICAP user.
module sem_wrapper (
    input         icap_clk,
    input         icap_resetn,
    output        status_heartbeat,
    output        status_initialization,
    output        status_observation,
    output        status_correction,
    output        status_classification,
    output        status_injection,
    output        status_essential,
    output        status_uncorrectable,
    output        status_idle,
    output        icap_request,
    output        icap_grant,
    output [7:0]  monitor_txdata,
    output        monitor_txwrite,
    input         monitor_txfull,
    input  [7:0]  monitor_rxdata,
    output        monitor_rxread,
    input         monitor_rxempty
);

wire [31:0] icap_i, icap_o;
wire        icap_csib, icap_rdwrb;
wire        fecc_crcerr, fecc_eccerr, fecc_eccerrsingle, fecc_syndromevalid;
wire [12:0] fecc_syndrome;
wire [25:0] fecc_far;
wire [4:0]  fecc_synbit;
wire [6:0]  fecc_synword;

assign status_idle = ~(status_initialization | status_observation |
                       status_correction | status_classification |
                       status_injection);
assign icap_grant  = 1'b1;

FRAME_ECCE2 #(
    .FARSRC("EFAR"),
    .FRAME_RBT_IN_FILENAME("NONE")
) u_frame_ecc (
    .CRCERROR      (fecc_crcerr),
    .ECCERROR      (fecc_eccerr),
    .ECCERRORSINGLE(fecc_eccerrsingle),
    .FAR           (fecc_far),
    .SYNBIT        (fecc_synbit),
    .SYNDROME      (fecc_syndrome),
    .SYNDROMEVALID (fecc_syndromevalid),
    .SYNWORD       (fecc_synword)
);

ICAPE2 #(
    .DEVICE_ID(32'hFFFFFFFF),
    .ICAP_WIDTH("X32"),
    .SIM_CFG_FILE_NAME("NONE")
) u_icape2 (
    .CLK  (icap_clk),
    .CSIB (icap_csib),
    .RDWRB(icap_rdwrb),
    .I    (icap_i),
    .O    (icap_o)
);

sem_0 u_sem_0 (
    .status_heartbeat      (status_heartbeat),
    .status_initialization (status_initialization),
    .status_observation    (status_observation),
    .status_correction     (status_correction),
    .status_classification (status_classification),
    .status_injection      (status_injection),
    .status_essential      (status_essential),
    .status_uncorrectable  (status_uncorrectable),
    .monitor_txdata        (monitor_txdata),
    .monitor_txwrite       (monitor_txwrite),
    .monitor_txfull        (monitor_txfull),
    .monitor_rxdata        (monitor_rxdata),
    .monitor_rxread        (monitor_rxread),
    .monitor_rxempty       (monitor_rxempty),
    .inject_strobe         (1'b0),
    .inject_address        (40'd0),
    .icap_o                (icap_o),
    .icap_csib             (icap_csib),
    .icap_rdwrb            (icap_rdwrb),
    .icap_i                (icap_i),
    .icap_clk              (icap_clk),
    .icap_request          (icap_request),
    .icap_grant            (icap_grant),
    .fecc_crcerr           (fecc_crcerr),
    .fecc_eccerr           (fecc_eccerr),
    .fecc_eccerrsingle     (fecc_eccerrsingle),
    .fecc_syndromevalid    (fecc_syndromevalid),
    .fecc_syndrome         (fecc_syndrome),
    .fecc_far              (fecc_far),
    .fecc_synbit           (fecc_synbit),
    .fecc_synword          (fecc_synword)
);

endmodule
