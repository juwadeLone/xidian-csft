`timescale 1ns/1ps

// Full CS imaging chain, DDR-to-DDR, square frame, single-lane SDF.
//
// Seven passes over five frame buffers in one DDR space (in-place tile-swap
// transposes per the locked single-DDR architecture):
//   P0 swap-transpose B0          (RAW becomes range-major in place)
//   P1 AZ FFT (centred) + H1      B0 -> B1   (azimuth vectors, length NAZ)
//   P2 swap-transpose B1
//   P3 RG FFT (centred) + H2      B1 -> B2   (range vectors, length NR)
//   P4 RG IFFT + H3               B2 -> B3
//   P5 swap-transpose B3
//   P6 C4 + AZ IFFT (un-centred)  B3 -> OUT
//
// rpr_noroll: the three half-turn rolls of CS_SEED_DERIVATION_2026-08-23.md
// ((-1)^n before the P1 and P3 FFTs, after the P6 IFFT) are replaced by
// addressing.  A roll in front of an FFT is a circular shift of its output by
// N/2, so the P1 and P3 reorder banks are read with the index MSB inverted
// (FFT_OUT_SHIFT); the un-roll after the P6 IFFT is the same shift of its
// input, so the P6 prefetch banks are read the same way (FFT_IN_SHIFT).  H3
// keeps its +pi*n: the P4 IFFT still sees a centred spectrum.  The identity
// window of P3 is gone as well.  H1/H2/H3/C4 are unchanged, driven per vector
// through the top-level seed streams, muxed by active_pass_id.
//
// Widths follow the trimmed fixed-point model (configs/final_w17t25.json): every
// pass is a 17-bit fixed-point SDF lane with its own scale mask, every memory
// and the output hold 17-bit components, and H1/H2/H3/C4 take and produce 17
// bits (multiply by a 25-bit coefficient, round half_up, saturate to 17).  The
// input stays 12 bits: the P1 lane entry clamps each 17-bit entry component to
// 12 bits and sign-extends it (LANE_ENTRY_W 12, the model's input limit; words
// inside 12 bits pass unchanged).  overflow_sticky is informational.

module csa_cs_chain_top #(
    parameter integer SIDE_LOG2 = 8,     // log2(frame side), square frame
    parameter integer ADDR_W = 36,
    parameter integer ID_W = 4,
    parameter [11:0] AZ_SCALE_MASK = 12'h680,       // P1 azimuth FFT
    parameter [11:0] RG_SCALE_MASK = 12'h6aa,       // P3 range FFT
    parameter [11:0] RG_IFFT_SCALE_MASK = 12'h7ac,  // P4 range IFFT
    parameter [11:0] AZ_IFFT_SCALE_MASK = 12'h7fa   // P6 azimuth IFFT
) (
    input  wire                     clk,
    input  wire                     rst,
    input  wire                     clear,
    input  wire                     start,
    // TMR16: voted-state override (driven by csa_cs_app_tmr).  While
    // align is high the scheduler's effective state and pass index are
    // the majority's (state_c/pass_c below), so this replica runs the
    // same transitions and engine start pulses in the same cycle as the
    // majority; the state register follows the voted trajectory and
    // pass_q is forced to the voted pass.  Tie 0 outside the wrapper.
    input  wire                     align,
    input  wire [2:0]               align_state,
    input  wire [2:0]               align_pass,
    input  wire [ADDR_W-1:0]        base_b0,
    input  wire [ADDR_W-1:0]        base_b1,
    input  wire [ADDR_W-1:0]        base_b2,
    input  wire [ADDR_W-1:0]        base_b3,
    input  wire [ADDR_W-1:0]        base_out,

    input  wire                         pre_seed_valid,
    output wire                         pre_seed_ready,
    input  wire [31:0]                  pre_seed_theta0,
    input  wire [31:0]                  pre_seed_delta1,
    input  wire [31:0]                  pre_seed_delta2,
    input  wire                         post_seed_valid,
    output wire                         post_seed_ready,
    input  wire [31:0]                  post_seed_theta0,
    input  wire [31:0]                  post_seed_delta1,
    input  wire [31:0]                  post_seed_delta2,
    output wire [2:0]              active_pass_id,
    output wire [2:0]              sched_state,   // raw state, TMR align vote
    output wire                     busy,
    output reg                      done,
    output wire                     error,
    output wire                     protocol_error,
    output wire                     overflow_sticky,
    output wire [3:0]               o_eng_ovf,      // {p6, p4, p3, p1}: the terms of overflow_sticky

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
    localparam integer SIDE = 1 << SIDE_LOG2;
    localparam integer VECTORS = SIDE;

    initial begin
        if (SIDE_LOG2 < 6 || SIDE_LOG2 > 12)
            $fatal(1,"SIDE_LOG2 must be in 6..12");
    end

    // ---- grant encoding: which engine owns the AXI interface ----
    localparam [2:0] GRANT_SWAP = 3'd0;
    localparam [2:0] GRANT_P1   = 3'd1;
    localparam [2:0] GRANT_P3   = 3'd2;
    localparam [2:0] GRANT_P4   = 3'd3;
    localparam [2:0] GRANT_P6   = 3'd4;

    localparam [2:0] ST_IDLE   = 3'd0;
    localparam [2:0] ST_LAUNCH = 3'd1;
    localparam [2:0] ST_RUN    = 3'd2;
    localparam [2:0] ST_DRAIN  = 3'd3;
    localparam [2:0] ST_HOLD   = 3'd4;
    localparam [2:0] ST_PRECLEAR = 3'd5;

    reg [2:0] state;
    reg [2:0] pass_q;
    reg [2:0] grant_q;

    // TMR16: effective scheduling state -- the voted values while align
    // is high, the local registers otherwise (identical when align=0).
    wire [2:0] state_c = align ? align_state : state;
    wire [2:0] pass_c  = align ? align_pass  : pass_q;
    assign sched_state = state;
    reg swap_start, p1_start, p3_start, p4_start, p6_start;
    reg pass_clear;
    reg [ADDR_W-1:0] swap_base_q;
    reg [ADDR_W-1:0] p_src_q, p_dst_q;

    wire swap_busy, swap_done, swap_error;
    wire p1_busy, p1_done, p1_error, p1_proto, p1_ovf;
    wire p3_busy, p3_done, p3_error, p3_proto, p3_ovf;
    wire p4_busy, p4_done, p4_error, p4_proto, p4_ovf;
    wire p6_busy, p6_done, p6_error, p6_proto, p6_ovf;

    // Per-pass engine configuration (all engines share the frame side).
    wire is_p1 = pass_c == 3'd1;
    wire is_p3 = pass_c == 3'd3;
    wire is_p4 = pass_c == 3'd4;
    wire is_p6 = pass_c == 3'd6;

    // AXI: five master bundles.
    wire [ID_W-1:0]   awid_m [0:4];
    wire [ADDR_W-1:0] awaddr_m [0:4];
    wire [7:0]        awlen_m [0:4];
    wire [2:0]        awsize_m [0:4];
    wire [1:0]        awburst_m [0:4];
    wire              awvalid_m [0:4];
    wire              awready_m [0:4];
    wire [255:0]      wdata_m [0:4];
    wire [31:0]       wstrb_m [0:4];
    wire              wlast_m [0:4];
    wire              wvalid_m [0:4];
    wire              wready_m [0:4];
    wire [ID_W-1:0]   bid_m [0:4];
    wire [1:0]        bresp_m [0:4];
    wire              bvalid_m [0:4];
    wire              bready_m [0:4];
    wire [ID_W-1:0]   arid_m [0:4];
    wire [ADDR_W-1:0] araddr_m [0:4];
    wire [7:0]        arlen_m [0:4];
    wire [2:0]        arsize_m [0:4];
    wire [1:0]        arburst_m [0:4];
    wire              arvalid_m [0:4];
    wire              arready_m [0:4];
    wire [ID_W-1:0]   rid_m [0:4];
    wire [255:0]      rdata_m [0:4];
    wire [1:0]        rresp_m [0:4];
    wire              rlast_m [0:4];
    wire              rvalid_m [0:4];
    wire              rready_m [0:4];

    assign m_axi_awid    = awid_m[grant_q];
    assign m_axi_awaddr  = awaddr_m[grant_q];
    assign m_axi_awlen   = awlen_m[grant_q];
    assign m_axi_awsize  = awsize_m[grant_q];
    assign m_axi_awburst = awburst_m[grant_q];
    assign m_axi_awvalid = awvalid_m[grant_q];
    assign m_axi_wdata   = wdata_m[grant_q];
    assign m_axi_wstrb   = wstrb_m[grant_q];
    assign m_axi_wlast   = wlast_m[grant_q];
    assign m_axi_wvalid  = wvalid_m[grant_q];
    assign m_axi_bready  = bready_m[grant_q];
    assign m_axi_arid    = arid_m[grant_q];
    assign m_axi_araddr  = araddr_m[grant_q];
    assign m_axi_arlen   = arlen_m[grant_q];
    assign m_axi_arsize  = arsize_m[grant_q];
    assign m_axi_arburst = arburst_m[grant_q];
    assign m_axi_arvalid = arvalid_m[grant_q];
    assign m_axi_rready  = rready_m[grant_q];
    for (genvar g = 0; g < 5; g = g + 1) begin : g_axi_ready
        assign awready_m[g] = (grant_q == g) ? m_axi_awready : 1'b0;
        assign wready_m[g]  = (grant_q == g) ? m_axi_wready  : 1'b0;
        assign bid_m[g]     = (grant_q == g) ? m_axi_bid     : {ID_W{1'b0}};
        assign bresp_m[g]   = (grant_q == g) ? m_axi_bresp   : 2'b00;
        assign bvalid_m[g]  = (grant_q == g) ? m_axi_bvalid  : 1'b0;
        assign arready_m[g] = (grant_q == g) ? m_axi_arready : 1'b0;
        assign rid_m[g]     = (grant_q == g) ? m_axi_rid     : {ID_W{1'b0}};
        assign rdata_m[g]   = (grant_q == g) ? m_axi_rdata   : 256'd0;
        assign rresp_m[g]   = (grant_q == g) ? m_axi_rresp   : 2'b00;
        assign rlast_m[g]   = (grant_q == g) ? m_axi_rlast   : 1'b0;
        assign rvalid_m[g]  = (grant_q == g) ? m_axi_rvalid  : 1'b0;
    end

    axi_matrix_swap_transpose_engine #(
        .ADDR_W(ADDR_W), .ID_W(ID_W),
        .MATRIX_LOG2(SIDE_LOG2), .TILE_LOG2(6)
    ) u_swap (
        .clk(clk), .rst(rst), .clear(clear || pass_clear), .start(swap_start),
        .base_addr(swap_base_q),
        .busy(swap_busy), .done(swap_done), .error(swap_error),
        .m_axi_awid(awid_m[0]), .m_axi_awaddr(awaddr_m[0]),
        .m_axi_awlen(awlen_m[0]), .m_axi_awsize(awsize_m[0]),
        .m_axi_awburst(awburst_m[0]), .m_axi_awvalid(awvalid_m[0]),
        .m_axi_awready(awready_m[0]), .m_axi_wdata(wdata_m[0]),
        .m_axi_wstrb(wstrb_m[0]), .m_axi_wlast(wlast_m[0]),
        .m_axi_wvalid(wvalid_m[0]), .m_axi_wready(wready_m[0]),
        .m_axi_bid(bid_m[0]), .m_axi_bresp(bresp_m[0]),
        .m_axi_bvalid(bvalid_m[0]), .m_axi_bready(bready_m[0]),
        .m_axi_arid(arid_m[0]), .m_axi_araddr(araddr_m[0]),
        .m_axi_arlen(arlen_m[0]), .m_axi_arsize(arsize_m[0]),
        .m_axi_arburst(arburst_m[0]), .m_axi_arvalid(arvalid_m[0]),
        .m_axi_arready(arready_m[0]), .m_axi_rid(rid_m[0]),
        .m_axi_rdata(rdata_m[0]), .m_axi_rresp(rresp_m[0]),
        .m_axi_rlast(rlast_m[0]), .m_axi_rvalid(rvalid_m[0]),
        .m_axi_rready(rready_m[0])
    );

    // Seed routing: one public stream set, muxed to the active engine.
    wire p1_pre_ready, p1_post_ready;
    wire p3_pre_ready, p3_post_ready;
    wire p4_post_ready;
    wire p6_pre_ready, p6_post_ready;

    // Pre seeds only in P6 (C4), post seeds only in P1/P3/P4 (H1/H2/H3).
    assign pre_seed_ready = is_p6 ? p6_pre_ready : 1'b0;
    assign post_seed_ready =
        is_p1 ? p1_post_ready : is_p3 ? p3_post_ready :
        is_p4 ? p4_post_ready : 1'b0;

    axi_fft_coeff_pass_engine #(
        .LOGN(SIDE_LOG2), .ADDR_W(ADDR_W), .ID_W(ID_W),
        .INVERSE(0), .SCALE_MASK(AZ_SCALE_MASK), .CORE_ID(4'h1),
        .BFP_LANE(0), .LANE_W(17), .LANE_IN_W(17), .LANE_OUT_W(17), .LANE_ENTRY_W(12),
        .CORDIC_ITERATIONS(25), .COEF_W(25),
        .PRE_IN_W(17), .PRE_OUT_W(17), .POST_IN_W(17), .POST_OUT_W(17),
        .FFT_OUT_SHIFT(1), .FFT_IN_SHIFT(0)
    ) u_p1 (
        .clk(clk), .rst(rst), .clear(clear || pass_clear), .start(p1_start),
        .source_addr(p_src_q), .destination_addr(p_dst_q),
        .vector_count(VECTORS),
        .pre_coeff_enable(1'b0), .post_coeff_enable(1'b1),   // rpr_noroll: no C0 roll
        .pre_seed_valid(1'b0),
        .pre_seed_ready(p1_pre_ready),
        .pre_seed_theta0(pre_seed_theta0),
        .pre_seed_delta1(pre_seed_delta1),
        .pre_seed_delta2(pre_seed_delta2),
        .post_seed_valid(post_seed_valid && is_p1),
        .post_seed_ready(p1_post_ready),
        .post_seed_theta0(post_seed_theta0),
        .post_seed_delta1(post_seed_delta1),
        .post_seed_delta2(post_seed_delta2),
        .busy(p1_busy), .done(p1_done), .error(p1_error),
        .protocol_error(p1_proto), .overflow_sticky(p1_ovf),
        .m_axi_awid(awid_m[1]), .m_axi_awaddr(awaddr_m[1]),
        .m_axi_awlen(awlen_m[1]), .m_axi_awsize(awsize_m[1]),
        .m_axi_awburst(awburst_m[1]), .m_axi_awvalid(awvalid_m[1]),
        .m_axi_awready(awready_m[1]), .m_axi_wdata(wdata_m[1]),
        .m_axi_wstrb(wstrb_m[1]), .m_axi_wlast(wlast_m[1]),
        .m_axi_wvalid(wvalid_m[1]), .m_axi_wready(wready_m[1]),
        .m_axi_bid(bid_m[1]), .m_axi_bresp(bresp_m[1]),
        .m_axi_bvalid(bvalid_m[1]), .m_axi_bready(bready_m[1]),
        .m_axi_arid(arid_m[1]), .m_axi_araddr(araddr_m[1]),
        .m_axi_arlen(arlen_m[1]), .m_axi_arsize(arsize_m[1]),
        .m_axi_arburst(arburst_m[1]), .m_axi_arvalid(arvalid_m[1]),
        .m_axi_arready(arready_m[1]), .m_axi_rid(rid_m[1]),
        .m_axi_rdata(rdata_m[1]), .m_axi_rresp(rresp_m[1]),
        .m_axi_rlast(rlast_m[1]), .m_axi_rvalid(rvalid_m[1]),
        .m_axi_rready(rready_m[1])
    );

    axi_fft_coeff_pass_engine #(
        .LOGN(SIDE_LOG2), .ADDR_W(ADDR_W), .ID_W(ID_W),
        .INVERSE(0), .SCALE_MASK(RG_SCALE_MASK), .CORE_ID(4'h3),
        .BFP_LANE(0), .LANE_W(17), .LANE_IN_W(17), .LANE_OUT_W(17),
        .CORDIC_ITERATIONS(25), .COEF_W(25),
        .PRE_IN_W(17), .PRE_OUT_W(17), .POST_IN_W(17), .POST_OUT_W(17),
        .FFT_OUT_SHIFT(1), .FFT_IN_SHIFT(0)
    ) u_p3 (
        .clk(clk), .rst(rst), .clear(clear || pass_clear), .start(p3_start),
        .source_addr(p_src_q), .destination_addr(p_dst_q),
        .vector_count(VECTORS),
        .pre_coeff_enable(1'b0), .post_coeff_enable(1'b1),   // rpr_noroll: no range roll
        .pre_seed_valid(1'b0),
        .pre_seed_ready(p3_pre_ready),
        .pre_seed_theta0(pre_seed_theta0),
        .pre_seed_delta1(pre_seed_delta1),
        .pre_seed_delta2(pre_seed_delta2),
        .post_seed_valid(post_seed_valid && is_p3),
        .post_seed_ready(p3_post_ready),
        .post_seed_theta0(post_seed_theta0),
        .post_seed_delta1(post_seed_delta1),
        .post_seed_delta2(post_seed_delta2),
        .busy(p3_busy), .done(p3_done), .error(p3_error),
        .protocol_error(p3_proto), .overflow_sticky(p3_ovf),
        .m_axi_awid(awid_m[2]), .m_axi_awaddr(awaddr_m[2]),
        .m_axi_awlen(awlen_m[2]), .m_axi_awsize(awsize_m[2]),
        .m_axi_awburst(awburst_m[2]), .m_axi_awvalid(awvalid_m[2]),
        .m_axi_awready(awready_m[2]), .m_axi_wdata(wdata_m[2]),
        .m_axi_wstrb(wstrb_m[2]), .m_axi_wlast(wlast_m[2]),
        .m_axi_wvalid(wvalid_m[2]), .m_axi_wready(wready_m[2]),
        .m_axi_bid(bid_m[2]), .m_axi_bresp(bresp_m[2]),
        .m_axi_bvalid(bvalid_m[2]), .m_axi_bready(bready_m[2]),
        .m_axi_arid(arid_m[2]), .m_axi_araddr(araddr_m[2]),
        .m_axi_arlen(arlen_m[2]), .m_axi_arsize(arsize_m[2]),
        .m_axi_arburst(arburst_m[2]), .m_axi_arvalid(arvalid_m[2]),
        .m_axi_arready(arready_m[2]), .m_axi_rid(rid_m[2]),
        .m_axi_rdata(rdata_m[2]), .m_axi_rresp(rresp_m[2]),
        .m_axi_rlast(rlast_m[2]), .m_axi_rvalid(rvalid_m[2]),
        .m_axi_rready(rready_m[2])
    );

    axi_fft_coeff_pass_engine #(
        .LOGN(SIDE_LOG2), .ADDR_W(ADDR_W), .ID_W(ID_W),
        .INVERSE(1), .SCALE_MASK(RG_IFFT_SCALE_MASK), .CORE_ID(4'h4),
        .BFP_LANE(0), .LANE_W(17), .LANE_IN_W(17), .LANE_OUT_W(17),
        .CORDIC_ITERATIONS(25), .COEF_W(25),
        .PRE_IN_W(17), .PRE_OUT_W(17), .POST_IN_W(17), .POST_OUT_W(17),
        .FFT_OUT_SHIFT(0), .FFT_IN_SHIFT(0)
    ) u_p4 (
        .clk(clk), .rst(rst), .clear(clear || pass_clear), .start(p4_start),
        .source_addr(p_src_q), .destination_addr(p_dst_q),
        .vector_count(VECTORS),
        .pre_coeff_enable(1'b0), .post_coeff_enable(1'b1),
        .pre_seed_valid(1'b0), .pre_seed_ready(),
        .pre_seed_theta0(32'd0), .pre_seed_delta1(32'd0),
        .pre_seed_delta2(32'd0),
        .post_seed_valid(post_seed_valid && is_p4),
        .post_seed_ready(p4_post_ready),
        .post_seed_theta0(post_seed_theta0),
        .post_seed_delta1(post_seed_delta1),
        .post_seed_delta2(post_seed_delta2),
        .busy(p4_busy), .done(p4_done), .error(p4_error),
        .protocol_error(p4_proto), .overflow_sticky(p4_ovf),
        .m_axi_awid(awid_m[3]), .m_axi_awaddr(awaddr_m[3]),
        .m_axi_awlen(awlen_m[3]), .m_axi_awsize(awsize_m[3]),
        .m_axi_awburst(awburst_m[3]), .m_axi_awvalid(awvalid_m[3]),
        .m_axi_awready(awready_m[3]), .m_axi_wdata(wdata_m[3]),
        .m_axi_wstrb(wstrb_m[3]), .m_axi_wlast(wlast_m[3]),
        .m_axi_wvalid(wvalid_m[3]), .m_axi_wready(wready_m[3]),
        .m_axi_bid(bid_m[3]), .m_axi_bresp(bresp_m[3]),
        .m_axi_bvalid(bvalid_m[3]), .m_axi_bready(bready_m[3]),
        .m_axi_arid(arid_m[3]), .m_axi_araddr(araddr_m[3]),
        .m_axi_arlen(arlen_m[3]), .m_axi_arsize(arsize_m[3]),
        .m_axi_arburst(arburst_m[3]), .m_axi_arvalid(arvalid_m[3]),
        .m_axi_arready(arready_m[3]), .m_axi_rid(rid_m[3]),
        .m_axi_rdata(rdata_m[3]), .m_axi_rresp(rresp_m[3]),
        .m_axi_rlast(rlast_m[3]), .m_axi_rvalid(rvalid_m[3]),
        .m_axi_rready(rready_m[3])
    );

    axi_fft_coeff_pass_engine #(
        .LOGN(SIDE_LOG2), .ADDR_W(ADDR_W), .ID_W(ID_W),
        .INVERSE(1), .SCALE_MASK(AZ_IFFT_SCALE_MASK), .CORE_ID(4'h6),
        .BFP_LANE(0), .LANE_W(17), .LANE_IN_W(17), .LANE_OUT_W(17),
        .CORDIC_ITERATIONS(25), .COEF_W(25),
        .PRE_IN_W(17), .PRE_OUT_W(17), .POST_IN_W(17), .POST_OUT_W(17),
        .FFT_OUT_SHIFT(0), .FFT_IN_SHIFT(1)
    ) u_p6 (
        .clk(clk), .rst(rst), .clear(clear || pass_clear), .start(p6_start),
        .source_addr(p_src_q), .destination_addr(p_dst_q),
        .vector_count(VECTORS),
        .pre_coeff_enable(1'b1), .post_coeff_enable(1'b0),   // rpr_noroll: no un-roll
        .pre_seed_valid(pre_seed_valid && is_p6),
        .pre_seed_ready(p6_pre_ready),
        .pre_seed_theta0(pre_seed_theta0),
        .pre_seed_delta1(pre_seed_delta1),
        .pre_seed_delta2(pre_seed_delta2),
        .post_seed_valid(1'b0),
        .post_seed_ready(p6_post_ready),
        .post_seed_theta0(post_seed_theta0),
        .post_seed_delta1(post_seed_delta1),
        .post_seed_delta2(post_seed_delta2),
        .busy(p6_busy), .done(p6_done), .error(p6_error),
        .protocol_error(p6_proto), .overflow_sticky(p6_ovf),
        .m_axi_awid(awid_m[4]), .m_axi_awaddr(awaddr_m[4]),
        .m_axi_awlen(awlen_m[4]), .m_axi_awsize(awsize_m[4]),
        .m_axi_awburst(awburst_m[4]), .m_axi_awvalid(awvalid_m[4]),
        .m_axi_awready(awready_m[4]), .m_axi_wdata(wdata_m[4]),
        .m_axi_wstrb(wstrb_m[4]), .m_axi_wlast(wlast_m[4]),
        .m_axi_wvalid(wvalid_m[4]), .m_axi_wready(wready_m[4]),
        .m_axi_bid(bid_m[4]), .m_axi_bresp(bresp_m[4]),
        .m_axi_bvalid(bvalid_m[4]), .m_axi_bready(bready_m[4]),
        .m_axi_arid(arid_m[4]), .m_axi_araddr(araddr_m[4]),
        .m_axi_arlen(arlen_m[4]), .m_axi_arsize(arsize_m[4]),
        .m_axi_arburst(arburst_m[4]), .m_axi_arvalid(arvalid_m[4]),
        .m_axi_arready(arready_m[4]), .m_axi_rid(rid_m[4]),
        .m_axi_rdata(rdata_m[4]), .m_axi_rresp(rresp_m[4]),
        .m_axi_rlast(rlast_m[4]), .m_axi_rvalid(rvalid_m[4]),
        .m_axi_rready(rready_m[4])
    );

    // P0/P2/P5 all reuse the same in-place swap-transpose engine.
    wire is_swap_pass = (pass_c == 3'd0) || (pass_c == 3'd2) || (pass_c == 3'd5);
    wire pass_done = is_swap_pass ? swap_done :
                     (pass_c == 3'd1) ? p1_done :
                     (pass_c == 3'd3) ? p3_done :
                     (pass_c == 3'd4) ? p4_done : p6_done;
    wire pass_busy = is_swap_pass ? swap_busy :
                     (pass_c == 3'd1) ? p1_busy :
                     (pass_c == 3'd3) ? p3_busy :
                     (pass_c == 3'd4) ? p4_busy : p6_busy;

    // Raw pass_q, not pass_c: this output feeds the TMR sched vector in
    // csa_cs_app_tmr, and voting/aligning on pass_c would close a
    // combinational loop pass_c -> sched_v -> align_pass -> pass_c that
    // locks at X in simulation.  pass_q itself is forced to the voted
    // value while align is high (below), so the raw register is the
    // correct divergence observable.  Identical to pre-TMR behaviour.
    assign active_pass_id = pass_q;
    assign error = swap_error || p1_error || p3_error || p4_error ||
                   p6_error;
    assign protocol_error = p1_proto || p3_proto || p4_proto || p6_proto;
    assign overflow_sticky = p1_ovf || p3_ovf || p4_ovf || p6_ovf;
    assign o_eng_ovf = {p6_ovf, p4_ovf, p3_ovf, p1_ovf};
    assign busy = (state_c != ST_IDLE && state_c != ST_HOLD) || swap_busy ||
                  p1_busy || p3_busy || p4_busy || p6_busy;

    // Next pass after the current one; 7 passes: 0,1,2,3,4,5,6.
    function [2:0] next_pass;
        input [2:0] p;
        begin
            case (p)
                3'd0: next_pass = 3'd1;
                3'd1: next_pass = 3'd2;
                3'd2: next_pass = 3'd3;
                3'd3: next_pass = 3'd4;
                3'd4: next_pass = 3'd5;
                3'd5: next_pass = 3'd6;
                default: next_pass = 3'd0;
            endcase
        end
    endfunction

    // Per-pass launch configuration.
    function [2:0] pass_grant;
        input [2:0] p;
        begin
            case (p)
                3'd0, 3'd2, 3'd5: pass_grant = GRANT_SWAP;
                3'd1: pass_grant = GRANT_P1;
                3'd3: pass_grant = GRANT_P3;
                3'd4: pass_grant = GRANT_P4;
                default: pass_grant = GRANT_P6;
            endcase
        end
    endfunction

    function [ADDR_W-1:0] pass_swap_base;
        input [2:0] p;
        begin
            case (p)
                3'd0: pass_swap_base = base_b0;
                3'd2: pass_swap_base = base_b1;
                default: pass_swap_base = base_b3;
            endcase
        end
    endfunction

    function [ADDR_W-1:0] pass_source;
        input [2:0] p;
        begin
            case (p)
                3'd1: pass_source = base_b0;
                3'd3: pass_source = base_b1;
                3'd4: pass_source = base_b2;
                default: pass_source = base_b3;
            endcase
        end
    endfunction

    function [ADDR_W-1:0] pass_destination;
        input [2:0] p;
        begin
            case (p)
                3'd1: pass_destination = base_b1;
                3'd3: pass_destination = base_b2;
                3'd4: pass_destination = base_b3;
                default: pass_destination = base_out;
            endcase
        end
    endfunction

    always @(posedge clk) begin
        if (rst || clear) begin
            state <= ST_IDLE;
            pass_q <= 3'd0;
            grant_q <= GRANT_SWAP;
            swap_start <= 1'b0;
            p1_start <= 1'b0;
            p3_start <= 1'b0;
            p4_start <= 1'b0;
            p6_start <= 1'b0;
            pass_clear <= 1'b0;
            swap_base_q <= '0;
            p_src_q <= '0;
            p_dst_q <= '0;
            done <= 1'b0;
        end else begin
            done <= 1'b0;
            swap_start <= 1'b0;
            p1_start <= 1'b0;
            p3_start <= 1'b0;
            p4_start <= 1'b0;
            p6_start <= 1'b0;
            pass_clear <= 1'b0;

            case (state_c)
                ST_IDLE: begin
                    if (start) begin
                        pass_q <= 3'd0;
                        state <= ST_LAUNCH;
                    end
                end

                ST_PRECLEAR: begin
                    pass_clear <= 1'b1;
                    state <= ST_LAUNCH;
                end

                ST_LAUNCH: begin
                    grant_q <= pass_grant(pass_c);
                    swap_base_q <= pass_swap_base(pass_c);
                    p_src_q <= pass_source(pass_c);
                    p_dst_q <= pass_destination(pass_c);
                    case (pass_grant(pass_c))
                        GRANT_SWAP: swap_start <= 1'b1;
                        GRANT_P1:   p1_start <= 1'b1;
                        GRANT_P3:   p3_start <= 1'b1;
                        GRANT_P4:   p4_start <= 1'b1;
                        default:    p6_start <= 1'b1;
                    endcase
                    state <= ST_RUN;
                end

                ST_RUN: begin
                    // Wait for the pass to close (done pulses once when the
                    // engine's reader/writer finish; drain covers pipeline
                    // tails before ownership may move on).
                    if (pass_done)
                        state <= ST_DRAIN;
                end

                ST_DRAIN: begin
                    if (!pass_busy) begin
                        if (pass_c == 3'd6) begin
                            state <= ST_IDLE;
                            done <= 1'b1;
                        end else begin
                            pass_q <= next_pass(pass_c);
                            state <= ST_PRECLEAR;
                        end
                    end
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase

            if (state_c != ST_IDLE && (error || protocol_error))
                state <= ST_HOLD;

            // TMR16: heal the pass-index register while the replica is
            // being aligned; the case above does not write it every cycle.
            if (align)
                pass_q <= align_pass;
        end
    end
endmodule
