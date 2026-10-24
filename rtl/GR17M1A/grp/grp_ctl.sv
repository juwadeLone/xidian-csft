`timescale 1ns/1ps

// One control copy of a grouped GR16S stage (GR16S control-estimation task,
// tasks/20261019-gr16s-ctl-est).  This is the control part of sdf_stage_grp
// pulled into its own module so it can be instantiated once (V1) or three
// times (V2 majority-voted shared control, V3 one copy per data domain).
// The datapath (sdf_stage_dp) consumes the bundle this module produces.
//
// Control state per copy (identical semantics to sdf_stage_grp):
//   phase / active_q / local_ce, first_half_d, addr_d, ctrl_valid_d,
//   the twiddle-mode flag registers (do_tw, unity, quarter), and the
//   valid/SOF/EOF timing chain.
//
// Timing chain implementation (parameter USE_CNT):
//   * USE_CNT == 0 (stages whose DEPTH < 32): the original three
//     DEPTH+1-bit shift chains + registered *_tw_d taps, plus a 4-deep
//     tick pipe so this copy also yields o_valid/o_sof/o_eof aligned with
//     the datapath rotator output.
//   * USE_CNT == 1 (stages 0..5, DEPTH >= 32, per D352 section 2):
//     L_s = DEPTH + 6 total stage latency.  Two down-counters
//     (sof_cnt / eof_cnt, ceil(log2 L_s) <= 11 bits) + one in_frame flag:
//       - i_sof loads sof_cnt = L_s-1; the registered emit pulse fires when
//         the counter leaves 1 (i.e. the o_sof beat); in_frame sets.
//       - i_eof loads eof_cnt = L_s-1; emit pulse at the same offset;
//         in_frame clears.
//     Level-j valid ("the beat now at level j is valid", j measured as the
//     number of ticks remaining before the output) is
//         VJ(j) = (0 < sof_cnt <= j)
//               | (in_frame & ~(0 < eof_cnt <= j))
//               | (eof_cnt == j)
//     The three terms cover: a head still inside this stage that reaches the
//     output within j ticks; an already-emitted frame whose tail has not
//     passed level j; and the tail beat itself sitting at level j.
//     Taps used by the stage:
//       rep_i_valid = VJ(5)         (was valid_delay[DEPTH], level DEPTH+1)
//       valid_tw_d  = VJ(4)         (was the valid_tw_d register, DEPTH+2)
//       sof_tw_d    = sof_cnt == 4  (SOF event at level DEPTH+2)
//       eof_tw_d    = eof_cnt == 4
//       o_sof/o_eof = emit pulses;  o_valid = in_frame | emits
//     Back-to-back frames work because at most one SOF and one EOF are in
//     flight in the chain (D352); counters reload on the next tick.
//
// VOTE_IN: 1 -> this copy computes its own majority of the three input tick
// copies (sdf_tk style, used by V2/V3); 0 -> it takes i_*3[0] verbatim (used
// by V3 per-domain chains, which wire a single-domain tick to all three
// pins).
module grp_ctl #(
    parameter integer LOGN = 11,
    parameter integer STAGE = 0,
    parameter integer VOTE_IN = 1,
    parameter integer USE_CNT = -1   // -1: auto (DEPTH >= 32)
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        clear,        // this copy's stage_clear
    input  wire        ce,           // this copy's engine_ce
    input  wire [2:0]  i_valid3,
    input  wire [2:0]  i_sof3,
    input  wire [2:0]  i_eof3,
    output wire        local_ce,
    output wire [10:0] phase_addr,   // zero-extended ADDR_W
    output wire        first_half_d,
    output wire [10:0] addr_d,       // zero-extended ADDR_W
    output wire        ctrl_valid_d,
    output wire        do_tw,        // do_twiddle_tw_d
    output wire        unity,        // unity_tw_d
    output wire        quarter,      // quarter_tw_d
    output wire        valid_tw_d,
    output wire        sof_tw_d,
    output wire        eof_tw_d,
    output wire        rep_i_valid,  // level DEPTH+1 token for grp_rep_dom
    output wire        o_valid,
    output wire        o_sof,
    output wire        o_eof
);
    localparam integer DEPTH = 1 << (LOGN-STAGE-1);
    localparam integer ADDR_W = (DEPTH <= 1) ? 1 : $clog2(DEPTH);
    localparam integer PHASE_W = LOGN-STAGE;
    localparam integer LS = DEPTH + 6;
    localparam integer CW = (LS <= 2) ? 1 : $clog2(LS);
    localparam integer CNT = (USE_CNT < 0) ? (DEPTH >= 32) : USE_CNT;

    // ---- input ticks -------------------------------------------------------
    wire i_valid = VOTE_IN ? ((i_valid3[0] & i_valid3[1]) |
                              (i_valid3[0] & i_valid3[2]) |
                              (i_valid3[1] & i_valid3[2])) : i_valid3[0];
    wire i_sof   = VOTE_IN ? ((i_sof3[0] & i_sof3[1]) |
                              (i_sof3[0] & i_sof3[2]) |
                              (i_sof3[1] & i_sof3[2])) : i_sof3[0];
    wire i_eof   = VOTE_IN ? ((i_eof3[0] & i_eof3[1]) |
                              (i_eof3[0] & i_eof3[2]) |
                              (i_eof3[1] & i_eof3[2])) : i_eof3[0];

    // ---- shared control state (same registers as sdf_stage_grp) ------------
    reg [PHASE_W-1:0] phase;
    reg active_q;
    assign local_ce = ce && (active_q || i_valid);
    wire first_half = !phase[PHASE_W-1];
    wire [ADDR_W-1:0] phase_a = (DEPTH <= 1) ? {ADDR_W{1'b0}}
                                             : phase[ADDR_W-1:0];
    reg fh_d;
    reg [ADDR_W-1:0] a_d;
    reg cv_d;

    // ---- tick engine -------------------------------------------------------
    wire v_tw, s_tw, e_tw, rep_v, c_ov, c_os, c_oe;
    generate
        if (CNT) begin : g_cnt
            // two down-counters + in-frame flag (D352 section 2)
            reg [CW-1:0] sof_cnt, eof_cnt;
            reg in_frame;
            reg sof_em, eof_em;
            wire sof_load = i_sof;
            wire eof_load = i_eof;
            always @(posedge clk) begin
                if (rst || clear) begin
                    sof_cnt  <= '0;
                    eof_cnt  <= '0;
                    in_frame <= 1'b0;
                    sof_em   <= 1'b0;
                    eof_em   <= 1'b0;
                end else if (local_ce) begin
                    sof_em <= (sof_cnt == {{(CW-1){1'b0}}, 1'b1});
                    eof_em <= (eof_cnt == {{(CW-1){1'b0}}, 1'b1});
                    if (sof_load)
                        sof_cnt <= LS - 1;
                    else if (sof_cnt != '0)
                        sof_cnt <= sof_cnt - {{(CW-1){1'b0}}, 1'b1};
                    if (eof_load)
                        eof_cnt <= LS - 1;
                    else if (eof_cnt != '0)
                        eof_cnt <= eof_cnt - {{(CW-1){1'b0}}, 1'b1};
                    if (sof_cnt == {{(CW-1){1'b0}}, 1'b1})
                        in_frame <= 1'b1;
                    else if (eof_cnt == {{(CW-1){1'b0}}, 1'b1})
                        in_frame <= 1'b0;
                end
            end

            // VJ(j): level-j valid / SOF / EOF at j ticks before the output
            function automatic vj;
                input integer j;
                input [CW-1:0] sc, ec;
                input        inf;
                begin
                    vj = ((sc != '0) && (sc <= j)) |
                         (inf && ~((ec != '0) && (ec <= j))) |
                         (ec == j);
                end
            endfunction
            assign v_tw  = vj(4, sof_cnt, eof_cnt, in_frame);
            assign s_tw  = (sof_cnt == 4);
            assign e_tw  = (eof_cnt == 4);
            assign rep_v = vj(5, sof_cnt, eof_cnt, in_frame);
            assign c_os  = sof_em;
            assign c_oe  = eof_em;
            assign c_ov  = in_frame | sof_em | eof_em;
        end else begin : g_chain
            // the original DEPTH+1-bit chains + registered tw taps + a
            // 4-deep pipe so this copy reports o_* at datapath timing
            reg [DEPTH:0] vd, sd, ed;
            reg vt, st, et;
            reg [3:0] vp, sp, ep;
            always @(posedge clk) begin
                if (rst || clear) begin
                    vd <= '0;
                    sd <= '0;
                    ed <= '0;
                    vt <= 1'b0;
                    st <= 1'b0;
                    et <= 1'b0;
                    vp <= 4'b0;
                    sp <= 4'b0;
                    ep <= 4'b0;
                end else if (local_ce) begin
                    vd <= {vd[DEPTH-1:0], i_valid};
                    sd <= {sd[DEPTH-1:0], i_sof};
                    ed <= {ed[DEPTH-1:0], i_eof};
                    vt <= vd[DEPTH];
                    st <= sd[DEPTH];
                    et <= ed[DEPTH];
                    vp <= {vp[2:0], vt};
                    sp <= {sp[2:0], st};
                    ep <= {ep[2:0], et};
                end
            end
            assign v_tw  = vt;
            assign s_tw  = st;
            assign e_tw  = et;
            assign rep_v = vd[DEPTH];
            assign c_ov  = vp[3];
            assign c_os  = sp[3];
            assign c_oe  = ep[3];
        end
    endgenerate

    assign valid_tw_d  = v_tw;
    assign sof_tw_d    = s_tw;
    assign eof_tw_d    = e_tw;
    assign rep_i_valid = rep_v;
    assign o_valid     = c_ov;
    assign o_sof       = c_os;
    assign o_eof       = c_oe;

    // ---- twiddle-mode flag registers + control registers -------------------
    reg dt, un, qt;
    always @(posedge clk) begin
        if (rst || clear) begin
            phase     <= '0;
            active_q  <= 1'b0;
            fh_d      <= 1'b0;
            a_d       <= '0;
            cv_d      <= 1'b0;
            dt        <= 1'b0;
            un        <= 1'b0;
            qt        <= 1'b0;
        end else if (local_ce) begin
            active_q <= 1'b1;
            phase    <= phase + {{(PHASE_W-1){1'b0}}, 1'b1};
            fh_d     <= first_half;
            a_d      <= phase_a;
            cv_d     <= 1'b1;
            dt       <= fh_d;
            un       <= (a_d == {ADDR_W{1'b0}});
            qt       <= (DEPTH > 2) && (a_d == (DEPTH >> 1));
        end
    end

    assign first_half_d = fh_d;
    assign ctrl_valid_d = cv_d;
    assign do_tw        = dt;
    assign unity        = un;
    assign quarter      = qt;
    assign phase_addr   = {{(11-ADDR_W){1'b0}}, phase_a};
    assign addr_d       = {{(11-ADDR_W){1'b0}}, a_d};
endmodule
