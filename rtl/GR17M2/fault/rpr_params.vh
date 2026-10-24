// Replica structure of an RPR rotator node, derived from its compare width K
// (rpr_cmp D3 block 3).  One place, so the replica, the voter tolerance and the
// test benches cannot disagree.
//
// The FP rotator computes y = floor(pre * w / 2^(17+SC)) (32 bits), with
// pre = feedback + x (33 bits) and w the Q1.17 twiddle.  The voter compares the
// top K bits of y.  A replica truncates feedback and x by SP = 32 - K + SC bits
// (so its sum has KA = 34 - SP bits), truncates w to KT bits, multiplies, and
// shifts by KT - 1.  Against the exact top(y), its error per component is below
//     2*sqrt(2)                     (the two truncated addends, |w| <= 1)
//   + 2^(2 + K - KT - SC)           (twiddle truncation, both products; 0 if KT = 18)
//   + 2^(2 - KT)                    (packing: one borrow per high product)
// so fault-free |top(y) - r| <= T = ceil(that bound).
//
// Structure (DSP48E1: 25 x 18 multiplier, 25-bit A/D pre-adder):
//   KA <= 8        MODE 0  packed, 2 DSP per replica, KT = min(18, 24 - 2*KA)
//   KA <= 17       MODE 1  3 DSP (A/D pre-adder), KT = 17 (twiddle sums in LUT
//                          must fit the 18-bit B port)
//   KA <= 25       MODE 2  4 DSP, KT = 18: the exact twiddle.  A 17-bit twiddle
//                          alone errs by 2^15 in y, too coarse for nodes whose
//                          critical bit is below ~17 (K >= 17).
// Include INSIDE a module body (the functions are module-scoped); no include
// guard for that reason.

function automatic integer rpr_ka(input integer k, input integer sc);
    rpr_ka = 34 - (32 - k + sc);
endfunction

function automatic integer rpr_mode(input integer k, input integer sc);
    integer ka;
    begin
        ka = rpr_ka(k, sc);
        rpr_mode = (ka <= 8) ? 0 : (ka <= 17) ? 1 : 2;
    end
endfunction

function automatic integer rpr_kt(input integer k, input integer sc);
    integer ka, m;
    begin
        ka = rpr_ka(k, sc);
        m = rpr_mode(k, sc);
        rpr_kt = (m == 0) ? ((24 - 2 * ka > 18) ? 18 : 24 - 2 * ka) : (m == 1) ? 17 : 18;
    end
endfunction

// 1e4 * 2^e, rounded up, for integer e in [-20, 20]
function automatic integer rpr_p2e4(input integer e);
    rpr_p2e4 = (e >= 0) ? (10000 << e) : ((10000 >> (-e)) + 1);
endfunction

function automatic integer rpr_t(input integer k, input integer sc);
    integer kt, b;
    begin
        kt = rpr_kt(k, sc);
        b = 28285;                                            // 2*sqrt(2), rounded up
        if (kt < 18) b = b + rpr_p2e4(2 + k - kt - sc);
        if (rpr_mode(k, sc) == 0) b = b + rpr_p2e4(2 - kt);
        rpr_t = (b + 9999) / 10000;
    end
endfunction


// ---- compensator complex product: y = floor(x * c / 2^29), x 32-bit data,
// c the Q1.29 coefficient (30 bits), no scaling.  The replica takes the top K
// bits of x (floor), c truncated to KT bits, and shifts by KT - 1.  Error per
// component below  sqrt(2) + 2^(1 + K - KT)  (+ packing borrows).
//   K <= 7    MODE 0 packed 2 DSP, KT = min(18, 24 - 2K)
//   K <= 16   MODE 1 3 DSP, KT = 17
//   K <= 18   MODE 2 4 DSP, KT = 18
//   K <= 24   MODE 3 6 DSP (3 products, coefficient side split hi/lo),
//             KT = K + 1: an 18-bit coefficient errs by 2^(K-17), too much
function automatic integer rpc_mode(input integer k);
    rpc_mode = (k <= 7) ? 0 : (k <= 16) ? 1 : (k <= 18) ? 2 : 3;
endfunction

function automatic integer rpc_kt(input integer k);
    integer m;
    begin
        m = rpc_mode(k);
        rpc_kt = (m == 0) ? ((24 - 2 * k > 18) ? 18 : 24 - 2 * k) : (m == 1) ? 17 :
                 (m == 2) ? 18 : k + 1;
    end
endfunction

function automatic integer rpc_t(input integer k);
    integer kt, b;
    begin
        kt = rpc_kt(k);
        b = 14143 + rpr_p2e4(1 + k - kt);                     // sqrt(2), rounded up
        if (rpc_mode(k) == 0) b = b + rpr_p2e4(2 - kt);
        rpc_t = (b + 9999) / 10000;
    end
endfunction

// ---- roll compensators (coefficient +-1: the CORDIC gives (2^29-1, -2) at
// phase 0 and (-2^29, 2) at pi): the replica is +-x in LUT, no multiplier.
// |y - (+-x)| <= 12 LSB, so fault-free |top(y) - r| <= 1 when 32 - K >= 4.
function automatic integer rpr_roll_t(input integer k);
    rpr_roll_t = 2;
endfunction

// ---- window (real coefficient w, Q1.29, on both components): replica top K
// of x, w truncated to KW bits, shift KW - 1; error below 1 + 2^(K - KW).
//   K <= 7    packed, 1 DSP (both components share w), KW = min(18, 24 - 2K)
//   K <= 18   2 DSP, KW = 18
//   K <= 24   4 DSP (w split hi/lo), KW = K + 1
function automatic integer rpw_mode(input integer k);
    rpw_mode = (k <= 7) ? 0 : (k <= 18) ? 1 : 2;
endfunction

function automatic integer rpw_kw(input integer k);
    integer m;
    begin
        m = rpw_mode(k);
        rpw_kw = (m == 0) ? ((24 - 2 * k > 18) ? 18 : 24 - 2 * k) : (m == 1) ? 18 : k + 1;
    end
endfunction

function automatic integer rpw_t(input integer k);
    integer b;
    begin
        b = 10000 + rpr_p2e4(k - rpw_kw(k));
        if (rpw_mode(k) == 0) b = b + rpr_p2e4(1 - rpw_kw(k));
        rpw_t = (b + 9999) / 10000;
    end
endfunction

// ---- CORDIC coefficient (Q1.29, 30 bits): table replica, 12-bit phase into
// a quarter-wave table of 1024 x 16 bits (bin centres).  Phase quantisation
// errs by at most pi * 2^(K-13) of the top-K unit; T = 2 for K <= 11.
function automatic integer rpr_coef_t(input integer k);
    rpr_coef_t = (k <= 11) ? 2 : 3;
endfunction
