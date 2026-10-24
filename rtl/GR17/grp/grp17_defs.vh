// GR17 grouped-RPR arithmetic parameters for the 17-bit full-range chain
// (W = 17, main-path twiddle/coefficient 25-bit Q1.24, 18-bit butterfly
// extension), derived against model/gr17/spec17.json.  The same formulas as
// GR16S's grp16s_defs.vh with the W17 constants substituted; the functions
// live in module scope, so every includer needs its own copy.
//
// Stage replica, width K, scale bit SC of the stage (K < 17 always):
//   SP = 17 - K + SC        operand bits dropped (trunc)
//   TW = 18 - SP            operand bits kept of the 18-bit butterfly ext
//   KA = TW + 1             butterfly-sum width
//   KT = this stage's own replica twiddle width (spec17.json kt vector)
//   r  = sat_K((num + 2^(KT-2)) >>> (KT-1))       (gr17_model.rep_stage)
// Reference-node product, width K:
//   KW = K                  top K bits of the 17-bit word (floor(x/2^(17-K)))
//   KC = this product's own coefficient width (spec17.json products)
//   r  = sat_K((x_t*c + 2^(KC-2)) >>> (KC-1))     (gr17_model.rep_product)
//
// All replica multiplies are LUT-only (rp_cmul_lut).  The gr17_model
// computes exact products, so MODE 0's packed form (the uncorrected
// borrow, which GR16S counted in T) is never selected: mode 1 (exact,
// 3 multiplies) below the operand bound, else mode 2 (exact, 4).

function automatic integer grp_sp17(input integer k, input integer sc);
    grp_sp17 = 17 - k + sc;
endfunction

function automatic integer grp_tw17(input integer k, input integer sc);
    grp_tw17 = 18 - grp_sp17(k, sc);
endfunction

function automatic integer grp_ka17(input integer k, input integer sc);
    grp_ka17 = grp_tw17(k, sc) + 1;
endfunction

function automatic integer grp_mode17(input integer ka, input integer kt);
    grp_mode17 = (ka <= 24) ? 1 : 2;
endfunction

function automatic integer grp_pkw17(input integer k);
    grp_pkw17 = (k >= 17) ? 17 : k;
endfunction

function automatic integer grp_pmode17(input integer k, input integer kc);
    integer ka;
    begin
        ka = grp_pkw17(k) + 1;
        grp_pmode17 = (ka <= 24) ? 1 : 2;
    end
endfunction
