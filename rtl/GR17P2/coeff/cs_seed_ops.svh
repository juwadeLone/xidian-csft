// ============================================================================
//  cs_seed_ops.svh -- the 14 numeric operators of the K7 CS seed builder,
//  extracted verbatim from rtl/coeff/cs_seed_builder.sv (single source of
//  truth for the numeric contract of the Q32 seed tables).
//
//  Value = m * 2^e , |m| in [2^62,2^63) or m == 0  (63-bit mantissa integer
//  float).  Rounding: round-half-even where stated, truncating div/sqrt with a
//  trailing RHE bit.  DO NOT "improve" these -- the host golden model and the
//  frozen sim/vectors/*.mem depend on them bit for bit.
// ============================================================================

// count32 constant: K = round(2^64 / 2*pi)
localparam logic [63:0] K_CONST = 64'd2935890503282001226;

    function automatic logic signed [127:0] rhe_pow2(input logic signed [127:0] num,
                                                     input integer k);
        logic signed [127:0] q;
        logic [127:0] fm;
        logic rb, st;
        begin
            if (k <= 0) rhe_pow2 = num;
            else if (k >= 128) rhe_pow2 = 128'sd0;  // |num|/2^k < 0.5 : RHE -> 0
            else begin
                q  = num >>> k;
                rb = num[k-1];
                fm = (128'd1 << (k - 1)) - 128'd1;  // mask of bits [k-2:0]
                st = |(num & fm);
                if (rb && (st || q[0])) q = q + 128'sd1;
                rhe_pow2 = q;
            end
        end
    endfunction
    function automatic logic signed [127:0] rhe_div(input logic signed [127:0] num,
                                                    input logic [63:0] den);
        logic [127:0] a, r, denx, t;
        logic signed [127:0] q;
        logic neg;
        begin
            neg  = num[127];
            a    = neg ? (~num + 128'd1) : num;
            q    = '0;
            r    = '0;
            denx = {64'd0, den};
            for (integer i = 127; i >= 0; i = i - 1) begin
                r = (r << 1) | {127'd0, a[i]};
                q = q << 1;
                if (r >= denx) begin
                    r = r - denx;
                    q = q | 128'd1;
                end
            end
            t = r << 1;
            if ((t > denx) || ((t == denx) && q[0])) q = q + 128'd1;
            rhe_div = neg ? -q : q;
        end
    endfunction
    function automatic logic [63:0] isqrt128(input logic [127:0] a);
        logic [67:0] rmdr, tst;
        logic [63:0] root;
        begin
            rmdr = 68'd0;
            root = 64'd0;
            for (integer i = 63; i >= 0; i = i - 1) begin
                rmdr = (rmdr << 2) | {66'd0, a[2*i +: 2]};
                tst  = {root, 2'b01};
                if (rmdr >= tst) begin
                    rmdr = rmdr - tst;
                    root = (root << 1) | 64'd1;
                end else
                    root = root << 1;
            end
            isqrt128 = root;
        end
    endfunction
    function automatic logic signed [63:0] f32_to_fx(input logic [31:0] w,
                                                     input integer S);
        logic        sgn;
        logic [7:0]  ef;
        logic [22:0] mf;
        logic [23:0] mant;
        integer      ue, k;
        logic signed [127:0] r;
        begin
            sgn  = w[31];
            ef   = w[30:23];
            mf   = w[22:0];
            if (ef == 8'd0) begin
                mant = {1'b0, mf};
                ue   = -126;
            end else begin
                mant = {1'b1, mf};
                ue   = int'(ef) - 127;
            end
            k = ue - 23 + S;
            if (k >= 0) r = ({{104{1'b0}}, mant}) <<< k;
            else        r = rhe_pow2({{104{1'b0}}, mant}, -k);
            f32_to_fx = sgn ? -$signed(r[63:0]) : $signed(r[63:0]);
        end
    endfunction
    function automatic logic [31:0] count32(input logic signed [127:0] xq,
                                            input integer S);
        logic [63:0]  xlo, xhi;
        logic signed [127:0] p;
        logic [127:0] q, fm;
        logic rb, st;
        integer sh;
        begin
            xlo = xq[63:0];
            xhi = xq[127:64];
            p = ($unsigned(xlo) * $unsigned(K_CONST))
              + (($signed(xhi) * $signed(K_CONST)) <<< 64);
            sh = S + 32;
            q  = p >> sh;
            rb = p[sh-1];
            fm = (128'd1 << (sh - 1)) - 128'd1;
            st = |(p & fm);
            if (rb && (st || q[0])) q = q + 128'd1;
            count32 = q[31:0];
        end
    endfunction
    task automatic sf_norm(input logic signed [127:0] p, input integer e0,
                           output logic signed [63:0] mo, output integer eo);
        logic [127:0] a;
        integer k, i;
        begin
            if (p == 0) begin mo = 64'sd0; eo = 0; end
            else begin
                a = p[127] ? (~p + 128'd1) : p;
                k = 0;
                for (i = 0; i < 128; i = i + 1) if (a[i]) k = i;
                if (k >= 62) begin
                    mo = rhe_pow2(p, k - 62);
                    eo = e0 + (k - 62);
                end else begin
                    mo = p <<< (62 - k);
                    eo = e0 - (62 - k);
                end
            end
        end
    endtask
    task automatic sf_mul(input logic signed [63:0] am, input integer ae,
                          input logic signed [63:0] bm, input integer be,
                          output logic signed [63:0] rm, output integer re);
        begin
            sf_norm($signed(am) * $signed(bm), ae + be, rm, re);
        end
    endtask
    task automatic sf_div(input logic signed [63:0] am, input integer ae,
                          input logic signed [63:0] bm, input integer be,
                          output logic signed [63:0] rm, output integer re);
        logic signed [127:0] num, q;
        logic [63:0] mag;
        begin
            if (bm == 0) begin rm = 64'sd0; re = 0; end
            else begin
                num = {{64{am[63]}}, am} <<< 62;
                mag = bm[63] ? (~bm + 64'd1) : bm;
                q   = rhe_div(num, mag);
                if (bm[63]) q = -q;
                sf_norm(q, ae - be - 62, rm, re);
            end
        end
    endtask
    task automatic sf_sqrt(input logic signed [63:0] am, input integer ae,
                           output logic signed [63:0] rm, output integer re);
        logic [127:0] arg;
        logic [63:0]  root;
        begin
            if (am <= 0) begin rm = 64'sd0; re = 0; end
            else begin
                if (ae & 1) begin
                    arg  = ({64'd0, $unsigned(am)}) <<< 63;
                    root = isqrt128(arg);
                    sf_norm({124'd0, root}, ((ae - 1) / 2) - 31, rm, re);
                end else begin
                    arg  = ({64'd0, $unsigned(am)}) <<< 62;
                    root = isqrt128(arg);
                    sf_norm({124'd0, root}, (ae / 2) - 31, rm, re);
                end
            end
        end
    endtask
    task automatic sf_add(input logic signed [63:0] am, input integer ae,
                          input logic signed [63:0] bm, input integer be,
                          output logic signed [63:0] rm, output integer re);
        logic signed [127:0] A, S;
        integer d;
        begin
            if (am == 0)      begin rm = bm; re = be; end
            else if (bm == 0) begin rm = am; re = ae; end
            else if (ae >= be) begin
                d = ae - be;
                if (d > 63) begin rm = am; re = ae; end
                else begin
                    A = ({{64{am[63]}}, am} <<< d) + {{64{bm[63]}}, bm};
                    sf_norm(A, be, rm, re);
                end
            end else begin
                d = be - ae;
                if (d > 63) begin rm = bm; re = be; end
                else begin
                    A = ({{64{bm[63]}}, bm} <<< d) + {{64{am[63]}}, am};
                    sf_norm(A, ae, rm, re);
                end
            end
        end
    endtask
    task automatic sf_sub(input logic signed [63:0] am, input integer ae,
                          input logic signed [63:0] bm, input integer be,
                          output logic signed [63:0] rm, output integer re);
        begin
            sf_add(am, ae, -bm, be, rm, re);
        end
    endtask
    task automatic sf_int(input integer k,
                          output logic signed [63:0] m, output integer e);
        begin
            sf_norm(k, 0, m, e);
        end
    endtask
    task automatic sf_f32(input logic [31:0] w,
                          output logic signed [63:0] m, output integer e);
        logic [23:0] mant;
        integer ue;
        begin
            if (w[30:0] == 31'd0) begin m = 64'sd0; e = 0; end
            else begin
                mant = (w[30:23] == 8'd0) ? {1'b0, w[22:0]} : {1'b1, w[22:0]};
                ue   = (w[30:23] == 8'd0) ? -126 : (int'(w[30:23]) - 127);
                sf_norm({104'd0, mant}, ue - 23, m, e);
                if (w[31]) m = -m;
            end
        end
    endtask
    task automatic sf_q64(input logic signed [63:0] m, input integer e,
                          output logic signed [127:0] q);
        integer s;
        begin
            q = {{64{m[63]}}, m};
            s = e + 64;
            if (s >= 0) q = q <<< s;
            else        q = rhe_pow2(q, -s);
        end
    endtask
