`timescale 1ns/1ps
// K7 CS seed generation, stage 2: the f64 micro-*sequence* executor.
//
// The four 2048-row seed tables are bit-exact results of a *sequence* of
// IEEE-754 binary64 operations (scripts/gen_seed_microcode.py compiles the
// reference expressions in scripts/gen_cs_seeds.py into that sequence and
// proves it reproduces the frozen .mem files field for field).  This module
// runs the sequence: a microcode ROM plus four Xilinx floating-point cores
// (add / mul / div / sqrt, binary64, round-to-nearest-even), one operation in
// flight at a time.
//
// Serialising is what makes the arithmetic reproducible: a strictly ordered
// issue/compute/capture cycle means the only rounding that ever happens is the
// one inside the four correctly-rounded Xilinx cores, so the tables this
// produces are bit-identical to the reference tables computed in float64.
// Cost is speed, and there is plenty of it: ~150 operations per row, 8192 rows
// per frame, ~0.16 s at 100 MHz, against the 35 s DDR upload that precedes it.
//
// Why double precision at all: the phase words are Q32 counts of turns, so the
// encoding step needs the pre-rounding phase to far better than half a count.
// The largest phase here is 93,624 turns; float32 carries it to 7.8e-3 turns
// (3.3e7 counts) and the 63-bit soft-float path that used to be here to
// 1.4e-14 turns, at the cost of a netlist that cannot be built.  binary64
// gives 1.46e-11 turns = 0.0625 counts, an 8x margin on the half-count rule.
//
// Microcode word (32 bit), matching scripts/gen_seed_microcode.py:
//     [2:0]   op
//     [10:3]  dst      destination register
//     [18:11] a        source register (or constant slot for op c)
//     [26:19] b        second source (or constant slot / word index)
// Ops:
//     0 c     rf[dst] <= cst[a]
//     1 add   rf[dst] <= rf[a] + rf[b]
//     2 sub   rf[dst] <= rf[a] - rf[b]
//     3 mul   rf[dst] <= rf[a] * rf[b]
//     4 div   rf[dst] <= rf[a] / rf[b]
//     5 sqrt  rf[dst] <= sqrt(rf[a])
//     6 row   rf[dst] <= (row_idx - 1024) as binary64   (exact: |value| <= 1024)
//     7 fin   hand rf[a][31:0] out as table word b      (0 = theta0, 1..2 = deltas)
//     8 csts  cst[b] <= rf[a]                           (runtime constant build)
//
// The parameter registers rf[0..7] are C, lam, B, Fs, Tp, v, Da, Rs, loaded
// through the par_* port before the program starts.  The setup program reads
// them and parks the derived values in the constant pool, so the four row
// programs only ever touch constants.

module k7_f64_seq_core #(
    parameter integer N_REG = 256,
    parameter integer N_CST = 35,
    parameter integer ROM_WORDS = 1024
)(
    input  wire         clk,
    input  wire         rst,
    input  wire         run,
    input  wire [9:0]   prog_off,
    input  wire [7:0]   prog_len,
    input  wire [11:0]  row_idx,
    input  wire [7:0]   par_addr,
    input  wire [63:0]  par_data,
    input  wire         par_we,
    output reg          busy,
    output reg          done,
    output reg          fin_we,
    output reg  [1:0]   fin_word,
    output reg  [31:0]  fin_data,
    // combinational debug read port (host observability via CS_RB flags[3])
    input  wire [7:0]   dbg_addr,
    output wire [63:0]  dbg_data
);

    localparam [3:0] OP_C    = 4'd0;
    localparam [3:0] OP_ADD  = 4'd1;
    localparam [3:0] OP_SUB  = 4'd2;
    localparam [3:0] OP_MUL  = 4'd3;
    localparam [3:0] OP_DIV  = 4'd4;
    localparam [3:0] OP_SQRT = 4'd5;
    localparam [3:0] OP_ROW  = 4'd6;
    localparam [3:0] OP_FIN  = 4'd7;
    localparam [3:0] OP_CSTS = 4'd8;

    localparam [2:0] S_IDLE = 3'd0;
    localparam [2:0] S_FETCH= 3'd1;
    localparam [2:0] S_DEC  = 3'd2;
    localparam [2:0] S_ISSUE= 3'd3;
    localparam [2:0] S_WAIT = 3'd4;
    localparam [2:0] S_END  = 3'd5;
    localparam [2:0] S_RD   = 3'd6;

    // ---------------------------------------------------------------- ROM
    (* rom_style = "block" *) reg [31:0] rom [0:ROM_WORDS-1];
    initial $readmemh("f64mc_rom.mem", rom);

    reg  [9:0]  raddr;
    reg  [31:0] rom_q;

    // Synchronous read = TWO cycles: address at S_FETCH, data latched at S_RD,
    // decoded at S_DEC.  (Was decoded at S_DEC one cycle too early -> every op
    // ran the *previous* word and the last word of each program was dropped.)
    always @(posedge clk) rom_q <= rom[raddr];

    // ------------------------------------------------------- constant pool
    reg [63:0] cst [0:N_CST-1];
    initial $readmemh("f64mc_consts.mem", cst);

    // ---------------------------------------------------- register file
    reg [63:0] rf [0:N_REG-1];

    reg         rf_we;
    reg  [7:0]  rf_waddr;
    reg  [63:0] rf_wdata;

    // ------------------------------------------------------------ state
    reg  [2:0]  st;
    reg  [9:0]  pc;
    reg  [9:0]  prog_end;
    reg  [3:0]  ip_op;
    reg  [7:0]  ip_dst;
    reg  [63:0] ip_a;
    reg  [63:0] ip_b;

    wire [3:0]  uop   = rom_q[3:0];
    wire [7:0]  u_dst = rom_q[11:4];
    wire [7:0]  u_a   = rom_q[19:12];
    wire [7:0]  u_b   = rom_q[27:20];

    wire [63:0] cst_r = cst[u_a[5:0]];
    wire [63:0] rf_a  = rf[u_a];
    wire [63:0] rf_b  = rf[u_b];

    wire last_op = (pc == prog_end);

    // (row_idx - 1024) as binary64.  The magnitude is <= 1024, so the value is
    // exactly representable: 11 significant bits at most.
    function [63:0] i2f_row;
        input [11:0] idx;
        reg   [11:0] d;
        reg   [11:0] mag;
        reg          sgn;
        reg   [4:0]  e;
        reg   [63:0] sh;
        integer      i;
        begin
            d   = idx - 12'd1024;
            sgn = d[11];
            mag = sgn ? ((~d) + 12'd1) : d;
            if (mag == 12'd0) begin
                i2f_row = 64'd0;
            end else begin
                e = 5'd0;
                for (i = 0; i <= 11; i = i + 1) begin
                    if (mag[i]) e = i[4:0];
                end
                sh = ({52'd0, mag} << (6'd52 - {1'b0, e}));
                i2f_row = {sgn, (11'd1023 + {6'd0, e}), sh[51:0]};
            end
        end
    endfunction

    wire [63:0] row_f64 = i2f_row(row_idx);

    // dbg_addr 0..63 -> rf[addr], 64..95 -> cst[addr-64], 96 -> core state
    wire [63:0] dbg_state = {6'd0, pc, prog_end, {2'b0, prog_len}, {5'd0, st},
                             8'd0, {31'd0, busy}};
    assign dbg_data = (dbg_addr < 8'd64)  ? rf[dbg_addr]
                    : (dbg_addr < 8'd96)  ? cst[dbg_addr - 8'd64]
                    : (dbg_addr == 8'd96) ? dbg_state
                    : 64'hDEAD_0000_0000_0000 | dbg_addr;

    // ------------------------------------------------------- AXI4-Stream IP
    reg         ip_tvalid;
    reg  [63:0] ip_add_b;
    reg  [7:0]  ip_operation;
    reg  [3:0]  ip_op_q;
    reg  [7:0]  ip_dst_q;

    wire ip_is_add = (ip_op == OP_ADD) || (ip_op == OP_SUB);
    wire ip_is_mul = (ip_op == OP_MUL);
    wire ip_is_div = (ip_op == OP_DIV);
    wire ip_is_sq  = (ip_op == OP_SQRT);

    wire add_a_tready, add_b_tready, add_o_tready, add_rvalid;
    wire mul_a_tready, mul_b_tready, mul_rvalid;
    wire div_a_tready, div_b_tready, div_rvalid;
    wire sq_a_tready, sq_rvalid;
    wire [63:0] add_rdata, mul_rdata, div_rdata, sq_rdata;

    wire issue_done = ip_is_add ? (add_a_tready & add_b_tready & add_o_tready)
                    : ip_is_mul ? (mul_a_tready & mul_b_tready)
                    : ip_is_div ? (div_a_tready & div_b_tready)
                                : sq_a_tready;

    wire res_valid = (ip_op_q == OP_ADD || ip_op_q == OP_SUB) ? add_rvalid
                   : (ip_op_q == OP_MUL) ? mul_rvalid
                   : (ip_op_q == OP_DIV) ? div_rvalid : sq_rvalid;

    wire [63:0] res_data = (ip_op_q == OP_ADD || ip_op_q == OP_SUB) ? add_rdata
                         : (ip_op_q == OP_MUL) ? mul_rdata
                         : (ip_op_q == OP_DIV) ? div_rdata : sq_rdata;

    fp64_add u_add (
        .aclk(clk),
        .s_axis_a_tvalid(ip_tvalid & ip_is_add),
        .s_axis_a_tready(add_a_tready),
        .s_axis_a_tdata(ip_a),
        .s_axis_b_tvalid(ip_tvalid & ip_is_add),
        .s_axis_b_tready(add_b_tready),
        .s_axis_b_tdata(ip_add_b),
        .s_axis_operation_tvalid(ip_tvalid & ip_is_add),
        .s_axis_operation_tready(add_o_tready),
        .s_axis_operation_tdata(ip_operation),
        .m_axis_result_tvalid(add_rvalid),
        .m_axis_result_tready(1'b1),
        .m_axis_result_tdata(add_rdata)
    );

    fp64_mul u_mul (
        .aclk(clk),
        .s_axis_a_tvalid(ip_tvalid & ip_is_mul),
        .s_axis_a_tready(mul_a_tready),
        .s_axis_a_tdata(ip_a),
        .s_axis_b_tvalid(ip_tvalid & ip_is_mul),
        .s_axis_b_tready(mul_b_tready),
        .s_axis_b_tdata(ip_b),
        .m_axis_result_tvalid(mul_rvalid),
        .m_axis_result_tready(1'b1),
        .m_axis_result_tdata(mul_rdata)
    );

    fp64_div u_div (
        .aclk(clk),
        .s_axis_a_tvalid(ip_tvalid & ip_is_div),
        .s_axis_a_tready(div_a_tready),
        .s_axis_a_tdata(ip_a),
        .s_axis_b_tvalid(ip_tvalid & ip_is_div),
        .s_axis_b_tready(div_b_tready),
        .s_axis_b_tdata(ip_b),
        .m_axis_result_tvalid(div_rvalid),
        .m_axis_result_tready(1'b1),
        .m_axis_result_tdata(div_rdata)
    );

    fp64_sqrt u_sqrt (
        .aclk(clk),
        .s_axis_a_tvalid(ip_tvalid & ip_is_sq),
        .s_axis_a_tready(sq_a_tready),
        .s_axis_a_tdata(ip_a),
        .m_axis_result_tvalid(sq_rvalid),
        .m_axis_result_tready(1'b1),
        .m_axis_result_tdata(sq_rdata)
    );

    // ------------------------------------------------------------- control
    always @(posedge clk) begin
        if (rst) begin
            st       <= S_IDLE;
            busy     <= 1'b0;
            done     <= 1'b0;
            fin_we   <= 1'b0;
            fin_word <= 2'd0;
            fin_data <= 32'd0;
            rf_we    <= 1'b0;
            rf_waddr <= 8'd0;
            rf_wdata <= 64'd0;
            ip_tvalid<= 1'b0;
            ip_operation <= 8'd0;
            raddr    <= 10'd0;
        end else begin
            done   <= 1'b0;
            fin_we <= 1'b0;
            rf_we  <= 1'b0;

            case (st)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (run) begin
                        busy     <= 1'b1;
                        pc       <= prog_off;
                        prog_end <= prog_off + {2'b0, prog_len} - 10'd1;
                        st       <= S_FETCH;
                    end
                end

                // ROM has a synchronous read port: address now, data next.
                // S_FETCH: address; S_RD: rom_q latches mem[addr]; S_DEC: decode.
                S_FETCH: begin
                    raddr <= pc;
                    st    <= S_RD;
                end

                S_RD: st <= S_DEC;

                S_DEC: begin
                    if ($test$plusargs("TRACE"))
                        $display("TRACE pc=%0d uop=%0d dst=%0d a=%0d b=%0d rfa=%016h rfb=%016h",
                                 pc, uop, u_dst, u_a, u_b, rf_a, rf_b);
                    case (uop)
                        OP_C: begin
                            rf_we    <= 1'b1;
                            rf_waddr <= u_dst;
                            rf_wdata <= cst_r;
                            st       <= last_op ? S_END : S_FETCH;
                            pc       <= pc + 10'd1;
                        end

                        OP_ROW: begin
                            rf_we    <= 1'b1;
                            rf_waddr <= u_dst;
                            rf_wdata <= row_f64;
                            st       <= last_op ? S_END : S_FETCH;
                            pc       <= pc + 10'd1;
                        end

                        OP_FIN: begin
                            fin_we   <= 1'b1;
                            fin_word <= u_b[1:0];
                            fin_data <= rf_a[31:0];
                            st       <= last_op ? S_END : S_FETCH;
                            pc       <= pc + 10'd1;
                        end

                        OP_CSTS: begin
                            cst[u_b[5:0]] <= rf_a;
                            st            <= last_op ? S_END : S_FETCH;
                            pc            <= pc + 10'd1;
                        end

                        default: begin
                            // arithmetic: latch operands, hand over to the IPs
                            ip_a     <= rf_a;
                            // subtract is a sign flip on the second operand: the
                            // adder is then only ever asked to add, so the
                            // result never depends on the IP's add/sub opcode
                            // encoding, and a-x is still correctly rounded.
                            ip_add_b <= (uop == OP_SUB) ? {~rf_b[63], rf_b[62:0]}
                                                        : rf_b;
                            ip_b     <= rf_b;
                            ip_op    <= uop;
                            ip_dst   <= u_dst;
                            ip_tvalid<= 1'b1;
                            ip_operation <= 8'd0;
                            st       <= S_ISSUE;
                        end
                    endcase
                end

                S_ISSUE: begin
                    if (issue_done) begin
                        ip_tvalid <= 1'b0;
                        ip_op_q   <= ip_op;
                        ip_dst_q  <= ip_dst;
                        st        <= S_WAIT;
                    end
                end

                S_WAIT: begin
                    if (res_valid) begin
                        rf_we    <= 1'b1;
                        rf_waddr <= ip_dst_q;
                        rf_wdata <= res_data;
                        st       <= last_op ? S_END : S_FETCH;
                        pc       <= pc + 10'd1;
                    end
                end

                S_END: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    st   <= S_IDLE;
                end

                default: st <= S_IDLE;
            endcase
        end
    end

    // Parity of the row: the last operation of a program is `fin` for the three
    // word program (word 2), so `last_op` above is evaluated with the pc of the
    // operation being decoded.  Fine for both combinational and IP ops because
    // the pc is only advanced when the operation is retired.

    always @(posedge clk) begin
        if (par_we) begin
            rf[par_addr] <= par_data;
        end else if (rf_we) begin
            rf[rf_waddr] <= rf_wdata;
        end
    end

endmodule
