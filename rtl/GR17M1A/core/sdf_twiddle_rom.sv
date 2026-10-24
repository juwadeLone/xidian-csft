`timescale 1ns/1ps

// Twiddle width and fractional bits come from the generated header so the
// ROM contents and the RTL cannot drift apart.  Regenerate with
// scripts/gen_twiddles.py; do not hardcode the format here.
`include "sdf_twiddle12_params.vh"

// One plain synchronous ROM per stage, loaded with $readmemh from the 12-bit
// Q1.11 tables chain/rtl/mem/twiddle12_s<STAGE_ID>.mem (stage s of a
// 2048-point transform, 1024 >> s entries, word {wi, wr}).
//
// The word read at addr on a ce edge is on wr/wi until the next ce edge.
module sdf_twiddle_rom #(
    parameter integer STAGE_ID = 0,
    parameter integer DEPTH = 1024,
    parameter integer ADDR_W = (DEPTH <= 1) ? 1 : $clog2(DEPTH),
    parameter integer INVERSE = 0,
    parameter integer TW_W = `SDF_TW_W,
    parameter integer TW_FRAC = `SDF_TW_FRAC,
    parameter MEM_DIR = "chain/rtl/mem/"
) (
    input  wire                         clk,
    input  wire                         ce,
    input  wire [ADDR_W-1:0]            addr,
    output wire signed [TW_W-1:0]       wr,
    output wire signed [TW_W-1:0]       wi
);

    localparam integer WORD_W = 2 * TW_W;
    // Most-negative Q1.<TW_FRAC> value has no positive counterpart; negating
    // it saturates instead of wrapping.
    localparam signed [TW_W-1:0] TW_MIN = -(1 <<< TW_FRAC);
    localparam signed [TW_W-1:0] TW_MAX =  (1 <<< TW_FRAC) - 1;

    reg [WORD_W-1:0] rom [0:DEPTH-1];
    reg [WORD_W-1:0] word_q;
    initial begin
        case (STAGE_ID)
            0:  $readmemh({MEM_DIR,"twiddle12_s00.mem"}, rom);
            1:  $readmemh({MEM_DIR,"twiddle12_s01.mem"}, rom);
            2:  $readmemh({MEM_DIR,"twiddle12_s02.mem"}, rom);
            3:  $readmemh({MEM_DIR,"twiddle12_s03.mem"}, rom);
            4:  $readmemh({MEM_DIR,"twiddle12_s04.mem"}, rom);
            5:  $readmemh({MEM_DIR,"twiddle12_s05.mem"}, rom);
            6:  $readmemh({MEM_DIR,"twiddle12_s06.mem"}, rom);
            7:  $readmemh({MEM_DIR,"twiddle12_s07.mem"}, rom);
            8:  $readmemh({MEM_DIR,"twiddle12_s08.mem"}, rom);
            9:  $readmemh({MEM_DIR,"twiddle12_s09.mem"}, rom);
            10: $readmemh({MEM_DIR,"twiddle12_s10.mem"}, rom);
            default: $error("Unsupported STAGE_ID=%0d for DEPTH=%0d", STAGE_ID, DEPTH);
        endcase
        if (STAGE_ID <= 10 && (1024 >> STAGE_ID) != DEPTH)
            $error("sdf_twiddle_rom: STAGE_ID=%0d needs DEPTH=%0d, got %0d", STAGE_ID, 1024 >> STAGE_ID, DEPTH);
    end
    always @(posedge clk) begin
        if (ce)
            word_q <= rom[addr];
    end

    wire signed [TW_W-1:0] wi_forward = $signed(word_q[WORD_W-1:TW_W]);
    assign wr = $signed(word_q[TW_W-1:0]);
    assign wi = (INVERSE == 0) ? wi_forward :
                ((wi_forward == TW_MIN) ? TW_MAX : -wi_forward);

endmodule
