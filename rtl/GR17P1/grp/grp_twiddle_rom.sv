`timescale 1ns/1ps

// Replica twiddle ROM for one grouped GR17 stage: the stage's OWN table at
// KT bits (spec17.json kt vector), not the main path's 25-bit table.
// Entries are gr17_model.rep_twiddles(block, KT, forward) = cos/sin at the
// same angles as the main table, rounded to nearest at Q1.(KT-1), packed
// {wi, wr} in a 2*KT-bit word, one entry per second-half beat (block/2).
//
// MEM_BASE selects the transform's table family; the file is
// "<MEM_BASE><two-digit STAGE_ID>.mem" (e.g. grp17_twtab_p1_s + "03.mem").
// STAGE_ID is the transform stage (0..10); DEPTH must equal block/2.
// INVERSE negates the sin half (saturating at the most-negative value),
// which is bit-exact against rep_twiddles(block, KT, inverse).
//
// The word read at addr on a ce edge is on wr/wi until the next ce edge,
// the same timing as sdf_twiddle_rom.
module grp_twiddle_rom #(
    parameter integer STAGE_ID = 0,
    parameter integer DEPTH = 1024,
    parameter integer ADDR_W = (DEPTH <= 1) ? 1 : $clog2(DEPTH),
    parameter integer INVERSE = 0,
    parameter integer KT = 13,
    parameter MEM_BASE = "chain/rtl/mem/grp17_twtab_p1_s"
) (
    input  wire                         clk,
    input  wire                         ce,
    input  wire [ADDR_W-1:0]            addr,
    output wire signed [KT-1:0]         wr,
    output wire signed [KT-1:0]         wi
);
    localparam integer WORD_W = 2 * KT;
    localparam signed [KT-1:0] TW_MIN = -(1 <<< (KT-1));
    localparam signed [KT-1:0] TW_MAX =  (1 <<< (KT-1)) - 1;

    reg [WORD_W-1:0] rom [0:DEPTH-1];
    reg [WORD_W-1:0] word_q;
    initial begin
        case (STAGE_ID)
            0:  $readmemh({MEM_BASE, "00.mem"}, rom);
            1:  $readmemh({MEM_BASE, "01.mem"}, rom);
            2:  $readmemh({MEM_BASE, "02.mem"}, rom);
            3:  $readmemh({MEM_BASE, "03.mem"}, rom);
            4:  $readmemh({MEM_BASE, "04.mem"}, rom);
            5:  $readmemh({MEM_BASE, "05.mem"}, rom);
            6:  $readmemh({MEM_BASE, "06.mem"}, rom);
            7:  $readmemh({MEM_BASE, "07.mem"}, rom);
            8:  $readmemh({MEM_BASE, "08.mem"}, rom);
            9:  $readmemh({MEM_BASE, "09.mem"}, rom);
            10: $readmemh({MEM_BASE, "10.mem"}, rom);
            default: $error("Unsupported STAGE_ID=%0d for DEPTH=%0d", STAGE_ID, DEPTH);
        endcase
        if (STAGE_ID <= 10 && (1024 >> STAGE_ID) != DEPTH)
            $error("grp_twiddle_rom: STAGE_ID=%0d needs DEPTH=%0d, got %0d",
                   STAGE_ID, 1024 >> STAGE_ID, DEPTH);
    end
    always @(posedge clk) begin
        if (ce)
            word_q <= rom[addr];
    end

    wire signed [KT-1:0] wi_forward = $signed(word_q[WORD_W-1:KT]);
    assign wr = $signed(word_q[KT-1:0]);
    assign wi = (INVERSE == 0) ? wi_forward :
                ((wi_forward == TW_MIN) ? TW_MAX : -wi_forward);
endmodule
