`timescale 1ns / 1ps
`include "udp_cmd_hdr.vh"

// Encode / decode the 16-byte UDP command header.
// Unpack path and pack path are independent (combinational).
// MAX_BYTES is a property of the board's memory, not of the protocol, so it is
// a parameter rather than the `define it used to read directly.  The default
// reproduces the A7 value exactly, so every existing instantiation and
// testbench is unaffected; only the K7 top, which has 2 GiB of DDR and a
// 134 MB frame, overrides it.  Leaving it at 512 KB is what NAKed the first
// 134 MB write on the K7 board -- correctly, by the A7 contract it was still
// enforcing.
module udp_cmd_hdr #(
    parameter [31:0] MAX_BYTES = `UDP_CMD_MAX_BYTES
) (
    input      [127:0] raw_le,
    output     [31:0]  magic,
    output     [7:0]   cmd,
    output     [7:0]   flags,
    output     [15:0]  seq,
    output     [31:0]  addr,
    output     [31:0]  length,
    output             magic_ok,
    output             cmd_is_wr,
    output             cmd_is_rd,
    output             cmd_is_xpose,
    output             addr_align_ok,
    output             len_align_ok,
    output             len_range_ok,
    output             hdr_ok,

    input      [7:0]   i_cmd,
    input      [7:0]   i_flags,
    input      [15:0]  i_seq,
    input      [31:0]  i_addr,
    input      [31:0]  i_length,
    output     [127:0] packed_le
);

assign magic  = raw_le[31:0];
assign cmd    = raw_le[39:32];
assign flags  = raw_le[47:40];
assign seq    = raw_le[63:48];
assign addr   = raw_le[95:64];
assign length = raw_le[127:96];

assign magic_ok      = (magic == `UDP_CMD_MAGIC);
assign cmd_is_wr     = (cmd == `UDP_CMD_WR);
assign cmd_is_rd     = (cmd == `UDP_CMD_RD);
assign cmd_is_xpose  = (cmd == `UDP_CMD_TRANSPOSE);
assign addr_align_ok = (addr[3:0] == `UDP_CMD_ALIGN);
assign len_align_ok  = (length[3:0] == `UDP_CMD_ALIGN);
assign len_range_ok  = (length != 32'd0) && (length <= MAX_BYTES);
assign hdr_ok        = magic_ok & (
                           (cmd_is_wr & addr_align_ok & len_align_ok & len_range_ok) |
                           (cmd_is_rd & addr_align_ok & len_align_ok & len_range_ok) |
                           (cmd_is_xpose & (addr[8:0] == 9'd0)) |
                           ((cmd == `UDP_CMD_FFT2D) & (addr[8:0] == 9'd0)) |
                           (cmd == `UDP_CMD_CS_COEFF) |
                           (cmd == `UDP_CMD_CS_PARAM) |
                           ((cmd == `UDP_CMD_CS_ROW) & (addr[31:11] == 21'd0) &
                             (length == 32'd12)) |
                           ((cmd == `UDP_CMD_CS_RB)  & (addr[31:11] == 21'd0) &
                             (length == 32'd12)) |
                           ((cmd == `UDP_CMD_SEM) & (length <= 32'd16)) |
                           (cmd == `UDP_CMD_CS_CLEAR)
                       );

assign packed_le = {
    i_length,
    i_addr,
    i_seq,
    i_flags,
    i_cmd,
    `UDP_CMD_MAGIC
};

endmodule
