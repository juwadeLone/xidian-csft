// UDP memory command header (16 bytes, little-endian on the wire)
// Python: struct.pack('<IBBHII', MAGIC, cmd, flags, seq, addr, length)
//
// byte0  @ [7:0]     MAGIC[7:0]   = 0x4D 'M'
// byte1  @ [15:8]    MAGIC[15:8]  = 0x50 'P'
// byte2  @ [23:16]   MAGIC[23:16] = 0x48 'H'
// byte3  @ [31:24]   MAGIC[31:24] = 0x33 '3'
// byte4  @ [39:32]   cmd
// byte5  @ [47:40]   flags
// byte6  @ [55:48]   seq[7:0]
// byte7  @ [63:56]   seq[15:8]
// byte8  @ [71:64]   addr[7:0]
// ...
// byte12 @ [103:96]  length[7:0]
// ...
// byte15 @ [127:120] length[31:24]

`ifndef UDP_CMD_HDR_VH
`define UDP_CMD_HDR_VH

`define UDP_CMD_MAGIC       32'h3348_504D
`define UDP_CMD_WR          8'h01
`define UDP_CMD_TRANSPOSE   8'h02
`define UDP_CMD_RD          8'h03
`define UDP_CMD_FFT2D       8'h04
`define UDP_CMD_CS_COEFF    8'h05
`define UDP_CMD_CS_PARAM    8'h06
`define UDP_CMD_CS_ROW      8'h07
`define UDP_CMD_CS_RB       8'h08
`define UDP_CMD_CS_CLEAR    8'h09
`define UDP_CMD_SEM         8'h10
`define UDP_CMD_WR_ACK      8'h81
`define UDP_CMD_XPOSE_ACK   8'h82
`define UDP_CMD_RD_ACK      8'h83
`define UDP_CMD_NAK         8'h8F
`define UDP_CMD_TELEM       8'h84
`define UDP_CMD_FFT2D_ACK   8'h85
`define UDP_CMD_CS_COEFF_ACK 8'h86
`define UDP_CMD_CS_PARAM_ACK 8'h87
`define UDP_CMD_CS_ROW_ACK  8'h88
`define UDP_CMD_CS_RB_ACK   8'h89
`define UDP_CMD_CS_CLEAR_ACK 8'h8A
`define UDP_CMD_SEM_ACK     8'h90
`define UDP_CMD_MAX_BYTES   32'd524288
`define UDP_CMD_ALIGN       4'd0
`define UDP_CMD_RD_DATA     8'h83

`endif
