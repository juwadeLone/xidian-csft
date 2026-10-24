`timescale 1ns/1ps

module tile_transpose_pingpong #(
    parameter integer TILE_LOG2 = 6,
    parameter integer DATA_W = 64
) (
    input  wire                     clk,
    input  wire                     rst,
    input  wire                     clear,
    input  wire [DATA_W-1:0]        s_data,
    input  wire                     s_valid,
    output wire                     s_ready,
    input  wire                     s_sof,
    input  wire                     s_eof,
    output wire [DATA_W-1:0]        m_data,
    output wire                     m_valid,
    input  wire                     m_ready,
    output wire                     m_sof,
    output wire                     m_eof,
    output reg                      protocol_error,
    output reg                      overflow_error
);
    localparam integer INDEX_W=2*TILE_LOG2;
    localparam integer TILE_SAMPLES=1<<INDEX_W;
    (* ram_style="block" *) reg [DATA_W-1:0] bank0[0:TILE_SAMPLES-1];
    (* ram_style="block" *) reg [DATA_W-1:0] bank1[0:TILE_SAMPLES-1];
    reg wr_bank;
    reg [INDEX_W-1:0] wr_index;
    reg [1:0] bank_ready;
    reg next_read_bank,read_bank,read_active;
    reg [INDEX_W-1:0] read_index;
    reg [DATA_W-1:0] read_data_q;
    reg read_valid_q,read_sof_q,read_eof_q;
    wire [INDEX_W-1:0] transposed_read_addr =
        {read_index[TILE_LOG2-1:0], read_index[INDEX_W-1:TILE_LOG2]};
    assign s_ready=!bank_ready[wr_bank];
    assign m_data=read_data_q;
    assign m_valid=read_valid_q;
    assign m_sof=read_sof_q;
    assign m_eof=read_eof_q;
    wire output_can_advance=!read_valid_q||m_ready;

    always @(posedge clk) begin
        if(rst||clear) begin
            wr_bank<=0;wr_index<=0;bank_ready<=0;next_read_bank<=0;
            read_bank<=0;read_active<=0;read_index<=0;read_data_q<=0;
            read_valid_q<=0;read_sof_q<=0;read_eof_q<=0;
            protocol_error<=0;overflow_error<=0;
        end else begin
            // AXI-stream style backpressure is not an overflow: while READY is
            // low, a compliant source keeps VALID and the payload stable until
            // the transfer is accepted.  No transfer exists in that interval,
            // so only an accepted VALID/READY beat may advance write state.
            if(s_valid&&s_ready) begin
                if((wr_index==0&&!s_sof)||(wr_index!=0&&s_sof)||
                   (wr_index==TILE_SAMPLES-1&&!s_eof)||(wr_index!=TILE_SAMPLES-1&&s_eof))
                    protocol_error<=1;
                if(!wr_bank) bank0[wr_index]<=s_data; else bank1[wr_index]<=s_data;
                if(wr_index==TILE_SAMPLES-1) begin
                    bank_ready[wr_bank]<=1;wr_bank<=~wr_bank;wr_index<=0;
                end else wr_index<=wr_index+1'b1;
            end
            if(!read_active&&bank_ready[next_read_bank]) begin
                read_active<=1;read_bank<=next_read_bank;read_index<=0;
            end
            if(output_can_advance) begin
                if(read_active) begin
                    if(!read_bank) read_data_q<=bank0[transposed_read_addr];
                    else read_data_q<=bank1[transposed_read_addr];
                    read_valid_q<=1;read_sof_q<=read_index==0;
                    read_eof_q<=read_index==TILE_SAMPLES-1;
                    if(read_index==TILE_SAMPLES-1) begin
                        read_active<=0;bank_ready[read_bank]<=0;
                        next_read_bank<=~next_read_bank;read_index<=0;
                    end else read_index<=read_index+1'b1;
                end else begin
                    read_valid_q<=0;read_sof_q<=0;read_eof_q<=0;
                end
            end
        end
    end
endmodule
