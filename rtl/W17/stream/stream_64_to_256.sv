`timescale 1ns/1ps

module stream_64_to_256 (
    input  wire          clk,
    input  wire          rst,
    input  wire          clear,
    input  wire [63:0]   s_data,
    input  wire          s_valid,
    output wire          s_ready,
    output wire [255:0]  m_data,
    output wire          m_valid,
    input  wire          m_ready
);
    reg [255:0] data_q;
    reg [1:0] lane_q;
    reg full_q;
    wire output_pop = full_q && m_ready;

    assign s_ready = !full_q || output_pop;
    assign m_data = data_q;
    assign m_valid = full_q;

    always @(posedge clk) begin
        if (rst || clear) begin
            data_q <= '0;
            lane_q <= '0;
            full_q <= 1'b0;
        end else begin
            if (output_pop)
                full_q <= 1'b0;

            if (s_valid && s_ready) begin
                data_q[lane_q*64 +: 64] <= s_data;
                if (lane_q == 2'd3) begin
                    lane_q <= 2'd0;
                    full_q <= 1'b1;
                end else begin
                    lane_q <= lane_q + 1'b1;
                end
            end
        end
    end
endmodule

