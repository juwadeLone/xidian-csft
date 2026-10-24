`timescale 1ns/1ps

module stream_256_to_64 (
    input  wire          clk,
    input  wire          rst,
    input  wire          clear,
    input  wire [255:0]  s_data,
    input  wire          s_valid,
    output wire          s_ready,
    output wire [63:0]   m_data,
    output wire          m_valid,
    input  wire          m_ready
);
    reg [255:0] data_q;
    reg [1:0] lane_q;
    reg full_q;

    wire pop_last = full_q && m_ready && lane_q == 2'd3;

    // Accept the next AXI beat on the same edge that lane 3 is consumed.  This
    // removes the otherwise unavoidable one-cycle gap every four samples and
    // is required by the no-stall-inside-a-vector SDF contract.
    assign s_ready = !full_q || pop_last;
    assign m_valid = full_q;
    assign m_data = data_q[lane_q*64 +: 64];

    always @(posedge clk) begin
        if (rst || clear) begin
            data_q <= '0;
            lane_q <= '0;
            full_q <= 1'b0;
        end else begin
            if (pop_last && s_valid) begin
                data_q <= s_data;
                lane_q <= 2'd0;
                full_q <= 1'b1;
            end else if (s_valid && s_ready) begin
                data_q <= s_data;
                lane_q <= 2'd0;
                full_q <= 1'b1;
            end else if (m_valid && m_ready) begin
                if (lane_q == 2'd3) begin
                    lane_q <= 2'd0;
                    full_q <= 1'b0;
                end else begin
                    lane_q <= lane_q + 1'b1;
                end
            end
        end
    end
endmodule
