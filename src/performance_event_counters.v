
`timescale 1ns/10ps

module performance_event_counters #(
    parameter INPUT_EVENT_BITMAP_WIDTH = 115,
    parameter COUNTER_WIDTH = 7 // 7 * 115 = 805 (leaving 219 bits for other data, AXI DMA can't use more than 1024 bits data width)
) (
    input clk, 
    input rst_n,
    input [INPUT_EVENT_BITMAP_WIDTH-1:0] performance_events, // bitmap
    output wire [INPUT_EVENT_BITMAP_WIDTH-1:0] counters[COUNTER_WIDTH:0]
);
    reg [INPUT_EVENT_BITMAP_WIDTH-1:0] r_counters[COUNTER_WIDTH:0];
    assign counters = r_counters;

	always @(posedge clk) begin
        if (~rst_n) begin
            r_counters <= 0;
        end else begin
            // increase counter based on corresponding input bit
            for (int i = 0; i < COUNTER_WIDTH; i++) begin
                r_counters[i] <= r_counters[i] + performance_events[i];
            end
        end
	end
endmodule

