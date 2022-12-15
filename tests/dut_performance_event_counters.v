`timescale 1ns/10ps  // time-unit = 1 ns, precision = 10 ps

module dut_performance_event_counters (
    parameter INPUT_EVENT_BITMAP_WIDTH = 115,
    parameter COUNTER_WIDTH = 7
);
    input clk = 0, 
    input rst_n = 1,
    input [INPUT_EVENT_BITMAP_WIDTH-1:0] performance_events = 0, // bitmap
    wire [INPUT_EVENT_BITMAP_WIDTH-1:0] counters[COUNTER_WIDTH:0]

    performance_event_counters #(
        .INPUT_EVENT_BITMAP_WIDTH(115),
        .COUNTER_WIDTH(7)
    ) dut_performance_event_counters (
        .clk(clk),
        .rst_n(rst_n),
        .performance_events(performance_events),
        .counters(counters)
    );


    reg [5:0] i = 0;
    always @ (posedge clk) begin
        i = i + 1;
        case (i)
            0: performance_events = `INPUT_EVENT_BITMAP_WIDTH'b001;
            1: performance_events = `INPUT_EVENT_BITMAP_WIDTH'b101;
            2: performance_events = `INPUT_EVENT_BITMAP_WIDTH'b001;
            3: performance_events = `INPUT_EVENT_BITMAP_WIDTH'b011;
            4: performance_events = `INPUT_EVENT_BITMAP_WIDTH'b101;
            5: performance_events = `INPUT_EVENT_BITMAP_WIDTH'b001;
            6: performance_events = `INPUT_EVENT_BITMAP_WIDTH'b001;
            7: performance_events = `INPUT_EVENT_BITMAP_WIDTH'b001;
            8: performance_events = `INPUT_EVENT_BITMAP_WIDTH'b001;
            9: performance_events = `INPUT_EVENT_BITMAP_WIDTH'b001;

            10: performance_events = `INPUT_EVENT_BITMAP_WIDTH'b001;
            11: performance_events = `INPUT_EVENT_BITMAP_WIDTH'b001;

            12: performance_events = `INPUT_EVENT_BITMAP_WIDTH'b001;
            13: performance_events = `INPUT_EVENT_BITMAP_WIDTH'b001;
            14: rst_n = 0;
            15: rst_n = 1;
            16: performance_events = `INPUT_EVENT_BITMAP_WIDTH'b001;
            17: performance_events = `INPUT_EVENT_BITMAP_WIDTH'b001;
            18: performance_events = `INPUT_EVENT_BITMAP_WIDTH'b001;
            19: performance_events = `INPUT_EVENT_BITMAP_WIDTH'b001;
            20: performance_events = `INPUT_EVENT_BITMAP_WIDTH'b001;
            default: begin
            end
        endcase
    end

endmodule


