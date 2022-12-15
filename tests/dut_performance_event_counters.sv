`timescale 1ns/10ps  // time-unit = 1 ns, precision = 10 ps

module dut_performance_event_counters ();
    localparam COUNTER_WIDTH = 7;
    localparam INPUT_EVENT_BITMAP_WIDTH=115;
    
    reg clk = 0;
    reg rst_n = 1;
    reg [INPUT_EVENT_BITMAP_WIDTH-1:0] performance_events = 0; // bitmap
    wire [INPUT_EVENT_BITMAP_WIDTH-1:0] counters[COUNTER_WIDTH-1:0];

    performance_event_counters #(
        .INPUT_EVENT_BITMAP_WIDTH(INPUT_EVENT_BITMAP_WIDTH),
        .COUNTER_WIDTH(COUNTER_WIDTH)
    ) performance_event_counters_inst (
        .clk(clk),
        .rst_n(rst_n),
        .performance_events(performance_events),
        .counters(counters)
    );

    reg [5:0] i = 0;
    always @ (posedge clk) begin
        i = i + 1;
        case (i)
            0: performance_events = 'b001;
            1: performance_events = 'b101;
            2: performance_events = 'b001;
            3: performance_events = 'b011;
            4: performance_events = 'b101;
            5: performance_events = 'b001;
            6: performance_events = 'b001;
            7: performance_events = 'b001;
            8: performance_events = 'b001;
            9: performance_events = 'b001;

            10: performance_events = 'b001;
            11: performance_events = 'b001;

            12: performance_events = 'b001;
            13: performance_events = 'b001;
            14: rst_n = 0;
            15: rst_n = 1;
            16: performance_events = 'b001;
            17: performance_events = 'b001;
            18: performance_events = 'b001;
            19: performance_events = 'b001;
            20: performance_events = 'b001;
            default: begin
            end
        endcase
    end

    always begin
        clk = ~clk;
        #5;
    end

endmodule


