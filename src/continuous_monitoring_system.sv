
// continuous monitoring system module will allow the user to do the following:
// 1. Supply information like program counter, instruction, etc. to the module (which will be stored in a FIFO)
// 2. Supply trace start/end addresses (and enable both optionally).
// 3. Supply monitored address range (lower bound + higher bound + enable both optionally).

`timescale 1ns/10ps

`define WFI_INSTRUCTION 'h10500073

`define CTRL_ADDR_WIDTH 8 // internal addressing (each of 256 addresses can result in a different action upon writing/reading)
`define CTRL_DATA_WIDTH 64 // control data width, the functionality of the module is controlled by writing to address+data ports

`define ADDR_TRIGGER_TRACE_START_ADDRESS_ENABLED 0
`define ADDR_TRIGGER_TRACE_END_ADDRESS_ENABLED 1
`define ADDR_TRIGGER_TRACE_START_ADDRESS 2
`define ADDR_TRIGGER_TRACE_END_ADDRESS 3
`define ADDR_MONITORED_ADDRESS_RANGE_LOWER_BOUND_ENABLED 4
`define ADDR_MONITORED_ADDRESS_RANGE_UPPER_BOUND_ENABLED 5
`define ADDR_MONITORED_ADDRESS_RANGE_LOWER_BOUND 6
`define ADDR_MONITORED_ADDRESS_RANGE_UPPER_BOUND 7
`define ADDR_WFI_STOP 8
`define ADDR_CLK_COUNTER 9
`define ADDR_LAST_WRITE_TIMESTAMP 10

`define CLK_COUNTER_WIDTH 64
`define NO_OF_PERFORMANCE_EVENTS 115

// 8 bits allow to store 256 possible values, it could be enough to make them distinct enough for creating the program profile
`define PERFORMANCE_EVENT_MOD_COUNTER_WIDTH 7

module continuous_monitoring_system #(
    parameter XLEN = 64,
    parameter AXI_DATA_WIDTH = 1024,//XLEN + 32 + `CLK_COUNTER_WIDTH,
    parameter CTRL_WRITE_ENABLE_POSEDGE_TRIGGERED = 1 // 1 = write enable is pos edge triggered, 0 = write enable is level triggered
    //parameter CTRL_ADDR_WIDTH = 4 // internal addressing (each of 16 addresses can result in a different action upon writing/reading)
) (
    input clk, rst_n, 

    // data pkt signals (to be stored in FIFO)
    input [31:0] instr,
    input [XLEN-1:0] pc,
    input pc_valid, // determines whether the current instruction/pc is executed now


    // axi signals (interfacing with FIFO)
    output wire M_AXIS_tvalid,
    input M_AXIS_tready,
    output wire [AXI_DATA_WIDTH-1:0] M_AXIS_tdata,
    output wire M_AXIS_tlast,
    input [31:0] tlast_interval, // number of items in FIFO after which tlast is asserted

    // control signals (determining operational mode of the continuous_monitoring_system)
    input [`CTRL_ADDR_WIDTH-1:0] ctrl_addr,
    input [`CTRL_DATA_WIDTH-1:0] ctrl_wdata,
    input ctrl_write_enable,

    // enable the module (if disabled, the module will not send any data to the FIFO)
    // this may be connected to the GPIO rst_n (the same one used to reset the processor)
    input en,
    input [`NO_OF_PERFORMANCE_EVENTS-1:0]performance_events
);
    wire drop_instr;

    // At the end of a program, a "wfi" (wait for interrupt) instruction is executed 
    // which stops the program from running. This is a good time to stop sending trace
    // to the FIFO.
    reg [1:0]wfi_stop = 0; // it is 2 bits to use it as a counter and control write enable only based on MSB (to delay disabling write by 1 cycle)

    // monitored address range
    reg monitored_address_range_lower_bound_enabled = 0;
    reg monitored_address_range_upper_bound_enabled = 0;
    reg [XLEN-1:0] monitored_address_range_lower_bound = 0;
    reg [XLEN-1:0] monitored_address_range_upper_bound = -1;
    // wire is_pc_in_range = (pc >= monitored_address_range_lower_bound) & (pc <= monitored_address_range_upper_bound);

    reg trigger_trace_start_address_enabled = 0;
    reg trigger_trace_end_address_enabled = 0;
    reg [XLEN-1:0] trigger_trace_start_address = 0;
    reg [XLEN-1:0] trigger_trace_end_address = -1;
    reg trigger_trace_start_reached = 0;
    reg trigger_trace_end_reached = 0;

    reg [`CLK_COUNTER_WIDTH-1:0] clk_counter = 0;
    reg [`CLK_COUNTER_WIDTH-1:0] last_write_timestamp = 0;
    wire [`CLK_COUNTER_WIDTH-1:0] clk_counter_delta = clk_counter - last_write_timestamp;

    wire [`PERFORMANCE_EVENT_MOD_COUNTER_WIDTH-1:0] performance_event_counters[`NO_OF_PERFORMANCE_EVENTS-1:0];

    // edge detector allows to detect pos/neg edges of a write enable signal
    // this is useful when this module is controlled by AXI GPIO from Python
    // it can be disabled by setting CTRL_WRITE_ENABLE_POSEDGE_TRIGGERED to 0
    edge_detector edge_detector_write_enable(
        .clk(clk),
        .sig(ctrl_write_enable),
        .neg_edge(),
        .pos_edge(ctrl_write_enable_pos_edge)
    );

    trace_filter trace_filter_inst (
        .clk(clk),
        .instr(instr),
        .drop_instr(drop_instr)
    );

    wire [AXI_DATA_WIDTH-1:0]data_pkt = {
            instr,
            clk_counter_delta,
            pc,
            performance_event_counters[0], performance_event_counters[1], performance_event_counters[2], performance_event_counters[3],
            performance_event_counters[4], performance_event_counters[5], performance_event_counters[6], performance_event_counters[7],
            performance_event_counters[8], performance_event_counters[9], performance_event_counters[10], performance_event_counters[11],
            performance_event_counters[12], performance_event_counters[13], performance_event_counters[14], performance_event_counters[15],
            performance_event_counters[16], performance_event_counters[17], performance_event_counters[18], performance_event_counters[19],
            performance_event_counters[20], performance_event_counters[21], performance_event_counters[22], performance_event_counters[23],
            performance_event_counters[24], performance_event_counters[25], performance_event_counters[26], performance_event_counters[27],
            performance_event_counters[28], performance_event_counters[29], performance_event_counters[30], performance_event_counters[31],
            performance_event_counters[32], performance_event_counters[33], performance_event_counters[34], performance_event_counters[35],
            performance_event_counters[36], performance_event_counters[37], performance_event_counters[38], performance_event_counters[39],
            performance_event_counters[40], performance_event_counters[41], performance_event_counters[42], performance_event_counters[43],
            performance_event_counters[44], performance_event_counters[45], performance_event_counters[46], performance_event_counters[47],
            performance_event_counters[48], performance_event_counters[49], performance_event_counters[50], performance_event_counters[51],
            performance_event_counters[52], performance_event_counters[53], performance_event_counters[54], performance_event_counters[55],
            performance_event_counters[56], performance_event_counters[57], performance_event_counters[58], performance_event_counters[59],
            performance_event_counters[60], performance_event_counters[61], performance_event_counters[62], performance_event_counters[63],
            performance_event_counters[64], performance_event_counters[65], performance_event_counters[66], performance_event_counters[67],
            performance_event_counters[68], performance_event_counters[69], performance_event_counters[70], performance_event_counters[71],
            performance_event_counters[72], performance_event_counters[73], performance_event_counters[74], performance_event_counters[75],
            performance_event_counters[76], performance_event_counters[77], performance_event_counters[78], performance_event_counters[79],
            performance_event_counters[80], performance_event_counters[81], performance_event_counters[82], performance_event_counters[83],
            performance_event_counters[84], performance_event_counters[85], performance_event_counters[86], performance_event_counters[87],
            performance_event_counters[88], performance_event_counters[89], performance_event_counters[90], performance_event_counters[91],
            performance_event_counters[92], performance_event_counters[93], performance_event_counters[94], performance_event_counters[95],
            performance_event_counters[96], performance_event_counters[97], performance_event_counters[98], performance_event_counters[99],
            performance_event_counters[100], performance_event_counters[101], performance_event_counters[102], performance_event_counters[103],
            performance_event_counters[104], performance_event_counters[105], performance_event_counters[106], performance_event_counters[107],
            performance_event_counters[108], performance_event_counters[109], performance_event_counters[110], performance_event_counters[111],
            performance_event_counters[112], performance_event_counters[113], performance_event_counters[114]
        };

        // performance_event_counters[114], performance_event_counters[113], performance_event_counters[112], performance_event_counters[111],
        // performance_event_counters[110], performance_event_counters[109], performance_event_counters[108], performance_event_counters[107],
        // performance_event_counters[106], performance_event_counters[105], performance_event_counters[104], performance_event_counters[103],
        // performance_event_counters[102], performance_event_counters[101], performance_event_counters[100], performance_event_counters[99],
        // performance_event_counters[98], performance_event_counters[97], performance_event_counters[96], performance_event_counters[95],
        // performance_event_counters[94], performance_event_counters[93], performance_event_counters[92], performance_event_counters[91],
        // performance_event_counters[90], performance_event_counters[89], performance_event_counters[88], performance_event_counters[87],
        // performance_event_counters[86], performance_event_counters[85], performance_event_counters[84], performance_event_counters[83],
        // performance_event_counters[82], performance_event_counters[81], performance_event_counters[80], performance_event_counters[79],
        // performance_event_counters[78], performance_event_counters[77], performance_event_counters[76], performance_event_counters[75],
        // performance_event_counters[74], performance_event_counters[73], performance_event_counters[72], performance_event_counters[71],
        // performance_event_counters[70], performance_event_counters[69], performance_event_counters[68], performance_event_counters[67],
        // performance_event_counters[66], performance_event_counters[65], performance_event_counters[64], performance_event_counters[63],
        // performance_event_counters[62], performance_event_counters[61], performance_event_counters[60], performance_event_counters[59],
        // performance_event_counters[58], performance_event_counters[57], performance_event_counters[56], performance_event_counters[55],
        // performance_event_counters[54], performance_event_counters[53], performance_event_counters[52], performance_event_counters[51],
        // performance_event_counters[50], performance_event_counters[49], performance_event_counters[48], performance_event_counters[47],
        // performance_event_counters[46], performance_event_counters[45], performance_event_counters[44], performance_event_counters[43],
        // performance_event_counters[42], performance_event_counters[41], performance_event_counters[40], performance_event_counters[39],
        // performance_event_counters[38], performance_event_counters[37], performance_event_counters[36], performance_event_counters[35],
        // performance_event_counters[34], performance_event_counters[33], performance_event_counters[32], performance_event_counters[31],
        // performance_event_counters[30], performance_event_counters[29], performance_event_counters[28], performance_event_counters[27],
        // performance_event_counters[26], performance_event_counters[25], performance_event_counters[24], performance_event_counters[23],
        // performance_event_counters[22], performance_event_counters[21], performance_event_counters[20], performance_event_counters[19],
        // performance_event_counters[18], performance_event_counters[17], performance_event_counters[16], performance_event_counters[15],
        // performance_event_counters[14], performance_event_counters[13], performance_event_counters[12], performance_event_counters[11],
        // performance_event_counters[10], performance_event_counters[9], performance_event_counters[8], performance_event_counters[7],
        // performance_event_counters[6], performance_event_counters[5], performance_event_counters[4], performance_event_counters[3],
        // performance_event_counters[2], performance_event_counters[1], performance_event_counters[0]
        // };

    wire data_to_axi_write_enable = en &
                                    pc_valid &
                                    ~drop_instr & 
                                    (wfi_stop < 2) & 
                                    (trigger_trace_start_reached | ~trigger_trace_start_address_enabled) &
                                    (~trigger_trace_end_reached | ~trigger_trace_end_address_enabled) &
                                    (pc >= monitored_address_range_lower_bound | ~monitored_address_range_lower_bound_enabled) &
                                    (pc <= monitored_address_range_upper_bound | ~monitored_address_range_upper_bound_enabled)
                                    ;

    wire performance_counters_rst_n = ~data_to_axi_write_enable & rst_n; // reset upon write to FIFO

    performance_event_counters performance_event_counters_inst (
        .clk(clk),
        .rst_n(performance_counters_rst_n),
        .performance_events(performance_events), // input bitmap (each bit is indicating if the corresponding performance event happens now)
        .counters(performance_event_counters)  // output counters
    );

    // performance_event_counters performance_event_counters_inst (
    //     .clk(clk),
    //     // .rst_n(performance_counters_rst_n),
    //     .rst_n(1),
    //     .performance_events(performance_events), // input bitmap (each bit is indicating if the corresponding performance event happens now)
    //     .counters(performance_event_counters_out)  // output counters
    // );

    data_to_axi_stream #(
        .DATA_WIDTH(AXI_DATA_WIDTH) // pc + instr sizes
    ) data_to_axi_stream_inst (
        .clk(clk),
        .rst_n(rst_n),
        .write_enable(data_to_axi_write_enable),
        .data_pkt(data_pkt),
        .tlast_interval(tlast_interval),
        //.force_tlast(instr == `WFI_INSTRUCTION),
        .tlast(instr == `WFI_INSTRUCTION),
        .M_AXIS_tvalid(M_AXIS_tvalid),
        .M_AXIS_tready(M_AXIS_tready),
        .M_AXIS_tdata(M_AXIS_tdata),
        .M_AXIS_tlast(M_AXIS_tlast)
    );

    // always @(posedge clk) begin
    //     if (rst_n == 0) begin
    //     end
    //     else begin
    //         if (trigger_trace_start_address_enabled && (pc == trigger_trace_start_address)) begin
    //             trigger_trace_start_reached <= 1;
    //             trigger_trace_end_reached <= 0;
    //             $display("trigger_trace_start_address (%H) reached", trigger_trace_start_address);
    //         end
    //         if (trigger_trace_end_address_enabled && (pc == trigger_trace_end_address)) begin
    //             trigger_trace_end_reached <= 1;
    //             trigger_trace_start_reached <= 0;
    //             $display("trigger_trace_end_address (%H) reached", trigger_trace_end_address);
    //         end
    //     end
    // end

    // control registers setting
    always @(posedge clk) begin
        if (rst_n == 0) begin
            // whole module status is reset (even if it was previously set through "ctrl_addr" and "ctrl_data")
            wfi_stop <= 0;

            monitored_address_range_lower_bound_enabled <= 0;
            monitored_address_range_upper_bound_enabled <= 0;
            monitored_address_range_lower_bound <= 0;
            monitored_address_range_upper_bound <= -1;

            trigger_trace_start_address_enabled <= 0;
            trigger_trace_end_address_enabled <= 0;
            trigger_trace_start_address <= 0;
            trigger_trace_end_address <= -1;
            trigger_trace_start_reached <= 0;
            trigger_trace_end_reached <= 0;

            clk_counter <= 0;
            last_write_timestamp <= 0;
        end
        else begin
            clk_counter <= clk_counter + 1;
            if (data_to_axi_write_enable) begin
                last_write_timestamp <= clk_counter;
            end 

            if (instr == `WFI_INSTRUCTION && wfi_stop < 2 && en) begin
                wfi_stop <= wfi_stop + 1;
            end 
            else if (instr != `WFI_INSTRUCTION) begin
                wfi_stop <= 0;
            end

            // if write enable is active (posedge/level triggered mode can be selected by CTRL_WRITE_ENABLE_POSEDGE_TRIGGERED)
            if ((CTRL_WRITE_ENABLE_POSEDGE_TRIGGERED & ctrl_write_enable_pos_edge) || (~CTRL_WRITE_ENABLE_POSEDGE_TRIGGERED & ctrl_write_enable)) begin
                case(ctrl_addr)
                    // trace trigger enables and addresses (must match the current PC exactly to trigger)
                    `ADDR_TRIGGER_TRACE_START_ADDRESS_ENABLED: begin
                        trigger_trace_start_address_enabled <= ctrl_wdata;
                    end 
                    `ADDR_TRIGGER_TRACE_END_ADDRESS_ENABLED: begin
                        trigger_trace_end_address_enabled <= ctrl_wdata;
                    end
                    `ADDR_TRIGGER_TRACE_START_ADDRESS: begin
                        trigger_trace_start_address <= ctrl_wdata;
                    end
                    `ADDR_TRIGGER_TRACE_END_ADDRESS: begin
                        trigger_trace_end_address <= ctrl_wdata;
                    end

                    // monitored address range (must be within the range to collect trace)
                    `ADDR_MONITORED_ADDRESS_RANGE_LOWER_BOUND_ENABLED: begin
                        monitored_address_range_lower_bound_enabled <= ctrl_wdata;
                    end
                    `ADDR_MONITORED_ADDRESS_RANGE_UPPER_BOUND_ENABLED: begin
                        monitored_address_range_upper_bound_enabled <= ctrl_wdata;
                    end
                    `ADDR_MONITORED_ADDRESS_RANGE_LOWER_BOUND: begin
                        monitored_address_range_lower_bound <= ctrl_wdata;
                    end
                    `ADDR_MONITORED_ADDRESS_RANGE_UPPER_BOUND: begin
                        monitored_address_range_upper_bound <= ctrl_wdata;
                    end

                    // WFI reached can be used to reset (it is reset anyway after loading Overlay again)
                    `ADDR_WFI_STOP: begin
                        wfi_stop <= ctrl_wdata;
                    end
                    `ADDR_CLK_COUNTER: begin
                        clk_counter <= ctrl_wdata;
                    end
                    `ADDR_LAST_WRITE_TIMESTAMP: begin
                        last_write_timestamp <= ctrl_wdata;
                    end

                    default: begin
                        // do nothing
                    end
                endcase
            end
            else begin
                if (trigger_trace_start_address_enabled && (pc == trigger_trace_start_address)) begin
                    trigger_trace_start_reached <= 1;
                    trigger_trace_end_reached <= 0;
                    $display("trigger_trace_start_address (%H) reached", trigger_trace_start_address);
                end
                if (trigger_trace_end_address_enabled && (pc == trigger_trace_end_address)) begin
                    trigger_trace_end_reached <= 1;
                    trigger_trace_start_reached <= 0;
                    $display("trigger_trace_end_address (%H) reached", trigger_trace_end_address);
                end
            end
        end
    end
endmodule

// (* always_ready, always_enabled *) method Bool pc_valid;

// // Core events
// (* always_ready, always_enabled *) method Bit#(Report_Width) evt_MEM_CAP_LOAD;
// (* always_ready, always_enabled *) method Bit#(Report_Width) evt_MEM_CAP_STORE;
// (* always_ready, always_enabled *) method Bit#(Report_Width) evt_MEM_CAP_LOAD_TAG_SET;
// (* always_ready, always_enabled *) method Bit#(Report_Width) evt_MEM_CAP_STORE_TAG_SET;

   // // TGC (tag cache) events
// (* always_ready, always_enabled *) method Bit#(Report_Width) tgc_evt_WRITE;
// (* always_ready, always_enabled *) method Bit#(Report_Width) tgc_evt_WRITE_MISS;
// (* always_ready, always_enabled *) method Bit#(Report_Width) tgc_evt_READ;
// (* always_ready, always_enabled *) method Bit#(Report_Width) tgc_evt_READ_MISS;
// (* always_ready, always_enabled *) method Bit#(Report_Width) tgc_evt_EVICT;
// (* always_ready, always_enabled *) method Bit#(Report_Width) tgc_evt_SET_TAG_WRITE;
// (* always_ready, always_enabled *) method Bit#(Report_Width) tgc_evt_SET_TAG_READ;