
// continuous monitoring system module will allow the user to do the following:
// 1. Supply information like program counter, instruction, etc. to the module (which will be stored in a FIFO)
// 2. Supply trace start/end addresses (and enable both optionally).
// 3. Supply monitored address range (lower bound + higher bound + enable both optionally).

`timescale 1ns/10ps

`define WFI_INSTRUCTION 'h0001

`define ADDR_WIDTH 8 // internal addressing (each of 256 addresses can result in a different action upon writing/reading)
`define DATA_WIDTH 64 // control data width, the functionality of the module is controlled by writing to address+data ports

`define ADDR_TRIGGER_TRACE_START_ADDRESS_ENABLED 0
`define ADDR_TRIGGER_TRACE_END_ADDRESS_ENABLED 1
`define ADDR_TRIGGER_TRACE_START_ADDRESS 2
`define ADDR_TRIGGER_TRACE_END_ADDRESS 3
`define ADDR_MONITORED_ADDRESS_RANGE_LOWER_BOUND_ENABLED 4
`define ADDR_MONITORED_ADDRESS_RANGE_UPPER_BOUND_ENABLED 5
`define ADDR_MONITORED_ADDRESS_RANGE_LOWER_BOUND 6
`define ADDR_MONITORED_ADDRESS_RANGE_UPPER_BOUND 7
`define ADDR_WFI_REACHED 8

module continuous_monitoring_system #(
    parameter XLEN = 64,
    parameter AXI_DATA_WIDTH = XLEN + 32,
    parameter CTRL_WRITE_ENABLE_POSEDGE_TRIGGERED = 1 // 1 = write enable is pos edge triggered, 0 = write enable is level triggered
    //parameter ADDR_WIDTH = 4 // internal addressing (each of 16 addresses can result in a different action upon writing/reading)
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
    input [`ADDR_WIDTH-1:0] ctrl_addr,
    input [`DATA_WIDTH-1:0] ctrl_wdata,
    input ctrl_write_enable
);
    wire drop_instr;

    // At the end of a program, a "wfi" (wait for interrupt) instruction is executed 
    // which stops the program from running. This is a good time to stop sending trace
    // to the FIFO.
    reg wfi_reached = 0;

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

    wire [AXI_DATA_WIDTH-1:0]data_pkt = {pc, instr};

    wire data_to_axi_write_enable = pc_valid &
                                    ~drop_instr & 
                                    ~wfi_reached & 
                                    (trigger_trace_start_reached | ~trigger_trace_start_address_enabled) &
                                    (~trigger_trace_end_reached | ~trigger_trace_end_address_enabled) &
                                    (pc >= monitored_address_range_lower_bound | ~monitored_address_range_lower_bound_enabled) &
                                    (pc <= monitored_address_range_upper_bound | ~monitored_address_range_upper_bound_enabled)
                                    ;

    data_to_axi_stream #(
        .DATA_WIDTH(AXI_DATA_WIDTH) // pc + instr sizes
    ) data_to_axi_stream_inst (
        .clk(clk),
        .rst_n(rst_n),
        .write_enable(data_to_axi_write_enable),
        .data_pkt(data_pkt),
        .tlast_interval(tlast_interval),
        .force_tlast(instr == `WFI_INSTRUCTION),
        .M_AXIS_tvalid(M_AXIS_tvalid),
        .M_AXIS_tready(M_AXIS_tready),
        .M_AXIS_tdata(M_AXIS_tdata),
        .M_AXIS_tlast(M_AXIS_tlast)
    );


    always @(posedge clk) begin
        if (rst_n == 0) begin
        end
        else begin
            if (trigger_trace_start_address_enabled & (pc == trigger_trace_start_address)) begin
                trigger_trace_start_reached <= 1;
                trigger_trace_end_reached <= 0;
                $display("trigger_trace_start_address (%H) reached", trigger_trace_start_address);
            end
            if (trigger_trace_end_address_enabled & (pc == trigger_trace_end_address)) begin
                trigger_trace_end_reached <= 1;
                trigger_trace_start_reached <= 0;
                $display("trigger_trace_end_address (%H) reached", trigger_trace_end_address);
            end
        end
    end

    // control registers setting
    always @(posedge clk) begin
        if (rst_n == 0) begin
            // whole module status is reset (even if it was previously set through "ctrl_addr" and "ctrl_data")
            wfi_reached <= 0;

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
        end
        else begin
            if (instr == `WFI_INSTRUCTION) begin
                wfi_reached <= 1;
            end
            else begin
                wfi_reached <= wfi_reached;
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
                    `ADDR_WFI_REACHED: begin
                        wfi_reached <= ctrl_wdata;
                    end

                    default: begin
                        // do nothing
                    end
                endcase
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