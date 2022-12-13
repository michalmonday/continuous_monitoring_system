
// continuous monitoring system module will allow the user to do the following:
// 1. Supply information like program counter, instruction, etc. to the module (which will be stored in a FIFO)
// 2. Supply trace start/end addresses (and enable both optionally).
// 3. Supply monitored address range (lower bound + higher bound + enable both optionally).

import continuous_monitoring_system_pkg::*;
`timescale 1ns/10ps

module continuous_monitoring_system
(
    input   logic   clk,
    input   logic   rst_n, 

    // data pkt signals (to be stored in FIFO)
    input   logic   [RISC_V_INSTRUCTION_WIDTH - 1 : 0]  instr,
    input   logic   [XLEN - 1 : 0]                      pc,

    // axi signals (interfacing with FIFO)
    output  logic                               M_AXIS_tvalid,
    input   logic                               M_AXIS_tready,
    output  logic   [DATA_PACKET_WIDTH - 1 : 0] M_AXIS_tdata,
    output  logic                               M_AXIS_tlast,
    input   logic   [31 : 0]                    tlast_interval, // number of items in FIFO after which tlast is asserted

    // control signals (determining operational mode of the continuous_monitoring_system)
    input   ctrl_addr_t                         ctrl_addr,
    input   logic   [CTRL_DATA_WIDTH - 1 : 0]   ctrl_wdata,
    input   logic                               ctrl_write_enable
);
    logic [DATA_PACKET_WIDTH - 1 : 0]    data_pkt;
    logic                                data_pkt_valid;

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

    trace_filter trace_filter_inst (
        .clk(clk),
        .rst_n(rst_n),
        .instr(instr),
        .pc(pc),
        .data_pkt(data_pkt),
        .data_pkt_valid(data_pkt_valid)
    );

   

    wire data_to_axi_write_enable = data_pkt_valid & 
                                    ~wfi_reached & 
                                    (trigger_trace_start_reached | ~trigger_trace_start_address_enabled) &
                                    (~trigger_trace_end_reached | ~trigger_trace_end_address_enabled) &
                                    (pc >= monitored_address_range_lower_bound | ~monitored_address_range_lower_bound_enabled) &
                                    (pc <= monitored_address_range_upper_bound | ~monitored_address_range_upper_bound_enabled);

    data_to_axi_stream #(
        .DATA_WIDTH(DATA_PACKET_WIDTH) // pc + instr sizes
    ) data_to_axi_stream_inst (
        .clk(clk),
        .rst_n(rst_n),
        .write_enable(data_to_axi_write_enable),
        .data_pkt(data_pkt),
        .tlast_interval(tlast_interval),
        .force_tlast(instr == WFI_INSTRUCTION),
        .M_AXIS_tvalid(M_AXIS_tvalid),
        .M_AXIS_tready(M_AXIS_tready),
        .M_AXIS_tdata(M_AXIS_tdata),
        .M_AXIS_tlast(M_AXIS_tlast)
    );


    // control registers setting
    always_ff @(posedge clk or posedge ctrl_write_enable) begin
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
            if (instr == WFI_INSTRUCTION) begin
                wfi_reached <= 1;
            end

            if (ctrl_write_enable) begin
                case(ctrl_addr)
                    // trace trigger enables and addresses (must match the current PC exactly to trigger)
                    TRIGGER_TRACE_START_ADDRESS_ENABLED: begin
                        trigger_trace_start_address_enabled <= ctrl_wdata;
                    end 
                    TRIGGER_TRACE_END_ADDRESS_ENABLED: begin
                        trigger_trace_end_address_enabled <= ctrl_wdata;
                    end
                    TRIGGER_TRACE_START_ADDRESS: begin
                        trigger_trace_start_address <= ctrl_wdata;
                    end
                    TRIGGER_TRACE_END_ADDRESS: begin
                        trigger_trace_end_address <= ctrl_wdata;
                    end

                    // monitored address range (must be within the range to collect trace)
                    MONITORED_ADDRESS_RANGE_LOWER_BOUND_ENABLED: begin
                        monitored_address_range_lower_bound_enabled <= ctrl_wdata;
                    end
                    MONITORED_ADDRESS_RANGE_UPPER_BOUND_ENABLED: begin
                        monitored_address_range_upper_bound_enabled <= ctrl_wdata;
                    end
                    MONITORED_ADDRESS_RANGE_LOWER_BOUND: begin
                        monitored_address_range_lower_bound <= ctrl_wdata;
                    end
                    MONITORED_ADDRESS_RANGE_UPPER_BOUND: begin
                        monitored_address_range_upper_bound <= ctrl_wdata;
                    end

                    // WFI reached can be used to reset (it is reset anyway after loading Overlay again)
                    WFI_REACHED: begin
                        wfi_reached <= ctrl_wdata;
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
                    // $display("trigger_trace_start_address (%H) reached", trigger_trace_start_address);
                end
                if (trigger_trace_end_address_enabled && (pc == trigger_trace_end_address)) begin
                    trigger_trace_end_reached <= 1;
                    trigger_trace_start_reached <= 0;
                    // $display("trigger_trace_end_address (%H) reached", trigger_trace_end_address);
                end
            end
        end
    end
endmodule