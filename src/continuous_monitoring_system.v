
// continuous monitoring system module will allow the user to do the following:
// 1. Supply a program binary. This way trace will only consist of indirect jumps, that can't be inferred from the binary.


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
`timescale 1ns/10ps

`define WFI_INSTRUCTION 16`h0001


module continuous_monitoring_system #(
    parameter XLEN = 64,
    parameter AXI_DATA_WIDTH = XLEN + 32
    //parameter ADDR_WIDTH = 4 // internal addressing (each of 16 addresses can result in a different action upon writing/reading)
) (
    input clk, rst_n, 
    // input [ADDR_WIDTH-1:0] addr,
    // input [DATA_WIDTH-1:0] data
    input [31:0] instr,
    input [XLEN-1:0] pc,
    input pc_valid,
    
    // MASTER AXI (supplies data packet to FIFO)
    output wire M_AXIS_tvalid,
    input M_AXIS_tready,
    output wire [AXI_DATA_WIDTH-1:0] M_AXIS_tdata,
    output wire M_AXIS_tlast,
    input [31:0] tlast_interval // number of items in FIFO after which tlast is asserted
);
    wire drop_instr;

    // At the end of a program, a "wfi" (wait for interrupt) instruction is executed 
    // which stops the program from running. This is a good time to stop sending trace
    // to the FIFO.
    reg program_finished = 0;

    trace_filter trace_filter_inst (
        .clk(clk),
        .instr(instr),
        .drop_instr(drop_instr)
    );

    wire [AXI_DATA_WIDTH-1:0]data_pkt = {pc, instr};

    data_to_axi_stream #(
        .DATA_WIDTH(AXI_DATA_WIDTH) // pc + instr sizes
    ) data_to_axi_stream_inst (
        .clk(clk),
        .rst_n(rst_n),
        .write_enable(pc_valid & ~drop_instr),
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
            program_finished <= 0;
        end
        else begin
            if (instr == `WFI_INSTRUCTION) begin
                program_finished <= 1;
            end
            else begin
                program_finished <= program_finished;
            end
        end
    end

endmodule