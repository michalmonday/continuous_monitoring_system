/* 
This wrapper was created because there is no way to use SystemVerilog as a module
in Vivado, it is also not possible to use IP where the top level module is written
in SystemVerilog. This is just a workaround.
*/
`define CTRL_ADDR_WIDTH 8 // internal addressing (each of 256 addresses can result in a different action upon writing/reading)
`define CTRL_DATA_WIDTH 64 // control data width, the functionality of the module is controlled by writing to address+data ports
`define NO_OF_PERFORMANCE_EVENTS 37
`define XLEN 64
`define AXI_DATA_WIDTH 512


module cms_ip_wrapper #(
    parameter CTRL_WRITE_ENABLE_POSEDGE_TRIGGERED = 1 // 1 = write enable is pos edge triggered, 0 = write enable is level triggered
    //parameter CTRL_ADDR_WIDTH = 4 // internal addressing (each of 16 addresses can result in a different action upon writing/reading)
) (
    input clk, rst_n, 

    // data pkt signals (to be stored in FIFO)
    input [31:0] instr,
    input [`XLEN-1:0] pc,
    // input pc_valid, // determines whether the current instruction/pc is executed now

    // axi signals (interfacing with FIFO)
    output wire M_AXIS_tvalid,
    input M_AXIS_tready,
    output wire [`AXI_DATA_WIDTH - 1 : 0] M_AXIS_tdata,
    output wire M_AXIS_tlast,
    input [31:0] tlast_interval, // number of items in FIFO after which tlast is asserted

    // control signals (determining operational mode of the continuous_monitoring_system)
    input [`CTRL_ADDR_WIDTH-1:0] ctrl_addr,
    input [`CTRL_DATA_WIDTH-1:0] ctrl_wdata,
    input ctrl_write_enable,

    // enable the module (if disabled, the module will not send any data to the FIFO)
    // this may be connected to the GPIO rst_n (the same one used to reset the processor)
    input en,
    input [`NO_OF_PERFORMANCE_EVENTS-1:0]performance_events,
    output wire trace_filter_pc_valid_probe,
    output wire [31:0] trace_filter_instr_probe,
    output wire trace_filter_drop_instr_probe,
    output wire data_to_axi_write_enable_probe,
    output wire data_to_axi_write_enable_pc_valid_probe,
    output wire [`XLEN-1:0] data_pkt_pc_probe,
    output wire [31:0] data_pkt_instr_probe
);

continuous_monitoring_system #(
    .CTRL_WRITE_ENABLE_POSEDGE_TRIGGERED(1) 
) cms (
    .clk(clk), .rst_n(rst_n), 

    // data pkt signals (to be stored in FIFO)
    .instr(instr),
    .pc(pc),
    // .pc_valid(pc_valid), // determines whether the current instruction/pc is executed now


    // axi signals (interfacing with FIFO)
    .M_AXIS_tvalid(M_AXIS_tvalid),
    .M_AXIS_tready(M_AXIS_tready),
    .M_AXIS_tdata(M_AXIS_tdata),
    .M_AXIS_tlast(M_AXIS_tlast),
    .tlast_interval(tlast_interval), // number of items in FIFO after which tlast is asserted

    // control signals (determining operational mode of the continuous_monitoring_system)
    .ctrl_addr(ctrl_addr),
    .ctrl_wdata(ctrl_wdata),
    .ctrl_write_enable(ctrl_write_enable),

    // enable the module (if disabled, the module will not send any data to the FIFO)
    // this may be connected to the GPIO rst_n (the same one used to reset the processor)
    .en(en),
    .performance_events(performance_events),

    .trace_filter_pc_valid_probe(trace_filter_pc_valid_probe),
    .trace_filter_instr_probe(trace_filter_instr_probe),
    .trace_filter_drop_instr_probe(trace_filter_drop_instr_probe),
    .data_to_axi_write_enable_probe(data_to_axi_write_enable_probe),
    .data_to_axi_write_enable_pc_valid_probe(data_to_axi_write_enable_pc_valid_probe),
    .data_pkt_pc_probe(data_pkt_pc_probe),
    .data_pkt_instr_probe(data_pkt_instr_probe)
);

endmodule
