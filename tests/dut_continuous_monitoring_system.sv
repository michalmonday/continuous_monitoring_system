`timescale 1ns/10ps  // time-unit = 1 ns, precision = 10 ps

`define ADDR_WIDTH 8 // internal addressing (each of 256 addresses can result in a different action upon writing/reading)
`define DATA_WIDTH 64 // control data width, the functionality of the module is controlled by writing to address+data ports
import continuous_monitoring_system_pkg::*;

module dut_continuous_monitoring_system;
    // localparam XLEN = 64;
    // localparam AXI_DATA_WIDTH = 1024;
    localparam period = 10;
    localparam clk_period = 5;

    reg M_AXIS_tready = 1;
    wire M_AXIS_tvalid, M_AXIS_tlast;
    wire [AXI_DATA_WIDTH-1:0] M_AXIS_tdata;

    reg clk=0, rst_n;
    reg [XLEN-1:0] pc = 4;
    reg [31:0] instr;
    reg pc_valid=1;

    ctrl_addr_t ctrl_addr = TRIGGER_TRACE_START_ADDRESS_ENABLED;
    reg [`DATA_WIDTH-1:0]ctrl_wdata = 0;
    reg ctrl_write_enable = 0;

    reg en = 1;

    localparam INPUT_EVENT_BITMAP_WIDTH = 115;
    reg [INPUT_EVENT_BITMAP_WIDTH-1:0] performance_events = 0; // bitmap

    localparam PC_LOCATION = 115*7;
    localparam INSTR_LOCATION = PC_LOCATION + 64 + 64;
    wire [63:0]tdata_pc = M_AXIS_tdata[PC_LOCATION+63:PC_LOCATION];
    wire [31:0]tdata_instr = M_AXIS_tdata[INSTR_LOCATION+31:INSTR_LOCATION];


    continuous_monitoring_system #(
        .XLEN(XLEN),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
        .CTRL_WRITE_ENABLE_POSEDGE_TRIGGERED(1)
    ) cms (
        .clk(clk),
        .rst_n(rst_n),

        // data pkt signals (to be stored in FIFO)
        .instr(instr),
        .pc(pc),
        .pc_valid(pc_valid),

        // axi signals (interfacing with FIFO)
        .M_AXIS_tvalid(M_AXIS_tvalid), // out
        .M_AXIS_tready(M_AXIS_tready), // input
        .M_AXIS_tdata(M_AXIS_tdata),   // out
        .M_AXIS_tlast(M_AXIS_tlast),   // out
        .tlast_interval(100),           // input

        // control signals (determining operational mode of the continuous_monitoring_system)
        .ctrl_addr(ctrl_addr), 
        .ctrl_wdata(ctrl_wdata), 
        .ctrl_write_enable(ctrl_write_enable),
        .en(en),
        .performance_events(performance_events)
    );

    always 
    begin
        clk = ~clk;
        #clk_period;
    end

    reg [4:0] i = 0;
    always @ (posedge clk) begin
        pc = pc +4;
        i = i + 1;
        case (i)
            0:  begin 
                instr = 32'h00000001; // nop
                performance_events = 8'b10101010;
            end
            1:  begin 
                instr = 32'h0000006f; // riscv branch instruction
                performance_events = 8'b10101010;
            end
            2:  begin 
                instr = 32'h00000001; // nop
                performance_events = 8'b10101010;
            end
            3:  begin 
                instr = 32'h00000001; // nop
                performance_events = 8'b10101010;
            end
            4:  begin 
                instr = 32'h0C601063; // blt     a0, a1, .LBB0_2
                performance_events = 8'b10101010;
            end
            5:  begin 
                instr = 32'h00000067; // riscv jalr instruction
                performance_events = 8'b01010101;
            end
            6:  begin 
                instr = 32'h00000000; // nop
                performance_events = 8'b00000001;
            end
            7:  begin 
                instr = 32'h000000ef; // riscv jal instruction
                performance_events = 8'b00000001;
            end
            8:  begin 
                instr = 32'h00130013; // addi t0, t1, 10
                performance_events = 8'b00000001;
            end
            9:  begin 
                instr = 32'h00000000; // nop
                performance_events = 8'b00000001;
            end
            10:  begin 
                instr = 32'h0000006f; // riscv branch instruction
                performance_events = 8'b00000001;
            end
            11:  begin 
                instr = 32'h0C601063; // blt     a0, a1, .LBB0_2
            end

            12: begin 
                 instr = 32'h10500073; // WFI (wait for interrupt)
            end
            13: begin 
                 instr = 32'h10500073; // WFI (wait for interrupt)
            end

            14: begin 
                 instr = 32'h0000006f; // riscv branch instruction
            end
            15: begin 
                 instr = 32'h0C601063; // blt     a0, a1, .LBB0_2
            end
            16: begin 
                 instr = 32'h00000067; // riscv jalr instruction
            end
            17: begin 
                 instr = 32'h00000000; // nop
            end
            18: begin 
                 instr = 32'h000000ef; // riscv jal instruction
            end
            19: begin 
                 instr = 32'h00130013; // addi t0, t1, 10
            end
            20: begin 
                 instr = 32'h00000000; // nop
            end
            21: begin 
                 instr = 32'h0000006f; // riscv branch instruction
            end
            22: begin 
                 instr = 32'h0C601063; // blt     a0, a1, .LBB0_2
            end
            default: begin
                 instr = 32'h00000000; // nop
            end
        endcase
    end


// module continuous_monitoring_system #(
//     parameter XLEN = 64,
//     parameter AXI_DATA_WIDTH = XLEN + 32
//     //parameter ADDR_WIDTH = 4 // internal addressing (each of 16 addresses can result in a different action upon writing/reading)
// ) (
//     input clk, rst_n, 
//     // input [ADDR_WIDTH-1:0] addr,
//     // input [DATA_WIDTH-1:0] data
//     input [31:0] instr,
//     input [XLEN-1:0] pc,
//     input pc_valid,
    
//     // MASTER AXI (supplies data packet to FIFO)
//     output reg M_AXIS_tvalid,
//     input M_AXIS_tready,
//     output reg [DATA_WIDTH-1:0] M_AXIS_tdata,
//     output reg M_AXIS_tlast,
//     input [31:0] tlast_interval // number of items in FIFO after which tlast is asserted
// );

endmodule