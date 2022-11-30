`timescale 1ns/1ps  // time-unit = 1 ns, precision = 10 ps

module dut_continuous_monitoring_system;
    localparam XLEN = 64;
    localparam AXI_DATA_WIDTH = 64 + 32;
    localparam period = 10;
    localparam clk_period = 5;

    reg M_AXIS_tready = 1;
    wire M_AXIS_tvalid, M_AXIS_tlast;
    wire [AXI_DATA_WIDTH-1:0] M_AXIS_tdata;

    reg clk=0, rst_n=1;
    reg [XLEN-1:0] pc = 4;
    reg [31:0] instr;
    reg pc_valid=1;

    continuous_monitoring_system #(
        .XLEN(XLEN),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH)
    ) cms (
        .clk(clk),
        .rst_n(rst_n),
        .instr(instr),
        .pc(pc),
        .pc_valid(pc_valid),
        .M_AXIS_tvalid(M_AXIS_tvalid),
        .M_AXIS_tready(M_AXIS_tready),
        .M_AXIS_tdata(M_AXIS_tdata),
        .M_AXIS_tlast(M_AXIS_tlast),
        .tlast_interval(3)
    );

    initial
    begin

        instr =  32'h00000000;
        #period; 

        // riscv branch instruction
        instr =  32'h0000006f;
        #period;

        // blt     a0, a1, .LBB0_2
        instr = 32'b1100011000000001000001100011;
        #period;

        // riscv jalr instruction
        instr =  32'h00000067;
        #period;

        instr =  32'h00000000;
        #period; 

        // riscv jal instruction
        instr =  32'h000000ef;
        #period;

        // addi t0, t1, 10
        instr = 32'b00000000000100110000000000010011;
        #period;

        instr =  32'h00000000;
        #period; 

        // riscv branch instruction
        instr =  32'h0000006f;
        #period;

        // blt     a0, a1, .LBB0_2
        instr = 32'b1100011000000001000001100011;
        #period;

        // riscv jalr instruction
        instr =  32'h00000067;
        #period;

        // riscv jal instruction
        instr =  32'h000000ef;
        #period;

        // addi t0, t1, 10
        instr = 32'b00000000000100110000000000010011;
        #period;
    end 

    always 
    begin
        clk = ~clk;
        #clk_period;
    end

    always @ (posedge clk) begin
        pc = pc +4;
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