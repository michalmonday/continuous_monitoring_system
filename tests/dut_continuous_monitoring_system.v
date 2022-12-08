`timescale 1ns/10ps  // time-unit = 1 ns, precision = 10 ps

module dut_continuous_monitoring_system;
    localparam XLEN = 64;
    localparam AXI_DATA_WIDTH = 64 + 32;
    localparam period = 10;
    localparam clk_period = 5;

    reg M_AXIS_tready = 1;
    wire M_AXIS_tvalid, M_AXIS_tlast;
    wire [AXI_DATA_WIDTH-1:0] M_AXIS_tdata;

    reg clk=0, rst_n;
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
        .tlast_interval(100)
    );

    always 
    begin
        clk = ~clk;
        #clk_period;
    end

    reg [3:0] i = 0;
    always @ (posedge clk) begin
        pc = pc +4;
        i = i + 1;
        case (i)
            0: instr = 32'h00000000; // nop
            1: instr = 32'h0000006f; // riscv branch instruction
            2: instr = 32'b1100011000000001000001100011; // blt     a0, a1, .LBB0_2
            3: instr = 32'h00000067; // riscv jalr instruction
            4: instr = 32'h00000000; // nop
            5: instr = 32'h000000ef; // riscv jal instruction
            6: instr = 32'b00000000000100110000000000010011; // addi t0, t1, 10
            7: instr = 32'h00000000; // nop
            8: instr = 32'h0000006f; // riscv branch instruction
            9: instr = 32'b1100011000000001000001100011; // blt     a0, a1, .LBB0_2
            10: instr = 32'h00000001; // WFI
            11: instr = 32'h0000006f; // riscv branch instruction
            12: instr = 32'b1100011000000001000001100011; // blt     a0, a1, .LBB0_2
            13: instr = 32'h00000067; // riscv jalr instruction
            14: instr = 32'h00000000; // nop
            15: instr = 32'h000000ef; // riscv jal instruction
            16: instr = 32'b00000000000100110000000000010011; // addi t0, t1, 10
            17: instr = 32'h00000000; // nop
            18: instr = 32'h0000006f; // riscv branch instruction
            19: instr = 32'b1100011000000001000001100011; // blt     a0, a1, .LBB0_2
            default: instr = 32'h00000000; // nop
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