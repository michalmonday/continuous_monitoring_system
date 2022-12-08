`timescale 1ns/10ps  // time-unit = 1 ns, precision = 10 ps

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

module dut_continuous_monitoring_system_control;
    localparam XLEN = 64;
    localparam AXI_DATA_WIDTH = 64 + 32;
    localparam period = 10;
    localparam clk_period = 5;

    reg M_AXIS_tready = 1;
    wire M_AXIS_tvalid, M_AXIS_tlast;
    wire [AXI_DATA_WIDTH-1:0] M_AXIS_tdata;

    reg clk=0, rst_n;
    reg [XLEN-1:0] pc = 'h80000000;
    reg [31:0] instr;
    reg pc_valid=1;

    reg [`ADDR_WIDTH-1:0]ctrl_addr = 0;
    reg [`DATA_WIDTH-1:0]ctrl_wdata = 0;
    reg ctrl_write_enable = 0;


    // just for simulation
    reg ctrl_initialized = 0;

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
        .ctrl_write_enable(ctrl_write_enable)
    );


    always 
    begin
        clk = ~clk;
        #clk_period;
    end

    reg [5:0] j = 0;
    always @ (posedge clk) begin
        if (~ctrl_initialized && j[0] == 1) begin
            case (j)
                1: begin
                    ctrl_addr = `ADDR_TRIGGER_TRACE_START_ADDRESS_ENABLED;
                    ctrl_wdata = 1;
                end
                3: begin
                    ctrl_addr = `ADDR_TRIGGER_TRACE_END_ADDRESS_ENABLED;
                    ctrl_wdata = 1;
                end
                5: begin
                    ctrl_addr = `ADDR_TRIGGER_TRACE_START_ADDRESS;
                    ctrl_wdata = 32'h80000008;
                end
                7: begin 
                    ctrl_addr = `ADDR_TRIGGER_TRACE_END_ADDRESS;
                    ctrl_wdata = 32'h80000020;
                end
                17: begin
                    ctrl_initialized = 1;
                end
            endcase
            ctrl_write_enable = 1;
        end
        else begin
            ctrl_write_enable = 0;
        end
        j = j + 1;
    end

    reg [4:0] i = 0;
    always @ (posedge clk) begin
        if (ctrl_initialized) begin
            pc = pc + 4;
            i = i + 1;
        end 
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
endmodule