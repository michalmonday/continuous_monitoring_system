// If simulating in Vivado make sure to set simulation runtime to at least 1500ns (xsim.simulate.runtime)

`timescale 1 ns/10 ps  // time-unit = 1 ns, precision = 10 ps
import continuous_monitoring_system_pkg::*;

module dut_trace_filter;

    // From program_statistics.txt
    localparam MEM_FILE_NAME = "riscv-example-cheri.mem";
    localparam NUMBER_OF_INSTRUCTIONS = 132;
    localparam NUMBER_OF_BRANCHES = 2;
    int  branch_instruction_array[NUMBER_OF_BRANCHES] = '{61, 106};



    logic                                       clk;
    logic                                       rst_n;
    logic                                       pc_valid;

    logic   [RISC_V_INSTRUCTION_WIDTH - 1 : 0]  instr;

    logic                                       drop_instr;

    reg [RISC_V_INSTRUCTION_WIDTH - 1 : 0]      memory  [NUMBER_OF_INSTRUCTIONS];
    integer i;

    trace_filter #(
        .SEND_INSTRUCTION_AFTER_BRANCH(1),
        .SEND_INSTRUCTION_AFTER_JUMP(0),
        .SEND_INSTRUCTION_AFTER_WFI(1)
    ) dut 
    (
        .clk(clk),
        .rst_n(rst_n),
        .pc_valid(pc_valid),
        .instr(instr),
        .drop_instr(drop_instr));

    
    // Clocking
    localparam half_period = 5;
    localparam period = 2 * half_period;
    always begin
        #half_period clk = !clk;
    end



    // Simulation begins
    initial begin
        clk = 1'b1;
        pc_valid = 1'b1;
        rst_n = 1'b1;

        instr = RISC_V_INSTRUCTION_WIDTH'('b0);

        $readmemh(MEM_FILE_NAME, memory);
        $display("Contents of memory:");
        for (i = 0; i < 132; i = i + 1) $display("%d:%h", i, memory[i]);

        for (i = 0; i < 132; i = i + 1)
        begin
            instr = memory[i];
            #period;
        end

        // Testing dropping a second instruction functionality

        pc_valid = 1'b1;
        instr = 32'h00029663; //  Branch instruction;

        #period; // 1 Clock cycle

        pc_valid = 1'b0;

        #period; // 3 Clock cycles
        #period;
        #period;

        pc_valid = 1'b1;
        instr = 32'b00000000000100110000000000010011; // Add Instruction

        #period;

        pc_valid = 1'b0;

        #period; // 2 Clock Cycles
        #period;

        pc_valid = 1'b1; // Still   Add Instruction

        #period;

        pc_valid = 1'b0;

        #period; // 1 Clock cycle

        pc_valid = 1'b1;
        instr = 32'h00000067; // JALR Instruction

        #period;

        $stop;
    end

    

endmodule