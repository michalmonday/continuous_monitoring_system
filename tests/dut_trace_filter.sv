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

    logic   [RISC_V_INSTRUCTION_WIDTH - 1 : 0]          instr;

    logic                                               drop_instr;

    reg [RISC_V_INSTRUCTION_WIDTH - 1 : 0]  memory  [NUMBER_OF_INSTRUCTIONS];
    integer i;

    trace_filter dut (
        .clk(clk),
        .rst_n(rst_n),
        .instr(instr),
        .drop_instr(drop_instr));

    
    // Clocking
    localparam period = 5;
    always begin
        #period clk = !clk;
    end

    // Simulation begins
    initial begin
        clk = 1'b0;
        rst_n = 1'b1;

        instr = RISC_V_INSTRUCTION_WIDTH'('b0);

        $readmemh(MEM_FILE_NAME, memory);
        $display("Contents of memory:");
        for (i = 0; i < 132; i = i + 1) $display("%d:%h", i, memory[i]);

        for (i = 0; i < 132; i = i + 1)
        begin
            instr = memory[i];
            #period;
            #period;
        end
    end

    

endmodule