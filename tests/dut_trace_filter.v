`timescale 1ns/10ps  // time-unit = 1 ns, precision = 10 ps

module dut_trace_filter;
    localparam clk_period = 5;
    localparam period = clk_period*2;  
    
    wire drop_instr;
    reg clk = 1;
    reg [31:0] instr = 0;

    trace_filter tf (.clk(clk), .instr(instr), .drop_instr(drop_instr));
    
    initial 
        begin
            #clk_period;
            
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
        clk = 1;
        #clk_period;
        clk = 0;
        #clk_period;
    end
endmodule
