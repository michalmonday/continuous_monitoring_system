// This module checks if instruction is not a branch/jump/return.
// If that's the case, then "drop_instr" is set to 1.
// Optional functionality of sending a second instruction proceeding a branch/jump/return.

`timescale 1ns/10ps

import continuous_monitoring_system_pkg::*;

module trace_filter #(
    parameter SEND_INSTRUCTION_AFTER_BRANCH = 1,
    parameter SEND_INSTRUCTION_AFTER_JUMP = 1,
    parameter SEND_INSTRUCTION_AFTER_WFI = 1
) 
(
    input   logic                                       clk, 
    input   logic                                       rst_n,
    input   logic                                       pc_valid,
    input   logic   [RISC_V_INSTRUCTION_WIDTH - 1 : 0]  next_instr,
    output  logic                                       drop_instr
);

    logic branch;
    logic jump;
    logic wfi;

    reg queue_instruction = 1'b0;
    reg send_next_instruction = 1'b0;
    
    // assign branch = (next_instr[6:0] == BRANCH_OPCODE) ||
    //                 (next_instr[1:0] == C_BRANCH_OPCODE && next_instr[15:14] == C_BRANCH_FUNCT3_2_MSB);


    // assign jump =   (next_instr[6:0] == JAL_OPCODE) || 
    //                 (next_instr[6:0] == JALR_OPCODE) ||
    //                 (next_instr[1:0] == C_JAL_OPCODE && next_instr[15:13] == C_JAL_FUNCT3_3_MSB) ||
    //                 (next_instr[1:0] == C_JALR_OPCODE && next_instr[15:13] == C_JALR_FUNCT4_3_MSB);

    // assign wfi =    (next_instr == WFI_INSTRUCTION);


    always_ff @(posedge clk) begin // Sends the instruction proceeding a branch/jump/return if the corresponding parameter is set.
        if (rst_n == 0) begin
            queue_instruction <= 1'b0;
            send_next_instruction <= 1'b0;
        end else if (pc_valid) begin
            branch <= (next_instr[6:0] == BRANCH_OPCODE) ||
                     (next_instr[1:0] == C_BRANCH_OPCODE && next_instr[15:14] == C_BRANCH_FUNCT3_2_MSB);
            jump <=  (next_instr[6:0] == JAL_OPCODE) || 
                     (next_instr[6:0] == JALR_OPCODE) ||
                     (next_instr[1:0] == C_JAL_OPCODE && next_instr[15:13] == C_JAL_FUNCT3_3_MSB) ||
                     (next_instr[1:0] == C_JALR_OPCODE && next_instr[15:13] == C_JALR_FUNCT4_3_MSB);


            wfi <= (next_instr == WFI_INSTRUCTION);


            queue_instruction <= (branch && SEND_INSTRUCTION_AFTER_BRANCH) || (jump && SEND_INSTRUCTION_AFTER_JUMP) || (wfi && SEND_INSTRUCTION_AFTER_WFI);
            send_next_instruction <= queue_instruction;
        end
    end
        

    assign drop_instr = ~(branch || jump  || wfi || send_next_instruction);
        
    
endmodule