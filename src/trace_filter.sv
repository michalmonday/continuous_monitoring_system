// This module checks if instruction is not a branch/jump/return.
// If that's the case, then "drop_instr" is set to 1.

`timescale 1ns/10ps

import continuous_monitoring_system_pkg::*;

module trace_filter //#(
    // parameter DATA_WIDTH = 64, // arbitrary value
    // parameter ADDR_WIDTH = 4 // internal addressing (each of 16 addresses can result in a different action upon writing/reading)
//)
(
    input clk, //rst_n, 
    input [RISC_V_INSTRUCTION_WIDTH - 1 : 0] instr,
    output wire drop_instr
);
    // always @(posedge clk) begin
    //     // compressed instructions (16-bit)
    //     if (instr[1:0] == `C_BRANCH_OPCODE && instr[15:14] == `C_BRANCH_FUNCT3_2_MSB) begin
    //         drop_instr <= 1'b0;
    //     end
    //     else if (instr[1:0] == `C_JAL_OPCODE && instr[15:13] == `C_JAL_FUNCT3_3_MSB) begin
    //         drop_instr <= 1'b0;
    //     end
    //     else if (instr[1:0] == `C_JALR_OPCODE && instr[15:13] == `C_JALR_FUNCT4_3_MSB) begin
    //         drop_instr <= 1'b0;
    //     end
    //     // not compressed instructions (32-bit)
    //     else if (instr[6:0] == `BRANCH_OPCODE) begin
    //         drop_instr <= 1'b0;
    //     end
    //     else if (instr[6:0] == `JAL_OPCODE) begin
    //         drop_instr <= 1'b0;
    //     end
    //     else if (instr[6:0] == `JALR_OPCODE) begin
    //         drop_instr <= 1'b0;
    //     end
    //     else begin
    //         drop_instr <= 1'b1;
    //     end
    // assign drop_instr = instr > 0;
    assign drop_instr = ~((instr[6:0] == BRANCH_OPCODE) || 
                        (instr[6:0] == JAL_OPCODE) || 
                        (instr[6:0] == JALR_OPCODE) ||
                        (instr[1:0] == C_BRANCH_OPCODE && instr[15:14] == C_BRANCH_FUNCT3_2_MSB) ||
                        (instr[1:0] == C_JAL_OPCODE && instr[15:13] == C_JAL_FUNCT3_3_MSB) ||
                        (instr[1:0] == C_JALR_OPCODE && instr[15:13] == C_JALR_FUNCT4_3_MSB) || 
                        (instr == WFI_INSTRUCTION)
                        );
        
    // // end
    // always @(posedge clk) begin
    //     // compressed instructions (16-bit)
    //     if (instr[1:0] == `C_BRANCH_OPCODE && instr[15:14] == `C_BRANCH_FUNCT3_2_MSB) begin
    //         drop_instr <= 1'b0;
    //     end
    //     else if (instr[1:0] == `C_JAL_OPCODE && instr[15:13] == `C_JAL_FUNCT3_3_MSB) begin
    //         drop_instr <= 1'b0;
    //     end
    //     else if (instr[1:0] == `C_JALR_OPCODE && instr[15:13] == `C_JALR_FUNCT4_3_MSB) begin
    //         drop_instr <= 1'b0;
    //     end
    //     // not compressed instructions (32-bit)
    //     else if (instr[6:0] == `BRANCH_OPCODE) begin
    //         drop_instr <= 1'b0;
    //     end
    //     else if (instr[6:0] == `JAL_OPCODE) begin
    //         drop_instr <= 1'b0;
    //     end
    //     else if (instr[6:0] == `JALR_OPCODE) begin
    //         drop_instr <= 1'b0;
    //     end
    //     else begin
    //         drop_instr <= 1'b1;
    //     end
    // end
endmodule