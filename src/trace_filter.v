// This module checks if instruction is not a branch/jump/return.
// If that's the case, then "drop_instr" is set to 1.


// Observation: first 2 bits of instruction (opcode) indicate whether instruction is compressed (16-bit) or not (32-bit).
//              If first 2 bits are '11' then instruction is not compressed and has 7-bit opcode.
//              If first 2 bits are less than '11' then instruction is compressed and has 2-bit opcode (with 3/4 MSB funct bits specifying instruction a bit further).

// For not compressed instructions (32-bit):
`define BRANCH_OPCODE 7'b1100011
`define JAL_OPCODE 7'b1101111
`define JALR_OPCODE 7'b1100111

// For compressed instructions (16-bit):
//     https://riscv.org/wp-content/uploads/2015/11/riscv-compressed-spec-v1.9.pdf
//     https://github.com/CTSRD-CHERI/Flute/blob/CHERI/src_Core/ISA/ISA_Decls_C.bsv
//     "Digital Design and Computer Architecture - RISC-V Edition" by David Money Harris and Sarah L. Harris
`define C_BRANCH_OPCODE 2'b10
`define C_JAL_OPCODE 2'b01
`define C_JALR_OPCODE 2'b00

// 3/4 MSB funct bits indicate what kind of compressed instruction it is.
// Bit #(3) funct3_C_BEQZ               = 3'b_110;
// Bit #(3) funct3_C_BNEZ               = 3'b_111;
`define C_BRANCH_FUNCT3_2_MSB 2'b11
// Bit #(3) funct3_C_JAL                = 3'b_001;     // RV32
// Bit #(3) funct3_C_J                  = 3'b_101;
`define C_JAL_FUNCT3_3_MSB 3'b101
// Bit #(4) funct4_C_JR                 = 4'b_1000;
// Bit #(4) funct4_C_JALR               = 4'b_1001;
`define C_JALR_FUNCT4_3_MSB 3'b100


module trace_filter //#(
    // parameter DATA_WIDTH = 64, // arbitrary value
    // parameter ADDR_WIDTH = 4 // internal addressing (each of 16 addresses can result in a different action upon writing/reading)
//)
(
    input clk, //rst_n, 
    input [31:0] instr,
    output reg drop_instr
);
    always @(posedge clk) begin
        // compressed instructions (16-bit)
        if (instr[1:0] == `C_BRANCH_OPCODE && instr[15:14] == `C_BRANCH_FUNCT3_2_MSB) begin
            drop_instr <= 1'b0;
        end
        else if (instr[1:0] == `C_JAL_OPCODE && instr[15:13] == `C_JAL_FUNCT3_3_MSB) begin
            drop_instr <= 1'b0;
        end
        else if (instr[1:0] == `C_JALR_OPCODE && instr[15:13] == `C_JALR_FUNCT4_3_MSB) begin
            drop_instr <= 1'b0;
        end
        // not compressed instructions (32-bit)
        else if (instr[6:0] == `BRANCH_OPCODE) begin
            drop_instr <= 1'b0;
        end
        else if (instr[6:0] == `JAL_OPCODE) begin
            drop_instr <= 1'b0;
        end
        else if (instr[6:0] == `JALR_OPCODE) begin
            drop_instr <= 1'b0;
        end
        else begin
            drop_instr <= 1'b1;
        end
        
    end
endmodule