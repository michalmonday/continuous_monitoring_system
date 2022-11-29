// Observation: first 2 bits of instruction (opcode) indicate whether instruction is compressed (16-bit) or not (32-bit).
//              If first 2 bits are '11' then instruction is not compressed and has 7-bit opcode.
//              If first 2 bits are less than '11' then instruction is compressed and has 2-bit opcode.

`define BRANCH_OPCODE 1100011
`define JAL_OPCODE 1101111
`define JALR_OPCODE 1100111

`define C_BRANCH_OPCODE 110
`define C_JAL_OPCODE 001
`define C_JALR_OPCODE 000


module trace_filter //#(
    // parameter DATA_WIDTH = 64, // arbitrary value
    // parameter ADDR_WIDTH = 4 // internal addressing (each of 16 addresses can result in a different action upon writing/reading)
//)
 (
    input clk, rst_n, 
    input [31:0] instr,
    output reg drop_instr
 );

endmodule