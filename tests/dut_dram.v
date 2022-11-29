`timescale 1 ns/10 ps  // time-unit = 1 ns, precision = 10 ps

module dut_dram;

    // duration for each bit = 5 * timescale = 5 * 1 ns  = 5ns
    localparam clk_period = 5;
    localparam period = clk_period*2;  

//     module dram #(
//     parameter WORD_SIZE = 64,
//     parameter ADDR_SIZE = 8,
//     parameter WORD_CAPACITY = 2**ADDR_SIZE
// ) (
//     input clk,
// 	input write_enable_1,
//     input write_enable_2, 
// 	input [WORD_SIZE-1:0] data_in_1, 
//     input [WORD_SIZE-1:0] data_in_2,
// 	input [ADDR_SIZE-1:0] address_1, 
//     input [ADDR_SIZE-1:0] address_2,
// 	output reg [WORD_SIZE-1:0] output_1, 
//     output reg [WORD_SIZE-1:0] output_2
// );


    localparam WORD_SIZE = 64;
    localparam ADDR_SIZE = 8;

    reg clk = 0;
    reg write_enable_1=0, write_enable_2=0;
    reg [WORD_SIZE-1:0] data_in_1=0, data_in_2=0;
    reg [ADDR_SIZE-1:0] address_1=0, address_2=0;

    wire [WORD_SIZE-1:0] output_1, output_2;

    dram #(.WORD_SIZE(WORD_SIZE), .ADDR_SIZE(ADDR_SIZE)) mem (
     .clk(clk),
     .write_enable_1(write_enable_1),
     .write_enable_2(write_enable_2),
     .data_in_1(data_in_1),
     .data_in_2(data_in_2),
     .address_1(address_1),
     .address_2(address_2),
     .output_1(output_1),
     .output_2(output_2));

    initial 
    begin
        #clk_period;

        data_in_1 = 64'h1234567890abcdef;
        write_enable_1 = 1;
        #period;
        write_enable_1 = 0;
        #period;

        address_2 = 8'h02;
        data_in_2 = 64'hdeadbeefabcd1234;
        write_enable_2 = 1;
        #period;
        write_enable_2 = 0;

        #period;#period;
        address_1 = 8'h02;
    end
    always 
    begin
        clk = 0;
        #clk_period;
        clk = 1;
        #clk_period;
    end
endmodule