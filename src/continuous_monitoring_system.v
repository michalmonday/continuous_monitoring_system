
// continuous monitoring system module will allow the user to do the following:
// 1. Supply a program binary. This way trace will only consist of indirect jumps, that can't be inferred from the binary.


module continuous_monitoring_system #(
    parameter AXI_DATA_WIDTH = 32, 
    parameter AXI_ADDR_WIDTH = 4 // internal addressing (each of 16 addresses can result in a different action upon writing/reading)
) (
    input clk, rst_n, 
    input [AXI_ADDR_WIDTH-1:0] addr,
    input [AXI_DATA_WIDTH-1:0] data
);

endmodule