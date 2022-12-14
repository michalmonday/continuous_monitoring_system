package continuous_monitoring_system_pkg;

// NOTE: KEEP IN MIND TO UPDATE cms_ip_wrapper.v DEFINITIONS AS WELL (WHEN CHANGING PARAMETERS HERE)
//       it has duplicate definitions because Vivado can't use SystemVerilog as top level file


// RISC-V standard widths
parameter XLEN = 64;
parameter RISC_V_INSTRUCTION_WIDTH = 32;

parameter AXI_DATA_WIDTH = 512;

// Instruction opcodes
parameter BRANCH_OPCODE = 7'b1100011;
parameter JAL_OPCODE = 7'b1101111;
parameter JALR_OPCODE = 7'b1100111;

// Compressed Instruction opcodes
parameter C_BRANCH_OPCODE = 2'b10;
parameter C_JAL_OPCODE = 2'b01;
parameter C_JALR_OPCODE = 2'b00;
parameter C_BRANCH_FUNCT3_2_MSB = 2'b11;
parameter C_JAL_FUNCT3_3_MSB = 3'b101;
parameter C_JALR_FUNCT4_3_MSB = 3'b100;

// Used by trace_filter to resynchronize after an arbitrary amount of time
parameter RESYNC_TIMER_WIDTH = 8;
parameter RESYNC_TIMER_RESET_VALUE = 8'b00001010;

// Used by continuous monitoring system
parameter WFI_INSTRUCTION = 32'h10500073;
parameter CTRL_ADDR_WIDTH = 8; // internal addressing (each of 256 addresses can result in a different action upon writing/reading)
parameter CTRL_DATA_WIDTH = 64; // control data width, the functionality of the module is controlled by writing to address+data ports

typedef enum logic [CTRL_ADDR_WIDTH - 1 : 0] { // control register addresses
    TRIGGER_TRACE_START_ADDRESS_ENABLED,
    TRIGGER_TRACE_END_ADDRESS_ENABLED,
    TRIGGER_TRACE_START_ADDRESS,
    TRIGGER_TRACE_END_ADDRESS,
    MONITORED_ADDRESS_RANGE_LOWER_BOUND_ENABLED,
    MONITORED_ADDRESS_RANGE_UPPER_BOUND_ENABLED,
    MONITORED_ADDRESS_RANGE_LOWER_BOUND,
    MONITORED_ADDRESS_RANGE_UPPER_BOUND,
    WFI_STOPPED,
    CLK_COUNTER,
    LAST_WRITE_TIMESTAMP
    } ctrl_addr_t;

parameter CLK_COUNTER_WIDTH = 64;
//parameter NO_OF_PERFORMANCE_EVENTS = 115;
parameter NO_OF_PERFORMANCE_EVENTS = 39;

parameter PERFORMANCE_EVENT_MOD_COUNTER_WIDTH = 7;

endpackage