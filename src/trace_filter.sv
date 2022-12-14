import continuous_monitoring_system_pkg::*;

module trace_filter (
    clk,
    rst_n,

    instr,
    drop_instr
);

    input   logic   clk;
    input   logic   rst_n;

    // Instruction data from CPU
    input   logic   [RISC_V_INSTRUCTION_WIDTH - 1 : 0]  instr;
    output  logic                                       drop_instr;   


    logic   [RESYNC_TIMER_WIDTH - 1 : 0] resync_timer = RESYNC_TIMER_WIDTH'('b0);
    
    logic   resync;

    // Different event variables
    logic   resync_timer_event  = 1'b0;
    logic   branch_event        = 1'b0;
    logic   jal_event           = 1'b0;
    logic   jalr_event          = 1'b0;
    logic   c_branch_event      = 1'b0;
    logic   c_jal_event         = 1'b0;
    logic   c_jalr_event        = 1'b0;
    logic   wfi_event           = 1'b0;


    // This handles resynchronisation after a certain amount of time
    always_ff @(posedge clk) begin : resync_timer_block
        if ((rst_n == 0) || resync) begin
            resync_timer <= RESYNC_TIMER_WIDTH'('b0);
        end else begin
            resync_timer <= resync_timer + 1;
        end
    end


    // This handles determining when an event is triggered
    always_ff @(posedge clk) begin : pipeline_stage_1
        if (rst_n == 0) begin
            resync_timer_event  <= 1'b0;
            branch_event        <= 1'b0;
            jal_event           <= 1'b0;
            jalr_event          <= 1'b0;
            c_branch_event      <= 1'b0;
            c_jal_event         <= 1'b0;
            c_jalr_event        <= 1'b0;
            wfi_event           <= 1'b0;
        end else begin
            resync_timer_event  <= resync_timer == RESYNC_TIMER_RESET_VALUE;
            branch_event        <= instr[6:0]   == BRANCH_OPCODE;
            jal_event           <= instr[6:0]   == JAL_OPCODE;
            jalr_event          <= instr[6:0]   == JALR_OPCODE;
            c_branch_event      <= instr[1:0]   == C_BRANCH_OPCODE  && instr[15:14] == C_BRANCH_FUNCT3_2_MSB;
            c_jal_event         <= instr[1:0]   == C_JAL_OPCODE     && instr[15:13] == C_JAL_FUNCT3_3_MSB;
            c_jalr_event        <= instr[1:0]   == C_JALR_OPCODE    && instr[15:13] == C_JALR_FUNCT4_3_MSB;
            wfi_event           <= instr        == WFI_INSTRUCTION;

        end
    end
    
    
    // This handles when a resync is triggered
    assign resync =     resync_timer_event || 
                        branch_event ||
                        jal_event ||
                        jalr_event ||   
                        c_branch_event ||
                        c_jal_event ||
                        c_jalr_event ||
                        wfi_event;

    
    // This handles when data_pkt are sent out of the trace port
    always_ff @(posedge clk) begin : pipeline_stage_2
        if (rst_n == 0) begin
            drop_instr <= 1'b0;
        end else begin
            drop_instr <= ~resync;
        end
    end

endmodule