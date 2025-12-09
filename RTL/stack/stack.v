// ============================================================================
// Stack Operations Module
// ============================================================================
// This module handles all stack operations (PUSH, POP, CALL, RET, RTI)
// Stack grows downward from address 255
// R3 is used as Stack Pointer (SP)

module stack_controller (
    input wire clk,                    // Clock signal
    input wire reset,                  // Reset signal
    
    // Stack operation control
    input wire stack_push,             // 1 = perform PUSH operation
    input wire stack_pop,              // 1 = perform POP operation
    input wire [7:0] push_data,        // Data to push onto stack
    output reg [7:0] pop_data,         // Data popped from stack
    
    // Stack Pointer (SP) interface
    input wire [7:0] sp_in,            // Current SP value (R3)
    output reg [7:0] sp_out,           // Updated SP value
    output reg sp_write_enable,        // 1 = update R3 with sp_out
    
    // Memory interface
    output reg mem_read,               // Memory read request
    output reg mem_write,              // Memory write request
    output reg [7:0] mem_address,      // Memory address
    output reg [7:0] mem_data_out,     // Data to write to memory
    input wire [7:0] mem_data_in,      // Data read from memory
    
    // Status signals
    output reg stack_busy,             // 1 = stack operation in progress
    output reg stack_overflow,         // 1 = stack overflow error
    output reg stack_underflow         // 1 = stack underflow error
);

    // ========================================================================
    // Constants
    // ========================================================================
    parameter STACK_INITIAL_VALUE = 8'hFF;  // Initial SP = 255
    parameter STACK_MIN = 8'h00;             // Minimum stack address
    parameter STACK_MAX = 8'hFF;             // Maximum stack address
    
    // ========================================================================
    // Stack Operation State Machine
    // ========================================================================
    // States for multi-cycle stack operations
    localparam IDLE = 2'b00;
    localparam PUSH_WRITE = 2'b01;
    localparam POP_READ = 2'b10;
    localparam UPDATE_SP = 2'b11;
    
    reg [1:0] state, next_state;
    
    // ========================================================================
    // Internal Registers
    // ========================================================================
    reg [7:0] sp_temp;                 // Temporary SP storage
    reg [7:0] data_temp;               // Temporary data storage
    
    // ========================================================================
    // State Machine - Sequential Logic
    // ========================================================================
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            sp_temp <= STACK_INITIAL_VALUE;
            stack_overflow <= 0;
            stack_underflow <= 0;
        end else begin
            state <= next_state;
        end
    end
    
    // ========================================================================
    // State Machine - Combinational Logic (Next State + Outputs)
    // ========================================================================
    always @(*) begin
        // Default values
        next_state = IDLE;
        mem_read = 0;
        mem_write = 0;
        mem_address = 8'h00;
        mem_data_out = 8'h00;
        sp_write_enable = 0;
        sp_out = sp_in;
        pop_data = 8'h00;
        stack_busy = 0;
        
        case (state)
            // ================================================================
            // IDLE State - Wait for stack operation request
            // ================================================================
            IDLE: begin
                stack_busy = 0;
                
                if (stack_push) begin
                    // PUSH Operation: X[SP--] <- data
                    // Step 1: Write data to memory at SP
                    // Step 2: Decrement SP
                    
                    // Check for overflow (SP at minimum)
                    if (sp_in == STACK_MIN) begin
                        stack_overflow = 1;
                        next_state = IDLE;
                    end else begin
                        mem_address = sp_in;           // Write to current SP
                        mem_data_out = push_data;      // Data to push
                        mem_write = 1;                 // Enable write
                        sp_temp = sp_in - 1;           // Calculate new SP
                        next_state = PUSH_WRITE;
                    end
                    
                end else if (stack_pop) begin
                    // POP Operation: data <- X[++SP]
                    // Step 1: Increment SP
                    // Step 2: Read data from memory at new SP
                    
                    // Check for underflow (SP at maximum)
                    if (sp_in == STACK_MAX) begin
                        stack_underflow = 1;
                        next_state = IDLE;
                    end else begin
                        sp_temp = sp_in + 1;           // Calculate new SP
                        mem_address = sp_temp;         // Read from new SP
                        mem_read = 1;                  // Enable read
                        next_state = POP_READ;
                    end
                    
                end else begin
                    next_state = IDLE;
                end
            end
            
            // ================================================================
            // PUSH_WRITE State - Complete PUSH operation
            // ================================================================
            PUSH_WRITE: begin
                stack_busy = 1;
                // Write completed, now update SP
                sp_out = sp_temp;              // New SP value
                sp_write_enable = 1;           // Signal to update R3
                next_state = IDLE;
                
                // Display for debugging
                $display("Time=%0t: PUSH - Data=0x%h pushed to Address=0x%h, New SP=0x%h", 
                         $time, mem_data_out, sp_in, sp_temp);
            end
            
            // ================================================================
            // POP_READ State - Complete POP operation
            // ================================================================
            POP_READ: begin
                stack_busy = 1;
                // Read data from memory
                pop_data = mem_data_in;        // Data popped from stack
                sp_out = sp_temp;              // New SP value
                sp_write_enable = 1;           // Signal to update R3
                next_state = IDLE;
                
                // Display for debugging
                $display("Time=%0t: POP - Data=0x%h popped from Address=0x%h, New SP=0x%h", 
                         $time, mem_data_in, sp_temp, sp_temp);
            end
            
            // ================================================================
            // Default State
            // ================================================================
            default: begin
                next_state = IDLE;
            end
        endcase
    end
    
    // ========================================================================
    // Stack Depth Monitor (Optional - for debugging)
    // ========================================================================
    wire [7:0] stack_depth;
    assign stack_depth = STACK_MAX - sp_in;
    
    // Warning when stack is getting full
    always @(posedge clk) begin
        if (stack_depth > 8'd200) begin
            $display("WARNING: Stack depth = %d (nearly full!)", stack_depth);
        end
    end

endmodule


// ============================================================================
// Stack Operations Wrapper for Processor Integration
// ============================================================================
// This module integrates stack operations with the processor's register file
// and instruction decoder

module stack_unit (
    input wire clk,
    input wire reset,
    
    // Instruction decode signals
    input wire is_push,                // PUSH instruction detected
    input wire is_pop,                 // POP instruction detected
    input wire is_call,                // CALL instruction detected
    input wire is_ret,                 // RET instruction detected
    input wire is_rti,                 // RTI instruction detected
    input wire is_interrupt,           // Interrupt occurred
    
    // Register file interface
    input wire [7:0] rb_data,          // Data from R[rb] for PUSH
    input wire [7:0] sp_current,       // Current SP value (R3)
    input wire [7:0] pc_current,       // Current PC value
    input wire [3:0] flags_current,    // Current flags (for interrupt)
    output reg [7:0] rb_data_out,      // Data for R[rb] from POP
    output reg [7:0] sp_updated,       // Updated SP value
    output reg [7:0] pc_restored,      // Restored PC (from RET/RTI)
    output reg [3:0] flags_restored,   // Restored flags (from RTI)
    output reg update_rb,              // 1 = write rb_data_out to R[rb]
    output reg update_sp,              // 1 = write sp_updated to R3
    output reg update_pc,              // 1 = write pc_restored to PC
    output reg update_flags,           // 1 = restore flags
    
    // Memory interface
    output wire mem_read,
    output wire mem_write,
    output wire [7:0] mem_address,
    output wire [7:0] mem_data_out,
    input wire [7:0] mem_data_in,
    
    // Status
    output wire stack_busy,
    output wire stack_error
);

    // ========================================================================
    // Internal Signals
    // ========================================================================
    reg stack_push_req;
    reg stack_pop_req;
    reg [7:0] push_data_sel;
    wire [7:0] pop_data;
    wire [7:0] sp_out;
    wire sp_write_en;
    wire stack_overflow;
    wire stack_underflow;
    
    assign stack_error = stack_overflow | stack_underflow;
    
    // State machine for multi-cycle operations (CALL, RET, RTI, INTERRUPT)
    localparam IDLE = 3'b000;
    localparam CALL_PUSH = 3'b001;
    localparam RET_POP = 3'b010;
    localparam RTI_POP_PC = 3'b011;
    localparam RTI_POP_FLAGS = 3'b100;
    localparam INT_PUSH_FLAGS = 3'b101;
    localparam INT_PUSH_PC = 3'b110;
    
    reg [2:0] state, next_state;
    
    // ========================================================================
    // Instantiate Stack Controller
    // ========================================================================
    stack_controller stack_ctrl (
        .clk(clk),
        .reset(reset),
        .stack_push(stack_push_req),
        .stack_pop(stack_pop_req),
        .push_data(push_data_sel),
        .pop_data(pop_data),
        .sp_in(sp_current),
        .sp_out(sp_out),
        .sp_write_enable(sp_write_en),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .mem_address(mem_address),
        .mem_data_out(mem_data_out),
        .mem_data_in(mem_data_in),
        .stack_busy(stack_busy),
        .stack_overflow(stack_overflow),
        .stack_underflow(stack_underflow)
    );
    
    // ========================================================================
    // State Machine for Complex Stack Operations
    // ========================================================================
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end
    
    always @(*) begin
        // Default values
        next_state = IDLE;
        stack_push_req = 0;
        stack_pop_req = 0;
        push_data_sel = 8'h00;
        update_rb = 0;
        update_sp = 0;
        update_pc = 0;
        update_flags = 0;
        rb_data_out = 8'h00;
        sp_updated = sp_current;
        pc_restored = 8'h00;
        flags_restored = 4'h0;
        
        case (state)
            IDLE: begin
                // Simple PUSH (PUSH Rb)
                if (is_push && !stack_busy) begin
                    stack_push_req = 1;
                    push_data_sel = rb_data;
                    update_sp = sp_write_en;
                    sp_updated = sp_out;
                    next_state = IDLE;
                end
                
                // Simple POP (POP Rb)
                else if (is_pop && !stack_busy) begin
                    stack_pop_req = 1;
                    rb_data_out = pop_data;
                    update_rb = sp_write_en;
                    update_sp = sp_write_en;
                    sp_updated = sp_out;
                    next_state = IDLE;
                end
                
                // CALL: Push PC+1, then jump
                else if (is_call && !stack_busy) begin
                    stack_push_req = 1;
                    push_data_sel = pc_current + 1;  // Return address
                    next_state = CALL_PUSH;
                end
                
                // RET: Pop PC
                else if (is_ret && !stack_busy) begin
                    stack_pop_req = 1;
                    next_state = RET_POP;
                end
                
                // RTI: Pop PC, then pop Flags
                else if (is_rti && !stack_busy) begin
                    stack_pop_req = 1;
                    next_state = RTI_POP_PC;
                end
                
                // INTERRUPT: Push Flags, then push PC
                else if (is_interrupt && !stack_busy) begin
                    stack_push_req = 1;
                    push_data_sel = {4'h0, flags_current};  // Save flags
                    next_state = INT_PUSH_FLAGS;
                end
                
                else begin
                    next_state = IDLE;
                end
            end
            
            CALL_PUSH: begin
                if (!stack_busy) begin
                    update_sp = 1;
                    sp_updated = sp_out;
                    next_state = IDLE;
                end else begin
                    next_state = CALL_PUSH;
                end
            end
            
            RET_POP: begin
                if (!stack_busy) begin
                    pc_restored = pop_data;
                    update_pc = 1;
                    update_sp = 1;
                    sp_updated = sp_out;
                    next_state = IDLE;
                end else begin
                    next_state = RET_POP;
                end
            end
            
            RTI_POP_PC: begin
                if (!stack_busy) begin
                    pc_restored = pop_data;
                    update_pc = 1;
                    update_sp = 1;
                    sp_updated = sp_out;
                    // Now pop flags
                    stack_pop_req = 1;
                    next_state = RTI_POP_FLAGS;
                end else begin
                    next_state = RTI_POP_PC;
                end
            end
            
            RTI_POP_FLAGS: begin
                if (!stack_busy) begin
                    flags_restored = pop_data[3:0];
                    update_flags = 1;
                    update_sp = 1;
                    sp_updated = sp_out;
                    next_state = IDLE;
                end else begin
                    next_state = RTI_POP_FLAGS;
                end
            end
            
            INT_PUSH_FLAGS: begin
                if (!stack_busy) begin
                    update_sp = 1;
                    sp_updated = sp_out;
                    // Now push PC
                    stack_push_req = 1;
                    push_data_sel = pc_current;
                    next_state = INT_PUSH_PC;
                end else begin
                    next_state = INT_PUSH_FLAGS;
                end
            end
            
            INT_PUSH_PC: begin
                if (!stack_busy) begin
                    update_sp = 1;
                    sp_updated = sp_out;
                    next_state = IDLE;
                end else begin
                    next_state = INT_PUSH_PC;
                end
            end
            
            default: next_state = IDLE;
        endcase
    end

endmodule