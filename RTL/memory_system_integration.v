// ============================================================================
// Complete Integration Example
// Memory + Stack + I/O System
// ============================================================================
// This module shows how to integrate memory, stack, and I/O modules together
// This is a simplified processor core focusing on memory operations

module memory_system_integrated (
    input wire clk,
    input wire reset,
    
    // Instruction interface (from processor fetch stage)
    input wire [7:0] instr_address,
    output reg [7:0] instruction,
    
    // Data interface (from processor execute stage)
    input wire data_read,
    input wire data_write,
    input wire [7:0] data_address,
    input wire [7:0] data_in,
    output reg [7:0] data_out,
    
    // Stack interface
    input wire stack_push,
    input wire stack_pop,
    input wire [7:0] stack_data_in,
    output wire [7:0] stack_data_out,
    input wire [7:0] sp_in,
    output wire [7:0] sp_out,
    output wire sp_update,
    
    // I/O interface
    input wire io_in_req,
    input wire io_out_req,
    input wire [7:0] io_data_out_val,
    output wire [7:0] io_data_in_val,
    input wire [7:0] in_port,
    output wire [7:0] out_port,
    
    // Interrupt
    input wire intr_in,
    output wire intr_signal,
    
    // Status
    output wire mem_busy,
    output wire stack_busy,
    output wire io_busy
);

    // ========================================================================
    // Memory Arbiter Signals
    // ========================================================================
    // The arbiter decides which requestor (instruction fetch, data access,
    // stack, I/O) gets access to memory
    
    reg mem_read_en;
    reg mem_write_en;
    reg [7:0] mem_addr_sel;
    reg [7:0] mem_data_in_sel;
    wire [7:0] mem_data_out_wire;
    
    // Request signals
    wire instr_req;
    wire data_req;
    wire stack_mem_req;
    
    assign instr_req = 1;  // Always fetching instructions
    assign data_req = data_read | data_write;
    
    // ========================================================================
    // Stack Module Connections
    // ========================================================================
    wire stack_mem_read;
    wire stack_mem_write;
    wire [7:0] stack_mem_address;
    wire [7:0] stack_mem_data_out;
    wire [7:0] stack_mem_data_in;
    wire stack_overflow;
    wire stack_underflow;
    
    assign stack_mem_req = stack_mem_read | stack_mem_write;
    assign stack_mem_data_in = mem_data_out_wire;
    
    stack_controller stack_ctrl (
        .clk(clk),
        .reset(reset),
        .stack_push(stack_push),
        .stack_pop(stack_pop),
        .push_data(stack_data_in),
        .pop_data(stack_data_out),
        .sp_in(sp_in),
        .sp_out(sp_out),
        .sp_write_enable(sp_update),
        .mem_read(stack_mem_read),
        .mem_write(stack_mem_write),
        .mem_address(stack_mem_address),
        .mem_data_out(stack_mem_data_out),
        .mem_data_in(stack_mem_data_in),
        .stack_busy(stack_busy),
        .stack_overflow(stack_overflow),
        .stack_underflow(stack_underflow)
    );
    
    // ========================================================================
    // I/O Module Connections
    // ========================================================================
    wire io_ready;
    
    io_controller io_ctrl (
        .clk(clk),
        .reset(reset),
        .io_read(io_in_req),
        .io_write(io_out_req),
        .write_data(io_data_out_val),
        .read_data(io_data_in_val),
        .in_port(in_port),
        .out_port(out_port),
        .intr_in(intr_in),
        .intr_signal(intr_signal),
        .io_busy(io_busy),
        .io_ready(io_ready)
    );
    
    // ========================================================================
    // Unified Memory Module
    // ========================================================================
    memory main_memory (
        .clk(clk),
        .reset(reset),
        .mem_read(mem_read_en),
        .mem_write(mem_write_en),
        .address(mem_addr_sel),
        .data_in(mem_data_in_sel),
        .data_out(mem_data_out_wire)
    );
    
    // ========================================================================
    // Memory Arbiter (Von Neumann Architecture)
    // ========================================================================
    // Priority: Stack > Data > Instruction
    // This prevents hazards and ensures correct operation
    
    always @(*) begin
        // Default: service instruction fetch
        mem_read_en = 0;
        mem_write_en = 0;
        mem_addr_sel = instr_address;
        mem_data_in_sel = 8'h00;
        
        // Priority 1: Stack operations (highest priority)
        if (stack_mem_req) begin
            mem_addr_sel = stack_mem_address;
            if (stack_mem_write) begin
                mem_write_en = 1;
                mem_read_en = 0;
                mem_data_in_sel = stack_mem_data_out;
            end else if (stack_mem_read) begin
                mem_write_en = 0;
                mem_read_en = 1;
            end
        end
        
        // Priority 2: Data memory operations
        else if (data_req) begin
            mem_addr_sel = data_address;
            if (data_write) begin
                mem_write_en = 1;
                mem_read_en = 0;
                mem_data_in_sel = data_in;
            end else if (data_read) begin
                mem_write_en = 0;
                mem_read_en = 1;
            end
        end
        
        // Priority 3: Instruction fetch (lowest priority)
        else begin
            mem_addr_sel = instr_address;
            mem_read_en = 1;
            mem_write_en = 0;
        end
    end
    
    // ========================================================================
    // Output Assignment
    // ========================================================================
    always @(*) begin
        // Instruction output
        if (!stack_mem_req && !data_req) begin
            instruction = mem_data_out_wire;
        end else begin
            instruction = 8'h00;  // NOP if stalled
        end
        
        // Data output
        if (data_read && !stack_mem_req) begin
            data_out = mem_data_out_wire;
        end else begin
            data_out = 8'h00;
        end
    end
    
    // ========================================================================
    // Memory Busy Signal
    // ========================================================================
    // Memory is "busy" when higher priority operations prevent lower ones
    assign mem_busy = stack_mem_req | data_req;
    
    // ========================================================================
    // Debug: Display memory arbiter decisions
    // ========================================================================
    always @(posedge clk) begin
        if (stack_mem_req) begin
            $display("Time=%0t: ARBITER -> STACK (Addr=0x%h)", $time, stack_mem_address);
        end else if (data_req) begin
            $display("Time=%0t: ARBITER -> DATA (Addr=0x%h)", $time, data_address);
        end else if (instr_req) begin
            $display("Time=%0t: ARBITER -> INSTR (Addr=0x%h)", $time, instr_address);
        end
    end

endmodule