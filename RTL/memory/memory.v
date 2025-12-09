// ============================================================================
// Memory Module for 8-bit Pipelined Processor
// Von Neumann Architecture (Single Memory for Instructions and Data)
// ============================================================================
// This module implements a 256-byte memory that can be accessed by the processor
// for both fetching instructions and reading/writing data.

module memory (
    input wire clk,              // Clock signal
    input wire reset,            // Reset signal
    input wire mem_read,         // Read enable signal (1 = read, 0 = no read)
    input wire mem_write,        // Write enable signal (1 = write, 0 = no write)
    input wire [7:0] address,    // 8-bit address (0-255)
    input wire [7:0] data_in,    // 8-bit data to write into memory
    output reg [7:0] data_out    // 8-bit data read from memory
);

    // ========================================================================
    // Memory Array Declaration
    // ========================================================================
    // This creates 256 memory locations, each holding 8 bits
    reg [7:0] memory_array [0:255];
    
    // ========================================================================
    // Memory Initialization
    // ========================================================================
    // Initialize memory with your program and data
    integer i;
    initial begin
        // Initialize all memory to zero
        for (i = 0; i < 256; i = i + 1) begin
            memory_array[i] = 8'h00;
        end
        
        // Load your program here
        // Example: Simple test program
        // Address 0: Reset vector (where PC starts after reset)
        memory_array[0] = 8'h10;  // PC starts at address 16
        
        // Address 1: Interrupt vector (where PC goes on interrupt)
        memory_array[1] = 8'h80;  // Interrupt service routine at address 128
        
        // Example program starting at address 16:
        // LDM R0, #5        : Load immediate value 5 into R0
        memory_array[16] = 8'hC0;  // Opcode 12 (LDM), ra=0, rb=0
        memory_array[17] = 8'h05;  // Immediate value 5
        
        // LDM R1, #10       : Load immediate value 10 into R1
        memory_array[18] = 8'hC1;  // Opcode 12 (LDM), ra=0, rb=1
        memory_array[19] = 8'h0A;  // Immediate value 10
        
        // ADD R0, R1        : R0 = R0 + R1 (5 + 10 = 15)
        memory_array[20] = 8'h21;  // Opcode 2 (ADD), ra=0, rb=1
        
        // NOP               : No operation
        memory_array[21] = 8'h00;  // Opcode 0 (NOP)
        
        // You can add more instructions here...
    end
    
    // ========================================================================
    // Memory Read Operation
    // ========================================================================
    // Reading is asynchronous (happens immediately based on address)
    always @(*) begin
        if (mem_read) begin
            data_out = memory_array[address];
        end else begin
            data_out = 8'h00;  // Output zero when not reading
        end
    end
    
    // ========================================================================
    // Memory Write Operation
    // ========================================================================
    // Writing happens on the rising edge of the clock
    always @(posedge clk) begin
        if (reset) begin
            // On reset, you can reinitialize if needed
            // (The initial block already handles this)
        end else if (mem_write) begin
            // Write data to memory at the specified address
            memory_array[address] <= data_in;
            
            // Debugging: Display write operations
            $display("Time=%0t: Memory Write - Address=0x%h, Data=0x%h", 
                     $time, address, data_in);
        end
    end
    
    // ========================================================================
    // Optional: Display memory contents for debugging
    // ========================================================================
    task display_memory;
        input [7:0] start_addr;
        input [7:0] end_addr;
        integer j;
        begin
            $display("\n=== Memory Contents ===");
            for (j = start_addr; j <= end_addr; j = j + 1) begin
                $display("Address [%3d] = 0x%h", j, memory_array[j]);
            end
            $display("=======================\n");
        end
    endtask

endmodule