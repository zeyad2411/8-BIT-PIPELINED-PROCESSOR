// ============================================================================
// Input/Output Module
// ============================================================================
// This module handles all I/O operations (IN and OUT instructions)
// Provides interface between processor and external devices

module io_controller (
    input wire clk,                    // Clock signal
    input wire reset,                  // Reset signal
    
    // I/O operation control
    input wire io_read,                // 1 = perform IN operation (read from port)
    input wire io_write,               // 1 = perform OUT operation (write to port)
    input wire [7:0] write_data,       // Data to write to output port
    output reg [7:0] read_data,        // Data read from input port
    
    // External I/O ports
    input wire [7:0] in_port,          // 8-bit input port (from external devices)
    output reg [7:0] out_port,         // 8-bit output port (to external devices)
    
    // Interrupt signal
    input wire intr_in,                // Interrupt request input
    output reg intr_signal,            // Synchronized interrupt to processor
    
    // Status signals
    output reg io_busy,                // 1 = I/O operation in progress
    output reg io_ready                // 1 = I/O ready for new operation
);

    // ========================================================================
    // Internal Registers
    // ========================================================================
    reg [7:0] in_port_buffer;          // Input port buffer (for stability)
    reg [7:0] out_port_buffer;         // Output port buffer
    
    // Interrupt synchronization (prevent metastability)
    reg intr_sync1, intr_sync2;
    reg intr_prev;
    
    // I/O timing control
    reg io_operation_active;
    
    // ========================================================================
    // Reset and Initialization
    // ========================================================================
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            out_port <= 8'h00;
            out_port_buffer <= 8'h00;
            in_port_buffer <= 8'h00;
            read_data <= 8'h00;
            io_busy <= 0;
            io_ready <= 1;
            io_operation_active <= 0;
            
            // Reset interrupt synchronizers
            intr_sync1 <= 0;
            intr_sync2 <= 0;
            intr_prev <= 0;
            intr_signal <= 0;
        end else begin
            // Default state - ready for operations
            io_ready <= 1;
            io_busy <= 0;
            
            // ================================================================
            // INPUT Operation (IN instruction)
            // ================================================================
            if (io_read && !io_operation_active) begin
                // Read from input port
                in_port_buffer <= in_port;     // Capture input
                read_data <= in_port;          // Output to processor
                io_busy <= 1;
                io_operation_active <= 1;
                
                $display("Time=%0t: IN - Read data=0x%h from IN.PORT", $time, in_port);
            end
            
            // ================================================================
            // OUTPUT Operation (OUT instruction)
            // ================================================================
            else if (io_write && !io_operation_active) begin
                // Write to output port
                out_port_buffer <= write_data;
                out_port <= write_data;        // Update output port
                io_busy <= 1;
                io_operation_active <= 1;
                
                $display("Time=%0t: OUT - Write data=0x%h to OUT.PORT", $time, write_data);
            end
            
            // ================================================================
            // Complete I/O operation
            // ================================================================
            else if (io_operation_active) begin
                io_operation_active <= 0;
                io_busy <= 0;
            end
            
            // ================================================================
            // Interrupt Synchronization (2-stage synchronizer)
            // ================================================================
            // Prevent metastability when interrupt signal crosses clock domains
            intr_sync1 <= intr_in;
            intr_sync2 <= intr_sync1;
            
            // Edge detection - trigger on rising edge of interrupt
            intr_prev <= intr_sync2;
            if (intr_sync2 && !intr_prev) begin
                intr_signal <= 1;  // Rising edge detected
                $display("Time=%0t: INTERRUPT - Rising edge detected", $time);
            end else begin
                intr_signal <= 0;  // Clear after one cycle
            end
        end
    end
    
    // ========================================================================
    // Optional: I/O Port History (for debugging)
    // ========================================================================
    // Keep track of last few I/O operations
    reg [7:0] io_history [0:7];
    reg [2:0] history_ptr;
    
    always @(posedge clk) begin
        if (reset) begin
            history_ptr <= 0;
        end else if (io_write) begin
            io_history[history_ptr] <= write_data;
            history_ptr <= history_ptr + 1;
        end
    end
    
    // ========================================================================
    // Task: Display I/O History
    // ========================================================================
    task display_io_history;
        integer i;
        begin
            $display("\n=== I/O Output History ===");
            for (i = 0; i < 8; i = i + 1) begin
                $display("Entry %0d: 0x%h", i, io_history[i]);
            end
            $display("=======================\n");
        end
    endtask

endmodule


// ============================================================================
// Memory-Mapped I/O Controller (Alternative Implementation)
// ============================================================================
// This version uses memory-mapped I/O where I/O ports appear as memory addresses
// Useful if you want to integrate I/O into the memory space

module mmio_controller (
    input wire clk,
    input wire reset,
    
    // Memory interface (from processor)
    input wire mem_read,
    input wire mem_write,
    input wire [7:0] mem_address,
    input wire [7:0] mem_data_in,
    output reg [7:0] mem_data_out,
    output reg mmio_access,            // 1 = this is a memory-mapped I/O access
    
    // External I/O ports
    input wire [7:0] in_port,
    output reg [7:0] out_port,
    
    // Interrupt
    input wire intr_in,
    output reg intr_signal
);

    // ========================================================================
    // Memory-Mapped I/O Address Map
    // ========================================================================
    // You can define specific memory addresses for I/O
    parameter ADDR_IN_PORT = 8'hFC;    // Address 252: Input port
    parameter ADDR_OUT_PORT = 8'hFD;   // Address 253: Output port
    parameter ADDR_INTR_STATUS = 8'hFE; // Address 254: Interrupt status
    parameter ADDR_INTR_CONTROL = 8'hFF; // Address 255: Interrupt control
    
    // ========================================================================
    // Internal Registers
    // ========================================================================
    reg intr_sync1, intr_sync2, intr_prev;
    reg intr_enable;
    
    // ========================================================================
    // Check if address is in memory-mapped I/O range
    // ========================================================================
    always @(*) begin
        if (mem_address >= ADDR_IN_PORT && mem_address <= ADDR_INTR_CONTROL) begin
            mmio_access = 1;
        end else begin
            mmio_access = 0;
        end
    end
    
    // ========================================================================
    // Memory-Mapped I/O Operations
    // ========================================================================
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            out_port <= 8'h00;
            mem_data_out <= 8'h00;
            intr_enable <= 0;
            intr_signal <= 0;
            intr_sync1 <= 0;
            intr_sync2 <= 0;
            intr_prev <= 0;
        end else begin
            // Default
            mem_data_out <= 8'h00;
            
            // ================================================================
            // Memory-Mapped READ Operations
            // ================================================================
            if (mem_read && mmio_access) begin
                case (mem_address)
                    ADDR_IN_PORT: begin
                        // Read from input port
                        mem_data_out <= in_port;
                        $display("Time=%0t: MMIO Read IN_PORT = 0x%h", $time, in_port);
                    end
                    
                    ADDR_OUT_PORT: begin
                        // Read current output port value
                        mem_data_out <= out_port;
                    end
                    
                    ADDR_INTR_STATUS: begin
                        // Read interrupt status
                        mem_data_out <= {7'h00, intr_sync2};
                    end
                    
                    ADDR_INTR_CONTROL: begin
                        // Read interrupt enable
                        mem_data_out <= {7'h00, intr_enable};
                    end
                    
                    default: mem_data_out <= 8'h00;
                endcase
            end
            
            // ================================================================
            // Memory-Mapped WRITE Operations
            // ================================================================
            if (mem_write && mmio_access) begin
                case (mem_address)
                    ADDR_OUT_PORT: begin
                        // Write to output port
                        out_port <= mem_data_in;
                        $display("Time=%0t: MMIO Write OUT_PORT = 0x%h", $time, mem_data_in);
                    end
                    
                    ADDR_INTR_CONTROL: begin
                        // Enable/disable interrupts
                        intr_enable <= mem_data_in[0];
                        $display("Time=%0t: MMIO Interrupt Enable = %b", $time, mem_data_in[0]);
                    end
                    
                    default: begin
                        // Ignore writes to read-only addresses
                    end
                endcase
            end
            
            // ================================================================
            // Interrupt Synchronization
            // ================================================================
            intr_sync1 <= intr_in;
            intr_sync2 <= intr_sync1;
            intr_prev <= intr_sync2;
            
            if (intr_sync2 && !intr_prev && intr_enable) begin
                intr_signal <= 1;
            end else begin
                intr_signal <= 0;
            end
        end
    end

endmodule


// ============================================================================
// I/O Unit - Complete Integration Module
// ============================================================================
// This module integrates I/O operations with processor instructions

module io_unit (
    input wire clk,
    input wire reset,
    
    // Instruction decode signals
    input wire is_in_instr,            // IN instruction detected
    input wire is_out_instr,           // OUT instruction detected
    input wire [1:0] rb_addr,          // Register Rb address
    
    // Register file interface
    input wire [7:0] rb_data,          // Data from R[rb] for OUT
    output reg [7:0] rb_data_out,      // Data for R[rb] from IN
    output reg update_rb,              // 1 = write rb_data_out to R[rb]
    
    // External I/O ports
    input wire [7:0] in_port,          // External input port
    output wire [7:0] out_port,        // External output port
    
    // Interrupt interface
    input wire intr_in,                // External interrupt signal
    output wire intr_signal,           // Interrupt to processor
    
    // Status
    output wire io_busy
);

    // ========================================================================
    // Internal Signals
    // ========================================================================
    wire io_read_req;
    wire io_write_req;
    wire [7:0] io_read_data;
    
    assign io_read_req = is_in_instr;
    assign io_write_req = is_out_instr;
    
    // ========================================================================
    // Instantiate I/O Controller
    // ========================================================================
    io_controller io_ctrl (
        .clk(clk),
        .reset(reset),
        .io_read(io_read_req),
        .io_write(io_write_req),
        .write_data(rb_data),
        .read_data(io_read_data),
        .in_port(in_port),
        .out_port(out_port),
        .intr_in(intr_in),
        .intr_signal(intr_signal),
        .io_busy(io_busy),
        .io_ready()
    );
    
    // ========================================================================
    // Connect I/O to Register File
    // ========================================================================
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            rb_data_out <= 8'h00;
            update_rb <= 0;
        end else begin
            if (is_in_instr && !io_busy) begin
                // IN instruction: read from port to register
                rb_data_out <= io_read_data;
                update_rb <= 1;
            end else begin
                update_rb <= 0;
            end
        end
    end

endmodule