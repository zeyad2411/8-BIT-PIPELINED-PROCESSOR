# Stack and I/O Module Complete Documentation

## Table of Contents
1. [Stack Operations Module](#stack-operations-module)
2. [I/O Operations Module](#io-operations-module)
3. [Integration Guide](#integration-guide)
4. [Testing Guide](#testing-guide)
5. [Common Issues and Solutions](#common-issues-and-solutions)

---

## Stack Operations Module

### Overview
The stack module implements all stack-related operations for your processor:
- **PUSH** - Push data onto stack
- **POP** - Pop data from stack
- **CALL** - Subroutine call (push return address)
- **RET** - Return from subroutine (pop return address)
- **RTI** - Return from interrupt (pop PC and flags)
- **Interrupt handling** - Save PC and flags on interrupt

### Stack Architecture

```
Memory Map (Stack Region):
┌─────┬─────────────┬──────────┐
│ Addr│  Content    │  Stack   │
├─────┼─────────────┼──────────┤
│ 255 │             │ ← SP=255 │ Initial (empty)
│ 254 │             │          │
│ 253 │             │          │
│ ... │             │          │
│  2  │             │          │
│  1  │             │          │
│  0  │             │          │
└─────┴─────────────┴──────────┘

After PUSH 0xAA:
┌─────┬─────────────┬──────────┐
│ 255 │    0xAA     │          │ Pushed value
│ 254 │             │ ← SP=254 │ New SP
│ 253 │             │          │
└─────┴─────────────┴──────────┘

After PUSH 0xBB:
┌─────┬─────────────┬──────────┐
│ 255 │    0xAA     │          │
│ 254 │    0xBB     │          │ Pushed value
│ 253 │             │ ← SP=253 │ New SP
└─────┴─────────────┴──────────┘

After POP:
┌─────┬─────────────┬──────────┐
│ 255 │    0xAA     │          │
│ 254 │    0xBB     │          │ Popped value (0xBB)
│ 253 │             │          │
│ 254 │             │ ← SP=254 │ SP restored
└─────┴─────────────┴──────────┘
```

### Module Interface

#### Inputs
```verilog
clk              // System clock
reset            // Reset signal
stack_push       // Perform PUSH operation
stack_pop        // Perform POP operation
push_data[7:0]   // Data to push
sp_in[7:0]       // Current stack pointer value (R3)
mem_data_in[7:0] // Data from memory (for POP)
```

#### Outputs
```verilog
pop_data[7:0]        // Data popped from stack
sp_out[7:0]          // New stack pointer value
sp_write_enable      // Signal to update R3
mem_read             // Memory read request
mem_write            // Memory write request
mem_address[7:0]     // Memory address for operation
mem_data_out[7:0]    // Data to write to memory
stack_busy           // Operation in progress
stack_overflow       // Stack overflow error
stack_underflow      // Stack underflow error
```

### Operation Timing

#### PUSH Operation
```
Cycle 0: PUSH requested
         - stack_push = 1
         - push_data = 0xAA
         - sp_in = 255

Cycle 1: Write to memory
         - mem_address = 255
         - mem_data_out = 0xAA
         - mem_write = 1
         - Calculate new SP = 254

Cycle 2: Update SP
         - sp_out = 254
         - sp_write_enable = 1
         - Operation complete
```

#### POP Operation
```
Cycle 0: POP requested
         - stack_pop = 1
         - sp_in = 254

Cycle 1: Calculate new SP and read
         - New SP = 255
         - mem_address = 255
         - mem_read = 1

Cycle 2: Return data
         - pop_data = mem_data_in
         - sp_out = 255
         - sp_write_enable = 1
         - Operation complete
```

### CALL/RET Implementation

#### CALL Subroutine
```verilog
// Pseudo-code for CALL instruction

1. Calculate return address: return_addr = PC + 1
2. PUSH return_addr onto stack
3. Update SP: R3 = SP - 1
4. Jump to subroutine: PC = R[rb]
```

Example:
```
Before CALL:
  PC = 0x10
  SP = 0xFF
  Stack[255] = empty

After CALL:
  PC = 0x50 (target address)
  SP = 0xFE
  Stack[255] = 0x11 (return address = 0x10 + 1)
```

#### RET from Subroutine
```verilog
// Pseudo-code for RET instruction

1. POP return_addr from stack
2. Update SP: R3 = SP + 1
3. Jump to return address: PC = return_addr
```

### RTI (Return from Interrupt)

The RTI instruction must restore both PC and FLAGS:

```
Interrupt occurs:
1. Save FLAGS: PUSH flags
2. Save PC: PUSH PC
3. Jump to ISR: PC = M[1]

RTI executes:
1. Restore PC: POP PC
2. Restore FLAGS: POP flags
3. Continue execution
```

**Important**: The stack layout during interrupt:
```
Before Interrupt:
┌─────┬──────┐
│ 255 │      │ ← SP
└─────┴──────┘

After Interrupt (before ISR):
┌─────┬──────────┐
│ 255 │   PC     │ Saved PC
│ 254 │  FLAGS   │ Saved FLAGS
│ 253 │          │ ← SP
└─────┴──────────┘

After RTI:
┌─────┬──────┐
│ 255 │      │ ← SP (restored)
└─────┴──────┘
```

---

## I/O Operations Module

### Overview
The I/O module handles communication between the processor and external devices through:
- **IN** instruction - Read from input port
- **OUT** instruction - Write to output port
- **Interrupt** handling - Detect external interrupt signals

### I/O Architecture

```
┌──────────────────────────────────────────┐
│           PROCESSOR                       │
│                                          │
│  ┌─────────────┐      ┌──────────────┐ │
│  │  Instruction│      │   Register   │ │
│  │   Decoder   │      │     File     │ │
│  └──────┬──────┘      └──────┬───────┘ │
│         │                    │          │
│    is_in│is_out         rb_data         │
│         └────────┬───────────┘          │
│                  ▼                       │
│         ┌────────────────┐              │
│         │   I/O Unit     │              │
│         └────────┬───────┘              │
└──────────────────┼──────────────────────┘
                   │
    ┌──────────────┼──────────────┐
    │              │              │
    ▼              ▼              ▼
┌────────┐   ┌──────────┐   ┌──────────┐
│IN_PORT │   │OUT_PORT  │   │ INTR_IN  │
│        │   │          │   │          │
│ 8 bits │   │  8 bits  │   │  1 bit   │
└────────┘   └──────────┘   └──────────┘
   FROM           TO           FROM
 External      External      External
  Device        Device        Device
```

### Module Interface

#### Inputs
```verilog
clk                  // System clock
reset                // Reset signal
io_read              // Perform IN operation
io_write             // Perform OUT operation
write_data[7:0]      // Data to write to output port
in_port[7:0]         // External input port
intr_in              // External interrupt request
```

#### Outputs
```verilog
read_data[7:0]       // Data read from input port
out_port[7:0]        // External output port
intr_signal          // Synchronized interrupt to processor
io_busy              // I/O operation in progress
io_ready             // Ready for new operation
```

### I/O Operations

#### IN Instruction (Read from Input Port)
```verilog
// Instruction: IN Rb
// Effect: R[rb] ← IN.PORT

Cycle 0: IN instruction decoded
         - is_in_instr = 1
         - rb_addr = register address

Cycle 1: Read from port
         - io_read = 1
         - read_data = in_port
         
Cycle 2: Write to register
         - R[rb] = read_data
         - update_rb = 1
```

Example:
```
IN_PORT = 0x42

Execute: IN R0

Result: R0 = 0x42
```

#### OUT Instruction (Write to Output Port)
```verilog
// Instruction: OUT Rb
// Effect: OUT.PORT ← R[rb]

Cycle 0: OUT instruction decoded
         - is_out_instr = 1
         - rb_data = R[rb]

Cycle 1: Write to port
         - io_write = 1
         - out_port = rb_data
```

Example:
```
R1 = 0x55

Execute: OUT R1

Result: OUT_PORT = 0x55
```

### Interrupt Handling

The interrupt signal requires special handling to prevent metastability:

```verilog
// Two-stage synchronizer
Cycle 0: intr_sync1 ← intr_in
Cycle 1: intr_sync2 ← intr_sync1
Cycle 2: Detect rising edge
         if (intr_sync2 && !intr_prev)
             intr_signal = 1
```

**Why synchronization?**
- External signals may change asynchronously with clock
- Metastability can cause unpredictable behavior
- Two flip-flops reduce metastability probability

**Edge Detection:**
```
Time:      0    1    2    3    4    5
         ___     ___     ___     ___
clk     |   |___|   |___|   |___|   |

intr_in ______________________------

sync1   ____________------__________

sync2   __________________------____
                          ↑
                    Rising edge detected
                    intr_signal = 1
```

---

## Integration Guide

### Connecting Stack Module to Processor

```verilog
module processor (
    input wire clk,
    input wire reset,
    // ... other signals ...
);

    // Register file
    reg [7:0] R [0:3];  // R0, R1, R2, R3
    wire [7:0] SP;
    assign SP = R[3];   // R3 is stack pointer
    
    // Stack signals
    wire stack_push, stack_pop;
    wire [7:0] stack_push_data, stack_pop_data;
    wire [7:0] sp_updated;
    wire sp_update_enable;
    
    // Instantiate stack unit
    stack_unit stack (
        .clk(clk),
        .reset(reset),
        .is_push(opcode == 4'h7 && ra == 2'b00),
        .is_pop(opcode == 4'h7 && ra == 2'b01),
        .is_call(opcode == 4'hB && brx == 2'b01),
        .is_ret(opcode == 4'hB && brx == 2'b10),
        .is_rti(opcode == 4'hB && brx == 2'b11),
        .rb_data(R[rb]),
        .sp_current(SP),
        .pc_current(PC),
        .rb_data_out(stack_pop_data),
        .sp_updated(sp_updated),
        .update_rb(update_rb_from_stack),
        .update_sp(update_sp_from_stack),
        // ... memory interface ...
    );
    
    // Update registers based on stack operations
    always @(posedge clk) begin
        if (update_sp_from_stack) begin
            R[3] <= sp_updated;  // Update SP
        end
        if (update_rb_from_stack) begin
            R[rb] <= stack_pop_data;  // Update Rb from POP
        end
    end

endmodule
```

### Connecting I/O Module to Processor

```verilog
module processor (
    // ... existing signals ...
    
    // External I/O ports
    input wire [7:0] IN_PORT,
    output wire [7:0] OUT_PORT,
    input wire INTR_IN
);

    // I/O signals
    wire is_in, is_out;
    wire [7:0] io_read_data;
    wire update_rb_from_io;
    wire interrupt_signal;
    
    // Decode IN/OUT instructions
    assign is_in = (opcode == 4'h7 && ra == 2'b11);
    assign is_out = (opcode == 4'h7 && ra == 2'b10);
    
    // Instantiate I/O unit
    io_unit io (
        .clk(clk),
        .reset(reset),
        .is_in_instr(is_in),
        .is_out_instr(is_out),
        .rb_addr(rb),
        .rb_data(R[rb]),
        .rb_data_out(io_read_data),
        .update_rb(update_rb_from_io),
        .in_port(IN_PORT),
        .out_port(OUT_PORT),
        .intr_in(INTR_IN),
        .intr_signal(interrupt_signal),
        .io_busy()
    );
    
    // Update register from IN instruction
    always @(posedge clk) begin
        if (update_rb_from_io) begin
            R[rb] <= io_read_data;
        end
    end
    
    // Handle interrupt
    always @(posedge clk) begin
        if (interrupt_signal) begin
            // Trigger interrupt handling
            // (save PC and flags, jump to ISR)
        end
    end

endmodule
```

---

## Testing Guide

### Running the Test Bench

1. **Using EDA Playground:**
   ```
   - Copy stack module code to design.sv
   - Copy I/O module code (append to design.sv)
   - Copy test bench to testbench.sv
   - Select "Icarus Verilog"
   - Check "Open EPWave after run"
   - Click "Run"
   ```

2. **Expected Output:**
   ```
   ========================================
   Stack and I/O Module Test Suite
   ========================================
   
   --- Test 1: System Reset ---
   Reset complete
   
   --- Test 2: Stack PUSH Operations ---
   Time=40: PUSH - Data=0xAA pushed to Address=0xFF, New SP=0xFE
   PUSH 0xAA: SP changed from 255 to 254
   Memory[255] = 0xAA (Expected: 0xAA) PASS
   ...
   
   All Tests Complete!
   ```

### What to Check in Waveforms

#### Stack PUSH Waveform
```
Look for:
✓ stack_push goes high
✓ mem_write goes high
✓ mem_address = current SP
✓ mem_data_out = push_data
✓ sp_out = SP - 1
✓ sp_write_enable goes high
```

#### Stack POP Waveform
```
Look for:
✓ stack_pop goes high
✓ sp_out = SP + 1 (incremented first)
✓ mem_read goes high
✓ mem_address = new SP
✓ pop_data = mem_data_in
✓ sp_write_enable goes high
```

#### I/O OUT Waveform
```
Look for:
✓ io_write goes high
✓ out_port updates to write_data
✓ io_busy goes high then low
```

#### I/O IN Waveform
```
Look for:
✓ io_read goes high
✓ read_data = in_port
✓ io_busy goes high then low
```

#### Interrupt Waveform
```
Look for:
✓ intr_in rising edge
✓ After 2 cycles: intr_sync2 goes high
✓ intr_signal pulses for 1 cycle
```

### Manual Testing Checklist

- [ ] PUSH operation decrements SP correctly
- [ ] POP operation increments SP correctly
- [ ] Data written by PUSH can be read by POP (LIFO order)
- [ ] Stack overflow detected when SP = 0
- [ ] Stack underflow detected when SP = 255
- [ ] OUT instruction updates output port
- [ ] IN instruction reads from input port
- [ ] Interrupt rising edge detected
- [ ] Interrupt synchronized properly (no glitches)
- [ ] CALL/RET sequence works (return address preserved)
- [ ] RTI restores PC and FLAGS correctly

---

## Common Issues and Solutions

### Issue 1: Stack Data Mismatch
```
Problem: Data pushed doesn't match data popped
```
**Solution:**
- Check memory model in test bench
- Verify mem_address during PUSH and POP
- Ensure SP is updated correctly between operations

### Issue 2: SP Not Updating
```
Problem: SP remains at 255 after PUSH
```
**Solution:**
- Check sp_write_enable signal
- Verify connection to register file
- Make sure R[3] is updated when sp_write_enable = 1

### Issue 3: I/O Port Not Updating
```
Problem: OUT_PORT doesn't change
```
**Solution:**
- Verify io_write signal is asserted
- Check timing - write happens on clock edge
- Ensure write_data has correct value

### Issue 4: Interrupt Not Detected
```
Problem: intr_signal never goes high
```
**Solution:**
- Check for rising edge on intr_in
- Verify two-stage synchronizer
- Make sure interrupt is held high for at least 2 clock cycles

### Issue 5: Metastability Warnings
```
Problem: Simulator shows X (unknown) values
```
**Solution:**
- Initialize all registers in reset
- Use synchronizers for async signals
- Ensure proper reset sequence

### Issue 6: Stack Overflow Not Detected
```
Problem: Can push when SP = 0
```
**Solution:**
- Check overflow condition: `if (sp_in == STACK_MIN)`
- Verify STACK_MIN parameter = 8'h00
- Make sure check happens before PUSH

### Issue 7: Multiple Operations Conflict
```
Problem: PUSH and POP at same time causes errors
```
**Solution:**
- Add mutual exclusion in control logic
- Use state machine to serialize operations
- Assert only one operation signal at a time

---

## Performance Considerations

### Stack Operations
- **Latency**: 2 clock cycles per operation
- **Throughput**: Can issue new operation every 2 cycles
- **Depth**: 256 bytes maximum

### I/O Operations
- **Latency**: 1-2 clock cycles
- **Throughput**: One operation per cycle (after busy clears)
- **Synchronization**: 2-3 cycle latency for interrupts

### Optimization Tips
1. **Pipeline**: Don't wait for stack_busy in pipeline - use forwarding
2. **Prediction**: Predict stack data for faster POP
3. **Buffering**: Buffer I/O data to hide latency
4. **Interrupt Priority**: Handle interrupt in parallel with execution

---

## Integration Example: Complete PUSH Instruction

```verilog
// In processor control unit
always @(posedge clk) begin
    case (state)
        DECODE: begin
            if (opcode == 4'h7 && ra == 2'b00) begin
                // PUSH instruction detected
                state <= EXECUTE;
                stack_operation <= PUSH;
            end
        end
        
        EXECUTE: begin
            if (stack_operation == PUSH) begin
                // Activate stack module
                stack_push <= 1;
                push_data <= R[rb];
                state <= WAIT_STACK;
            end
        end
        
        WAIT_STACK: begin
            stack_push <= 0;
            if (!stack_busy) begin
                // Update SP
                R[3] <= sp_out;
                // Increment PC
                PC <= PC + 1;
                state <= FETCH;
            end
        end
    endcase
end
```
