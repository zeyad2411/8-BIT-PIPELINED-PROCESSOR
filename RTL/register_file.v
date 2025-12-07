module Processor_Reg_File(
input wire [1:0] ra_add,rb_add,
input wire clk,
input wire [7:0] IN_port,IN_mem,
input wire [3:0] opcode,/*this is the op-code*/
input wire [3:0] cu_input,/*
							bits 1 and 0 select which result will be written into Rb
							bit 2 will select which shift operation will be done on Rb
							bit 3 will select which one of those will be written on Rb finally*/
input wire [3:0] flags,
input wire rb_en,ra_en,flags_en,
input wire [7:0] ALU_result,
output reg [7:0] ra_out,rb_out
);
wire [7:0] Rb_feed;//a wire to feed Rb finally it will be a normal assignment in the always block
/*1-General Purpose Registers*/
reg [7:0] R[0:3];/*General Purpose Registers
		   SP is the last register (R[3])*/
reg CCR [0:3];/*status registers (flags)
		Zero 	 flag is CCR[0], changes after arithmetic, logic, or shift operations
		Negative flag is CCR[1], changes after arithmetic, logic, or shift operations
		Carry 	 flag is CCR[2], changes after arithmetic or shift operations
		Overflow flag is CCR[3], changes after arithmetic or shift operations*/
/*1- output the values inside the register file*/
always @(posedge clk) begin
ra_out <=R[ra_add];
rb_out <=R[rb_add];
if(rb_en) begin
	R[rb_add] <= Rb_feed;
end

else if(ra_en) begin
	R[ra_add] <= ALU_result;//only 1 source for Ra
end

if(flags_en) begin
	{CCR[3],CCR[2],CCR[1],CCR[0]} <= flags;
end

end

wire [7:0] imm_val;
assign imm_val = {opcode,rb_add,ra_add};

wire [7:0]Rb_in_mux;//R[rb] will also be modified through a shifting register
/*logic implementing the input to R[rb]*/

MUX_4_1 Rb_sources(
.A(IN_mem),//op-code 7 ra 1 OR op-code 12 ra 1 OR op-code 13 --> sel : 00
.B(IN_port),//op-code 7 ra 3								 --> sel : 01
.D(ALU_result),//op-code 8									 --> sel : 11
.C(imm_val),//op-code 12 ra 0								 --> sel : 10
.control(cu_input[1:0]),//this should probably be modified to be a function in ra
.out(Rb_in_mux)
);

wire [7:0] Rb_in_shift;//this selects which Rb shifting operation will be taken

MUX_2_1 Rb_shift(
.A({R[rb_add][6:0],CCR[2]}), // Sel --> 0
.B({CCR[2],R[rb_add][7:1]}), // Sel --> 1
.control(cu_input[2]),
.out(Rb_in_shift)
);


MUX_2_1 Rb_shift_input(
.A(Rb_in_mux), // Sel --> 0
.B(Rb_in_shift),// Sel --> 1
.control(cu_input[3]),
.out(Rb_feed)
);
endmodule
