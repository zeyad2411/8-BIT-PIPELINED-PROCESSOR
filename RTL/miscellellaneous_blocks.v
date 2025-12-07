/*multiplexers*/
module MUX_2_1 
  #(parameter BUS_SIZE = 8)(
    input wire [BUS_SIZE-1:0] A,B,
input wire control,
    output wire [BUS_SIZE-1:0] out
);/*default bus size of 16 due to the ALU*/

assign out = control? B : A;

endmodule

module MUX_4_1 
  #(parameter BUS_SIZE = 8)(
    input wire [BUS_SIZE-1:0] A,B,C,D,
input wire [1:0] control,
    output wire [BUS_SIZE-1:0] out
);
  wire [BUS_SIZE-1:0] firstout,secondout;
MUX_2_1 firstmux
(.A(A),.B(B),
.control(control[0]),
.out(firstout));

MUX_2_1 secondmux
(.A(C),.B(D),
.control(control[0]),
.out(secondout));

MUX_2_1 thirdmux
(.A(firstout),.B(secondout),
.control(control[1]),
.out(out));

endmodule

module ADDER
#(parameter
BUS_SIZE = 8//needs to be at least 2
)(
input wire [BUS_SIZE-1:0] op1,op2,
input wire carryin,
output wire [BUS_SIZE-1:0] out,
output wire carryout
);
wire carry[BUS_SIZE-1:0];
generate
FULL_ADDER First_bit(.A(op1[0]),.B(op2[0]),.carryin(carryin),.cout(carry[0]),.sum(out[0]));
genvar i;
for (i=1;i<BUS_SIZE;i = i+1) begin
FULL_ADDER myFA(.A(op1[i]),.B(op2[i]),.carryin(carry[i-1]),.cout(carry[i]),.sum(out[i]));
end
endgenerate
assign carryout = carry[BUS_SIZE-1];
endmodule

module HALF_ADDER
(
input wire A,B,
output wire sum,cout
);
assign sum = A^B;
assign cout = A&B;
endmodule

module FULL_ADDER
(
input wire A,B,carryin,
output wire sum,cout
);
wire w1,w2,w3;
HALF_ADDER 
first(.A(A),.B(B),.sum(w1),.cout(w2));

HALF_ADDER
second(.A(carryin),.B(w1),.sum(sum),.cout(w3));

assign cout = w2 | w3;
endmodule
