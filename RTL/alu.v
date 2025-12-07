module processor_ALU(
input wire [5:0] control_sig,//last control signal is for the 2 to 1 multiplexers
input wire [7:0] op1,op2,//op1 is ra and op2 is rb
input wire carryin,
output wire [7:0] result,
output wire carryout
);
/*multiplexers to select the input to the adder module*/
wire [7:0] Ar,Br;
/*general wires for ease of arithmetic operations*/
wire [7:0] all_ones   = 8'b11111111;
wire [7:0] all_zeroes = 8'b00000000;
wire [7:0] arithmetic_helper;
/*MUX to select either*/
MUX_2_1 arithmetic_mux 
(.A(all_ones),.B(all_zeroes),
.control(control_sig[4]),//should be changed
.out(arithmetic_helper)); 

/*mux to select the number or the inverted number*/
wire [7:0] num_invert;
MUX_2_1 num_inverted
(.A(op2),.B(~op2),
.control(control_sig[4]),//should be changed
.out(num_invert));

/*MUX to select either Ra or all ones*/
wire [7:0] Ra_mux;
MUX_2_1 Ra_multiplexer
(.A(op1),.B(all_ones),
.control(control_sig[5]),//should be changed
.out(Ra_mux));
/*huge note here: ALL CONTROL SIGNALS IN THE 3 PREVIOUS MUXES SHOULD BE CHANGED*/

/*multiplexers to select the input operands to the adder*/
MUX_4_1 Ar_MUX(
    .A(op1),
    .B(arithmetic_helper),
    .C(Ra_mux),
    .D(op1),
    .control(control_sig[1:0]),.out(Ar));
MUX_4_1 Br_MUX(
    .A(num_invert),
    .B(op2),
    .C(num_invert),
    .D(arithmetic_helper),
    .control(control_sig[1:0]),.out(Br));

/*adder assignment part*/
wire [7:0] adder_out;
ADDER 
#(.BUS_SIZE(8)) 
ALU_ADDER 
(.op1(Ar),.op2(Br),.carryin(carryin),.out(adder_out),.carryout(carryout));

/*logic block module*/
wire [7:0] logic_out,and_res,or_res;
assign and_res =op1 & op2;
assign or_res = op1 | op2;
MUX_2_1 LOGIC_MUX(.A(and_res),.B(or_res),.control(control_sig[2]),.out(logic_out));

/*a MUX to select the output from either logic or arithmetic*/
MUX_2_1 out_mux(.A(adder_out),.B(logic_out),.control(control_sig[3]),.out(result));

endmodule
