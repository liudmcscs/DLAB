module SeqMultiplier(
input wire clk,
input wire enable,
input wire [7:0] A,
input wire [7:0] B,
output wire [15:0] C
);
reg[15:0]C1;
reg[3:0]cou;
reg[7:0]B1;
always@(posedge clk)
if(enable==0)
begin
	cou<=8;
	B1<=B;
	C1<=0;
end
else if(cou!=0) 
begin
	cou=cou-1;
	C1=C1<<1;
	if((B1&128)==128)
	C1<=C1+A;
	B1<=B1<<1;
end
assign C=C1;
endmodule