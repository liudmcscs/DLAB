`timescale 1ns / 1ps
module mmult(
  input  clk,                 // Clock signal
  input  reset_n,             // Reset signal (negative logic)
  input  enable,              // Activation signal for matrix multiplication
  input  [0:9*8-1] A_mat,     // A matrix
  input  [0:9*8-1] B_mat,     // B matrix
  output reg valid,               // Signals that the output is valid to read
  output reg [0:9*17-1] C_mat // The result of A x B
);
reg [0:3*8-1] A1 ;
reg [0:3*8-1] A2 ;
reg [0:3*8-1] A3 ;
reg [0:3*8-1] B1 ;
reg [0:3*8-1] B2 ;
reg [0:3*8-1] B3 ;
reg [0:9*17-1]C;
reg [0:3]cou;


always@(posedge clk)
begin
	if(enable==0)
	begin
		A1=A_mat[0:23];
		A2=A_mat[24:47];
		A3=A_mat[48:71];
		B1={B_mat[0:7],B_mat[24:24+7],B_mat[48:48+7]};
		B2={B_mat[8:8+7],B_mat[32:32+7],B_mat[56:56+7]};
		B3={B_mat[16:16+7],B_mat[40:40+7],B_mat[64:64+7]};
		cou=5;
		valid=0;
		C_mat=0;
	end
	else if(cou==4)
	begin
		C_mat[0:16]=A1[0:7]*B1[0:7]+A1[8:15]*B1[8:15]+A1[16:23]*B1[16:23];
		C_mat[17+:17]=A1[0:7]*B2[0:7]+A1[8:15]*B2[8:15]+A1[16:23]*B2[16:23];
		C_mat[34+:17]=A1[0:7]*B3[0:7]+A1[8:15]*B3[8:15]+A1[16:23]*B3[16:23];
	end
	else if(cou==3)
	begin
		C_mat[51+:17]=A2[0:7]*B1[0:7]+A2[8:15]*B1[8:15]+A2[16:23]*B1[16:23];
		C_mat[68+:17]=A2[0:7]*B2[0:7]+A2[8:15]*B2[8:15]+A2[16:23]*B2[16:23];
		C_mat[85+:17]=A2[0:7]*B3[0:7]+A2[8:15]*B3[8:15]+A2[16:23]*B3[16:23];
	end
	else if(cou==2)
	begin
		C_mat[102+:17]=A3[0:7]*B1[0:7]+A3[8:15]*B1[8:15]+A3[16:23]*B1[16:23];
		C_mat[119+:17]=A3[0:7]*B2[0:7]+A3[8:15]*B2[8:15]+A3[16:23]*B2[16:23];
		C_mat[136+:17]=A3[0:7]*B3[0:7]+A3[8:15]*B3[8:15]+A3[16:23]*B3[16:23];
	end
	else if(cou==0)
	begin
		valid=1;
	end
	cou=cou-1;
end
always @(negedge reset_n)
begin
	C_mat = 0;
end
endmodule


