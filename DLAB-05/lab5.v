`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Dept. of CS, National Chiao Tung University
// Engineer: Chun-Jen Tsai
// 
// Create Date: 2017/10/16 14:21:33
// Design Name: 
// Module Name: lab5
// Project Name: 
// Target Devices: Xilinx FPGA @ 100MHz 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module lab5(
  input clk,
  input reset_n,
  input [3:0] usr_btn,
  output [3:0] usr_led,
  output LCD_RS,
  output LCD_RW,
  output LCD_E,
  output [3:0] LCD_D
);

// turn off all the LEDs
wire btn_level, btn_pressed;
reg prev_btn_level;
reg [127:0] row_A="                "; // Initialize the text of the first row. 
reg [127:0] row_B="                "; // Initialize the text of the second row.
reg [12:0]couprime=2;
reg [12:0]findprime=0;
reg [28:0]rectime=0;
reg [1023:0]prime=0;
reg st=0;
reg levelfinish=0;
reg startprint=0;
reg [1:0]acou=0;

LCD_module lcd0(
  .clk(clk),
  .reset(~reset_n),
  .row_A(row_A),
  .row_B(row_B),
  .LCD_E(LCD_E),
  .LCD_RS(LCD_RS),
  .LCD_RW(LCD_RW),
  .LCD_D(LCD_D)
);
    
debounce btn_db0(
  .clk(clk),
  .btn_input(usr_btn[3]),
  .btn_output(btn_level)
);    

always @(posedge clk) begin
  if (~reset_n)
    prev_btn_level <= 1;
  else
    prev_btn_level <= btn_level;
end

assign btn_pressed = (btn_level == 1 && prev_btn_level == 0);


//reg [1:0]v;
always@(posedge clk)
begin
	if(reset_n==0)
	begin
		couprime=2;
		startprint=0;
	end	
	else if(startprint==0)
	begin
		if(couprime<=1023&&acou==1)
		begin
			couprime=couprime+1;
		end
		else if(couprime==1024)
		begin
			couprime=1024;
			startprint=1;
		end
		else couprime=couprime;
	end
    else if(startprint)
    begin
		couprime=1024;
    end
    else
    begin
		couprime=1024;
    end
end

reg kk=0;
always@(posedge clk)
begin
	if(reset_n==0)
	begin
		findprime=0;
		acou=0;
		kk=0;
	end
	else if(startprint==0)
	begin
		if(findprime<=1023/*&&v==0*/)
		begin
			if(findprime==0)
			begin
				kk=0;
				findprime=couprime;
			end
			else
			begin
				findprime=findprime+couprime;
				kk=1;
			end
			acou=0;
		end
		else if(findprime>=1023)
		begin
			acou=1;
			kk=0;
			findprime=0;
		end
	end
    else if(startprint)
    begin
		findprime=1023;
    end
    else
    begin
		findprime=findprime;
    end
end

always@(posedge clk)
begin
	if(reset_n==0) prime=0;
	else if(startprint==0)
	begin
		prime[0]=1;
		prime[1]=1;
		prime[5]=0;
		prime[7]=0;
		prime[11]=0;
		if(acou==0&&kk==1&&findprime<1024) prime[findprime]=1;
	end
	else
	begin
		prime=prime;
	end
end

//assign usr_led=test;
assign usr_led[0]=prime[4];
assign usr_led[1]=prime[11];
assign usr_led[2]=prime[5];
assign usr_led[3]=prime[7];
//assign usr_led[1]=startprint;

always@(posedge clk)
begin
	if (reset_n==0)
	begin
		st=0;
		rectime=0;
	end
	else if(startprint)
	begin
		if(rectime==70000000)
		begin
			rectime=0;
			st=1;
		end
		else if(rectime<70000000)
		begin
			rectime=rectime+1;
			st=0;
		end
		else
		begin
			rectime=rectime;
		end
	end
end

reg [7:0]timescou=1;
reg [7:0]timescou1=2;

reg c=0;
always@(posedge clk)
begin
	if(reset_n==0)
	begin
		c=0;
	end
	else if(btn_pressed)
	begin
		if(c==0)c=1;
		else if(c)c=0;
	end
end

reg [11:0]u=2;
reg [11:0]uu=3;
reg [11:0]o=2;
reg [11:0]oo=3;
reg hold=1,hold1=1;
reg pro=0,pro1=0;

always@(posedge clk)
begin
	if(reset_n==0)
	begin
		timescou=1;
	end
	else if(st)
	begin	
		if(c==0)
		begin
			if(timescou<172)
			begin
				timescou=timescou+1;
			end
			else if(timescou==172)
			begin
				timescou=1;
			end
		end
		else if(c==1)
		begin
			if(timescou>0)
			begin
				timescou=timescou-1;
			end
			else if(timescou==0)
			begin
				timescou=172;
			end
		end
	end			
end

always@(posedge clk)
begin
	if(reset_n==0)
	begin
		timescou1=2;
	end
	else if(st)
	begin
		if(c==0)
		begin
			if(timescou1<172)
			begin
				timescou1=timescou1+1;
			end
			else if(timescou1==172)
			begin
				timescou1=1;
			end
		end
		else if(c==1)
		begin
			if(timescou1>0)
			begin
				timescou1=timescou1-1;
			end
			else if(timescou1==0)
			begin
				timescou1=172;
			end
		end
	end		
end

always@(posedge clk)
begin
	if(reset_n==0)
	begin
		u=2;
		pro=0;
		hold=1;
	end
	else if(st)
	begin
		pro=1;	
	end
	else if(pro)
	begin
		if(prime[u]==0)
		begin
			hold=0;
			pro=0;
		end
		else if(prime[u]==1)
		begin
			hold=1;
			if(c==0)
			begin
				if(u<1023) u=u+1;
				else if(u==1023) u=0;
			end
			else if(c)
			begin
				if(u>0) u=u-1;
				else if(u==0) u=1023;
			end
		end
	end
	else if(hold==0)
	begin
		hold=1;
		if(c==0)
		begin
			if(u<1023) u=u+1;
			else if(u==1023) u=0;
		end
		else if(c)
		begin
			if(u>0) u=u-1;
			else if(u==0) u=1023;
		end
	end
end

always@(posedge clk)
begin
	if(reset_n==0)
	begin
	o=2;
	end
	else if(hold==0)
	begin
	o=u;	
	end
end


always@(posedge clk)
begin
	if(reset_n==0)
	begin
		uu=3;
		pro1=0;
		hold1=1;
	end
	else if(st)
	begin
		pro1=1;	
	end
	else if(pro1)
	begin
		if(prime[uu]==0)
		begin
			hold1=0;
			pro1=0;
		end
		else if(prime[uu]==1)
		begin
			hold1=1;
			if(c==0)
			begin
				if(uu<1023) uu=uu+1;
				else if(uu==1023) uu=0;
			end
			else if(c)
			begin
				if(uu>0) uu=uu-1;
				else if(uu==0) uu=1023;
			end
		end
	end
	else if(hold1==0)
	begin
		hold1=1;
		if(c==0)
		begin
			if(uu<1023) uu=uu+1;
			else if(uu==1023) uu=0;
		end
		else if(c)
		begin
			if(uu>0) uu=uu-1;
			else if(uu==0) uu=1023;
		end
	end
end

always@(posedge clk)
begin
	if(reset_n==0)
	begin
		oo=3;
	end
	else if(hold1==0)
	begin
		oo=uu;	
	end
end

always@(posedge clk)
//row_A="                "; // Initialize the text of the first row. 
//row_B="                "; // Initialize the text of the second row.
begin
	row_A[127:120]="P";
	row_A[119:112]="r";
	row_A[111:104]="i";
	row_A[103:96]="m";
	row_A[95:88]="e";
	row_A[87:80]= " ";
	row_A[79:72]="#";
	row_A[71:64]=(timescou[7:4]>=0&&timescou[7:4]<10)?timescou[7:4]+48:timescou[7:4]+55;
	row_A[63:56]=(timescou[3:0]>=0&&timescou[3:0]<10)?timescou[3:0]+48:timescou[3:0]+55;
	row_A[55:48]=" ";
	row_A[47:40]="i";	
	row_A[39:32]="s";
	row_A[31:24]=" ";
	row_A[23:16]=(o[9:8]>=0&&o[9:8]<10)?o[9:8]+48:o[9:8]+55;
	row_A[15:8] =(o[7:4]>=0&&o[7:4]<10)?o[7:4]+48:o[7:4]+55;
	row_A[7:0]  =(o[3:0]>=0&&o[3:0]<10)?o[3:0]+48:o[3:0]+55;

	row_B[127:120]="P";
	row_B[119:112]="r";
	row_B[111:104]="i";
	row_B[103:96]="m";
	row_B[95:88]="e";
	row_B[87:80]=" ";
	row_B[79:72]="#";
	row_B[71:64]=(timescou1[7:4]>=0&&timescou1[7:4]<10)?timescou1[7:4]+48:timescou1[7:4]+55;
	row_B[63:56]=(timescou1[3:0]>=0&&timescou1[3:0]<10)?timescou1[3:0]+48:timescou1[3:0]+55;
	row_B[55:48]=" ";
	row_B[47:40]="i";	
	row_B[39:32]="s";
	row_B[31:24]=" ";	
	row_B[23:16]=(oo[9:8]>=0&&oo[9:8]<10)?oo[9:8]+48:oo[9:8]+55;
	row_B[15:8] =(oo[7:4]>=0&&oo[7:4]<10)?oo[7:4]+48:oo[7:4]+55;
	row_B[7:0]  =(oo[3:0]>=0&&oo[3:0]<10)?oo[3:0]+48:oo[3:0]+55;
end

endmodule
