`timescale 1ns / 1ps



module lab3(
  input  clk,            // System clock at 100 MHz
  input  reset_n,        // System reset signal, in negative logic
  input  [3:0] usr_btn,  // Four user pushbuttons
  output [3:0] usr_led   // Four yellow LEDs
);
parameter hold=15'b100111000100000;

reg [3:0]btn;
reg [3:0]prebtn;
reg signed [3:0]counter;
reg signed [3:0]counter1;
reg [14:0]cou0;//debouncing
reg [14:0]cou1;
reg [19:0]cou2;
reg [19:0]cou3;
reg [3:0]st1;
reg [2:0]ctrl;
reg [20:0]pwmcou;
reg [1:0]temp;

always@(posedge clk)
begin
	if(reset_n==0)
	begin
		btn[0]=0;
		st1[0]=0;
		cou0=0;
		prebtn[0]=0;
	end

	else if(prebtn[0])
	begin
		btn[0]=0;
		st1[0]=0;
		cou0=0;
	end

	else if(usr_btn[0])
	begin
		st1[0]=1;
	end

	else if(st1[0])
	begin
		if(cou0<hold)
		begin
			cou0=cou0+1;
		end
		else if(cou0==hold)
		begin
			btn[0]=1;
		end
	end 
	prebtn[0]=btn[0];
end

always@(posedge clk)
begin
	if(reset_n==0)
	begin
		btn[1]=0;
		st1[1]=0;
		cou1=0;
		prebtn[1]=0;
	end

	else if(prebtn[1])
	begin
		btn[1]=0;
		st1[1]=0;
		cou1=0;
	end


	else if(usr_btn[1])
	begin
		st1[1]=1;
	end


	else if(st1[1])
	begin
		if(cou1<hold)
		begin
			cou1=cou1+1;
		end
		else if(cou1==hold)
		begin
			btn[1]=1;
		end
	end 
	prebtn[1]=btn[1];
end

always@(posedge clk)
begin
	if(reset_n==0)
	begin
		btn[2]=0;
		st1[2]=0;
		cou2=0;
		prebtn[2]=0;
	end

	else if(prebtn[2])
	begin
		btn[2]=0;
		st1[2]=0;
		cou2=0;
	end


	else if(usr_btn[2])
	begin
		st1[2]=1;
	end


	else if(st1[2])
	begin
		if(cou2<hold)
		begin
			cou2=cou2+1;
		end
		else if(cou2==hold)
		begin
			btn[2]=1;
		end
	end 
	prebtn[2]=btn[2];
end

always@(posedge clk)
begin
	if(reset_n==0)
	begin
		btn[3]=0;
		st1[3]=0;
		cou3=0;
		prebtn[3]=0;
	end
	else if(prebtn[3])
	begin
		btn[3]=0;
		st1[3]=0;
		cou3=0;
	end
	else if(usr_btn[3])
	begin
		st1[3]=1;
	end
	else if(st1[3])
	begin
		if(cou3<hold)
		begin
			cou3=cou3+1;
		end
		else if(cou3==hold)
		begin
			btn[3]=1;
		end
	end 
	prebtn[3]=btn[3];
end

always@(posedge clk)
begin
	if(reset_n==0)
	begin
		ctrl=0;
	end
	else if(btn[2])
	begin
		if(ctrl==0)
		begin
			ctrl=0;
		end
		else if(ctrl>0)
		begin
			ctrl=ctrl-1;
		end
	end
	else if(btn[3])
	begin
		if(ctrl==4)
		begin
			ctrl=4;
		end
		else if(ctrl<4)
		begin
			ctrl=ctrl+1;
		end
	end
end

always@(posedge clk)
begin
	if(reset_n==0)
	begin
		pwmcou=0;
		temp=0;
	end
	else if(ctrl==0)
	begin
		if(pwmcou<1000000)
		begin
			pwmcou=pwmcou+1;
			if(pwmcou<50000)
			begin
				temp=1;
			end
			else if(pwmcou==50000)
			begin
				temp=0;
			end
		end
		else if(pwmcou==1000000)
		begin
			pwmcou=0;
		end
	end
	else if(ctrl==1)
	begin
		if(pwmcou<1000000)
		begin
			pwmcou=pwmcou+1;
			if(pwmcou<250000)
			begin
				temp=1;
			end
			else if(pwmcou==250000)
			begin
				temp=0;
			end
		end
		else if(pwmcou==1000000)
		begin
			pwmcou=0;	
		end
	end
	else if(ctrl==2)
	begin
		if(pwmcou<1000000)
		begin
			pwmcou=pwmcou+1;
			if(pwmcou<500000)
			begin
				temp=1;
			end
			else if(pwmcou==500000)
			begin
				temp=0;
			end
		end
		else if(pwmcou==1000000)
			begin
				pwmcou=0;
			end
	end
	else if(ctrl==3)
	begin
		if(pwmcou<1000000)
		begin
			pwmcou=pwmcou+1;
			if(pwmcou<750000)
			begin
				temp=1;
			end
			else if(pwmcou==750000)
			begin
				temp=0;
			end
		end
		else if(pwmcou==1000000)
		begin
			pwmcou=0;
		end
	end
	else if(ctrl==4)
	begin
		if(pwmcou<1000000)
		begin
			pwmcou=pwmcou+1;
			if(pwmcou<1000000)
			begin
				temp=1;
			end
			else if(pwmcou==1000000)
			begin
				temp=0;
			end
		end
		else if(pwmcou==1000000)
		begin
			pwmcou=0;
		end
	end
end




always@(posedge clk)
begin
	if(!reset_n)
	begin
		counter=0;
	end
	else if(btn[0])
	begin
		if(counter==-8)counter<=counter;
		else counter<=counter-1;
	end
	else if(btn[1])
	begin
		if(counter==7)counter=counter;
		else counter=counter+1;
	end
	else
	begin
		counter=counter;
	end
end

always@(posedge clk)
begin
	if(reset_n==0)
	begin
		counter1[0] = 0;
		counter1[1] = 0;
		counter1[2] = 0;
		counter1[3] = 0;
	end
	else if(temp)
	begin
		counter1[0] = counter[0];
		counter1[1] = counter[1];
		counter1[2] = counter[2];
		counter1[3] = counter[3];
	end
	else if(temp==0)
	begin
		counter1[0] = 0;
		counter1[1] = 0;
		counter1[2] = 0;
		counter1[3] = 0;
	end
end

assign usr_led[0] = counter1[0];
assign usr_led[1] = counter1[1];
assign usr_led[2] = counter1[2];
assign usr_led[3] = counter1[3];

endmodule
