`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Dept. of Computer Science, National Chiao Tung University
// Engineer: Chun-Jen Tsai
// 
// Create Date: 2017/12/06 20:44:08
// Design Name: 
// Module Name: lab9
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: This is a sample circuit to show you how to initialize an SRAM
//              with a pre-defined data file. Hit BTN0/BTN1 let you browse
//              through the data.
// 
// Dependencies: LCD_module, debounce
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module lab9(
  // General system I/O ports
  input  clk,
  input  reset_n,
  input  [3:0] usr_btn,
  output [3:0] usr_led,

  // 1602 LCD Module Interface
  output LCD_RS,
  output LCD_RW,
  output LCD_E,
  output [3:0] LCD_D
);

localparam [3:0] S_MAIN_INIT   =4'b1111, S_MAIN_ADDR  =4'b0001,
                 S_MAIN_READ   =4'b0010, S_MAIN_WAIT  =4'b0011,
                 S_MAIN_RECORD =4'b0100, S_MAIN_LOOP1 =4'b0101,   
                 S_MAIN_LOOP2  =4'b0110, S_MAIN_LOOP3 =4'b0111,   
                 S_MAIN_SHOW   =4'b1000;
// declare system variables
wire [1:0]        btn_level, btn_pressed;
reg  [1:0]        prev_btn_level;
reg  [3:0]        P, P_next;
reg  [11:0]       sample_addr;
reg  signed [7:0] sample_data;
wire [7:0]        abs_data;
reg  [127:0]row_A= "Press BTN0 to do";
reg  [127:0]row_B= "x-correlation...";

// declare SRAM control signals
wire [10:0] sram_addr;
wire [7:0]  data_in;
wire [7:0]  data_out;
wire        sram_we, sram_en;

assign usr_led = 4'h00;

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
  .btn_input(usr_btn[0]),
  .btn_output(btn_level[0])
);

debounce btn_db1(
  .clk(clk),
  .btn_input(usr_btn[1]),
  .btn_output(btn_level[1])
);

//
// Enable one cycle of btn_pressed per each button hit
//
always @(posedge clk) begin
  if (~reset_n)
    prev_btn_level <= 2'b00;
  else
    prev_btn_level <= btn_level;
end

// ------------------------------------------------------------------------
// The following code describes an initialized SRAM memory block that
// stores an 1024+64 8-bit signed data samples.
sram ram0(.clk(clk), .we(sram_we), .en(sram_en),
          .addr(sram_addr), .data_i(data_in), .data_o(data_out));

assign sram_we = usr_btn[3]; // In this demo, we do not write the SRAM. However,
                             // if you set 'we' to 0, Vivado fails to synthesize
                             // ram0 as a BRAM -- this is a bug in Vivado.
assign sram_en = (P == S_MAIN_ADDR || P == S_MAIN_READ); // Enable the SRAM block.
assign sram_addr = sample_addr[11:0];
assign data_in = 8'b0; // SRAM is read-only so we tie inputs to zeros.
// End of the SRAM memory block.
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// FSM of the main controller
always @(posedge clk) begin
  if (~reset_n) begin
    P <= S_MAIN_INIT; // read samples at 000 first
  end
  else begin
    P <= P_next;
  end
end
            
reg btn0=0;
always@(posedge clk)
begin
    if(reset_n==0)btn0=0;
    else if(usr_btn[0]==1)btn0=1;
end

always @(*) begin // FSM next-state logic
  case (P)
    S_MAIN_INIT:
      if(initcount==500)P_next=S_MAIN_ADDR;
      else P_next=S_MAIN_INIT;
    S_MAIN_ADDR: // send an address to the SRAM 
      P_next = S_MAIN_READ;
    S_MAIN_READ: // fetch the sample from the SRAM
      if(btn0==0)P_next = S_MAIN_WAIT; 
      else if(btn0==1)P_next=S_MAIN_RECORD;
      else P_next=S_MAIN_READ;
    S_MAIN_WAIT: // wait for a button click
      if (btn0==1) P_next = S_MAIN_RECORD;
      else P_next = S_MAIN_WAIT;
    S_MAIN_RECORD:
      if(recordfinish==1) P_next= S_MAIN_LOOP1;
      else if(recordfinish==0)P_next=S_MAIN_ADDR;
      else P_next=S_MAIN_RECORD;
    S_MAIN_LOOP1:
      if(loop1finish==1) P_next=S_MAIN_LOOP2;
      else P_next=S_MAIN_LOOP1;
   S_MAIN_LOOP2 :
      if(loop2finish==1) P_next=S_MAIN_LOOP3;  
      else P_next=S_MAIN_LOOP2;
   S_MAIN_LOOP3 :
      if(loop3finish==2) P_next=S_MAIN_SHOW; 
      else if(loop3finish==1)P_next=S_MAIN_LOOP1;
      else P_next=S_MAIN_LOOP3;   
   S_MAIN_SHOW:
      P_next = S_MAIN_SHOW;
  endcase
end

reg[9:0]initcount=0;

always@(posedge clk)
begin
    if(reset_n==0)
    begin
        initcount=0;
    end
    else if(initcount<500)
    begin
        initcount=initcount+1;
    end
    else
    begin
        initcount=500;
    end
end

// FSM ouput logic: Fetch the data bus of sram[] for display
always @(posedge clk) begin
  if (~reset_n) sample_data <= 8'b0;
  else if (sram_en && !sram_we) sample_data <= data_out;
end
// End of the main controller
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// The following code updates the 1602 LCD text messages.

assign abs_data = (sample_data < 0)? -sample_data : sample_data;

reg signed[7:0]f[0:1023];
reg signed[7:0]g  [0:63];
reg signed[30:0]c[0:959];
reg[6:0]gt=0;
reg[13:0] times=0;
reg recordfinish=0;
reg nextaddr=0;

always@(posedge clk)
begin
    if(reset_n==0)
    begin
        recordfinish<=0;
        times<=0;
        nextaddr<=0;
		gt<=0;
    end
    else if(P==S_MAIN_RECORD)
    begin
        if(recordfinish==0)
        begin
            if(times<=1023)
            begin
                f[times]<=sample_data;
                nextaddr<=0;
            end
            else if(times>1023&&times<1087)
            begin
                g[gt]<=sample_data;
                nextaddr<=0;
				gt<=gt+1;
            end
            else if(times==1087)
            begin
                g[gt]<=sample_data;
                nextaddr<=0;
 
				gt<=gt+1;
            end 
            else if(times==1088)
            begin
				recordfinish<=1;
                nextaddr<=0;
				gt<=gt+1;
            end 
			
            else 
            begin
                nextaddr=0;
            end
            times<=times+1;            
        end
        else 
        begin
            recordfinish<=1;
        end       
    end
	else if(P!=S_MAIN_RECORD)
	begin
		if(P_next==S_MAIN_RECORD) nextaddr<=1;
		else nextaddr<=0;
	end	
end

reg[9:0]x=0;
reg[6:0]k=0;
reg signed[30:0]sum=0;
reg signed[30:0]max=0;
reg [11:0]max_pos=0;
reg loop1finish=0;
reg loop2finish=0;
reg [1:0]loop3finish=0;
always@(posedge clk)
begin
    if(reset_n==0)
    begin
        x<=0;
        k<=0;
        sum<=0;
        max<=0;
        max_pos<=0;
        loop1finish<=0;
        loop2finish<=0;
        loop3finish<=0;
    end
    else if(P==S_MAIN_LOOP1)
    begin
        if(x<960)
        begin            
            if(loop1finish==0)
            begin
                sum<=0;
                loop1finish<=1;
                loop2finish<=0;
                loop3finish<=0;
            end            
        end
    end
    else if(P==S_MAIN_LOOP2)
    begin
        if(loop2finish==0)
        begin
            if(k<63)
            begin
                sum<=(sum+(f[(k+x)]*g[k]));
                loop2finish<=0;
                loop1finish<=0;
                loop3finish<=0;
                k<=k+1;
            end 
            else if(k==63)
            begin
                sum<=(sum+(f[(k+x)]*g[k]));
                loop2finish<=1;
                loop1finish<=0;
                loop3finish<=0;
                k<=0;
            end
        end

    end
    else if(P==S_MAIN_LOOP3)
    begin
        if(loop3finish==0)
        begin
            if(x<959)
            begin
                c[x]<=sum;
                if(sum>max)
                begin
                    max<=sum;
                    max_pos<=x;
                end
                loop3finish<=1;
                loop2finish<=0;
                loop1finish<=0;
                x<=x+1;
            end
            else if(x==959)
            begin
				c[x]<=sum;
                if(sum>max)
                begin
                    max<=sum;
                    max_pos<=x;
                end
                loop2finish<=0;
                loop1finish<=0;
                x<=x+1;
                loop3finish<=2;
            end
        end
    end
end

reg [30:0]showdata=0;

always@(posedge clk)
begin
	if(reset_n==0)
	begin
		showdata=0;
	end
	else if(P==S_MAIN_LOOP3&&P_next==S_MAIN_SHOW)
	begin
		showdata = (max < 0)? -max : max;
	end
end

always @(posedge clk) 
begin
	if(reset_n==0)
	begin
		row_A <= "Press BTN0 to do";
		row_B <= "x-correlation...";
	end
	else if(P==S_MAIN_SHOW)
	begin
		row_A[127:120]<="M";
		row_A[119:112]<="a";
		row_A[111:104]<="x";
		row_A[103:96]<=" ";
		row_A[95:88]<="v";
		row_A[87:80]<="a"; 
		row_A[79:72]<="l"  ;
		row_A[71:64]<="u";
		row_A[63:56]<="e";
		row_A[55:48]<=" "; 
		row_A[47:40]<=(showdata[23:20]>=0&&showdata[23:20]<10)?showdata[23:20]+48:showdata[23:20]+55;
		row_A[39:32]<=(showdata[19:16]>=0&&showdata[19:16]<10)?showdata[19:16]+48:showdata[19:16]+55;
		row_A[31:24]<=(showdata[15:12]>=0&&showdata[15:12]<10)?showdata[15:12]+48:showdata[15:12]+55;
		row_A[23:16]<=(showdata[11: 8]>=0&&showdata[11: 8]<10)?showdata[11: 8]+48:showdata[11: 8]+55;
		row_A[15:8] <=(showdata[ 7: 4]>=0&&showdata[ 7: 4]<10)?showdata[ 7: 4]+48:showdata[ 7: 4]+55;
		row_A[7:0]  <=(showdata[ 3: 0]>=0&&showdata[ 3: 0]<10)?showdata[ 3: 0]+48:showdata[ 3: 0]+55;
		
		
		row_B[127:120]<="M";
		row_B[119:112]<="a";
		row_B[111:104]<="x";
		row_B[103:96]<=" "; 
		row_B[95:88]<="l";
		row_B[87:80]<="o";
		row_B[79:72]<="c";  
		row_B[71:64]<="a";  
		row_B[63:56]<="t";  
		row_B[55:48]<="i";  
		row_B[47:40]<="o";  
		row_B[39:32]<="n";  
		row_B[31:24]<=" ";  
		row_B[23:16]<=(max_pos[11:8]>=0&&max_pos[11:8]<10)?max_pos[11:8]+48:max_pos[11:8]+55;
		row_B[15:8] <=(max_pos[ 7:4]>=0&&max_pos[ 7:4]<10)?max_pos[ 7:4]+48:max_pos[ 7:4]+55;  
		row_B[7:0]  <=(max_pos[ 3:0]>=0&&max_pos[ 3:0]<10)?max_pos[ 3:0]+48:max_pos[ 3:0]+55; 
	end
end
// End of the 1602 LCD text-updating code.
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// The circuit block that processes the user's button event.
always @(posedge clk) begin
  if (~reset_n)
    sample_addr <= 12'h000;
  else if (nextaddr==1)
    sample_addr <= (sample_addr < 2048)? sample_addr + 1 : sample_addr;
end
// End of the user's button control.
// ------------------------------------------------------------------------

endmodule
