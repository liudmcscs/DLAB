`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Dept. of Computer Science, National Chiao Tung University
// Engineer: Chun-Jen Tsai
// 
// Create Date: 2017/05/08 15:29:41
// Design Name: 
// Module Name: lab6
// Project Name: 
// Target Devices: 
// Tool Versions:
// Description: The sample top module of lab 6: sd card reader. The behavior of
//              this module is as follows
//              1. When the SD card is initialized, display a message on the LCD.
//                 If the initialization fails, an error message will be shown.
//              2. The user can then press usr_btn[2] to trigger the sd card
//                 controller to read the super block of the sd card (located at
//                 block # 8192) into the SRAM memory.
//              3. During SD card reading time, the four LED lights will be turned on.
//                 They will be turned off when the reading is done.
//              4. The LCD will then displayer the sector just been read, and the
//                 first byte of the sector.
//              5. Everytime you press usr_btn[2], the next byte will be displayed.
// 
// Dependencies: clk_divider, LCD_module, debounce, sd_card
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module lab6(
  // General system I/O ports
  input  clk,
  input  reset_n,
  input  [3:0] usr_btn,
  output [3:0] usr_led,
  // SD card specific I/O ports
  output spi_ss,
  output spi_sck,
  output spi_mosi,
  input  spi_miso,

  // 1602 LCD Module Interface
  output LCD_RS,
  output LCD_RW,
  output LCD_E,
  output [3:0] LCD_D
);

localparam [3:0] S_MAIN_INIT = 4'b0000, S_MAIN_IDLE = 4'b0001,
                 S_MAIN_WAIT = 4'b0010, S_MAIN_READ = 4'b0011,
                 S_MAIN_DONE = 4'b0100, S_MAIN_FIND = 4'b0101,
                 S_MAIN_COMP = 4'b0110, S_MAIN_SHOW = 4'b0111;

// Declare system variables
wire btn_level, btn_pressed;
reg  prev_btn_level;
reg  [5:0] send_counter;
reg  [3:0] P, P_next;
reg  [9:0] sd_counter;
reg  [7:0] data_byte;
reg  [31:0] blk_addr;

reg  [127:0] row_A = "SD1card1cannot  ";
reg  [127:0] row_B = "be initialized! ";
reg  done_flag; // Signals the completion of reading one SD sector.

// Declare SD card interface signals
wire clk_sel;
wire clk_500k;
reg  rd_req;
reg  [31:0] rd_addr;
wire init_finished;
wire [7:0] sd_dout;
wire sd_valid;

// Declare the control/data signals of an SRAM memory block
wire [7:0] data_in;
wire [7:0] data_out;
wire [8:0] sram_addr;
wire       sram_we, sram_en;

assign clk_sel = (init_finished)? clk : clk_500k; // clock for the SD controller
//assign usr_led = 4'h00;

clk_divider#(200) clk_divider0(
  .clk(clk),
  .reset(~reset_n),
  .clk_out(clk_500k)
);

debounce btn_db0(
  .clk(clk),
  .btn_input(usr_btn[2]),
  .btn_output(btn_level)
);

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

sd_card sd_card0(
  .cs(spi_ss),
  .sclk(spi_sck),
  .mosi(spi_mosi),
  .miso(spi_miso),

  .clk(clk_sel),
  .rst(~reset_n),
  .rd_req(rd_req),
  .block_addr(rd_addr),
  .init_finished(init_finished),
  .dout(sd_dout),
  .sd_valid(sd_valid)
);

sram ram0(
  .clk(clk),
  .we(sram_we),
  .en(sram_en),
  .addr(sram_addr),
  .data_i(data_in),
  .data_o(data_out)
);

//
// Enable one cycle of btn_pressed per each button hit
//
always @(posedge clk) begin
  if (~reset_n)
    prev_btn_level <= 0;
  else
    prev_btn_level <= btn_level;
end

assign btn_pressed = (btn_level == 1 && prev_btn_level == 0)? 1 : 0;

// ------------------------------------------------------------------------
// The following code sets the control signals of an SRAM memory block
// that is connected to the data output port of the SD controller.
// Once the read request is made to the SD controller, 512 bytes of data
// will be sequentially read into the SRAM memory block, one byte per
// clock cycle (as long as the sd_valid signal is high).
assign sram_we = sd_valid;          // Write data into SRAM when sd_valid is high.
assign sram_en = 1;                 // Always enable the SRAM block.
assign data_in = sd_dout;           // Input data always comes from the SD controller.
assign sram_addr = sd_counter[8:0]; // Set the driver of the SRAM address signal.
// End of the SRAM memory block
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// FSM of the SD card reader that reads the super block (512 bytes)
always @(posedge clk) begin
  if (~reset_n) begin
    P <= S_MAIN_INIT;
    done_flag <= 0;
  end
  else begin
    P <= P_next;
    if (P == S_MAIN_DONE)
      done_flag <= 1;
    else if (P == S_MAIN_FIND && P_next == S_MAIN_IDLE)
      done_flag <= 0;
    else
      done_flag <= done_flag;
  end
end

reg [7:0]word[0:7];
reg [1:0]dlabfind=0;
reg wfinish=0;
reg[7:0] record=0;

always @(*) begin // FSM next-state logic
  case (P)  
  
    S_MAIN_INIT: // wait for SD card initialization
      if (init_finished == 1) P_next = S_MAIN_IDLE;
      else P_next = S_MAIN_INIT;
    S_MAIN_IDLE: // wait for button click
      if (btn_pressed == 1) P_next = S_MAIN_WAIT;
      else P_next = S_MAIN_IDLE;
    S_MAIN_WAIT: // issue a rd_req to the SD controller until it's ready
      P_next = S_MAIN_READ;
    S_MAIN_READ: // wait for the input data to enter the SRAM buffer
      if (sd_counter == 512) P_next = S_MAIN_DONE;
      else P_next = S_MAIN_READ;
    S_MAIN_DONE: // read byte 512 of the superblock from sram[]
      P_next = S_MAIN_FIND;
    S_MAIN_FIND:
    P_next = S_MAIN_COMP;
    S_MAIN_COMP:
     if(wfinish==0&&sd_counter<512)P_next=S_MAIN_DONE;
	 else if(wfinish==0&&sd_counter==512)P_next=S_MAIN_WAIT;
     else if(wfinish==1)P_next=S_MAIN_SHOW;
	S_MAIN_SHOW:
	P_next=S_MAIN_SHOW; 
    default:
      P_next = S_MAIN_IDLE;
  endcase
end

always@(posedge clk)
begin
    if(reset_n==0)
    begin
        word[0]<=" ";
        word[1]<=" ";
        word[2]<=" ";
        word[3]<=" ";
        word[4]<=" ";
        word[5]<=" ";
        word[6]<=" ";
        word[7]<=" ";
    end
    
    else if(P==S_MAIN_FIND&&wfinish==0)
    begin
        word[7]<=data_out;
        word[6]<=word[7];
        word[5]<=word[6];
        word[4]<=word[5];
        word[3]<=word[4];
        word[2]<=word[3];
        word[1]<=word[2];
        word[0]<=word[1];
    end
end

always@(posedge clk)
begin
    if(reset_n==0)
    begin
		dlabfind<=0;
    end    
    else if(P==S_MAIN_COMP)
    begin
		if(word[0]=="D"&&word[1]=="L"&&word[2]=="A"&&word[3]=="B"&&word[4]=="_"&&word[5]=="T"&&word[6]=="A"&&word[7]=="G")
		begin
			dlabfind<=1;        
		end
    end
end

always@(posedge clk)
begin
    if(reset_n==0)
    begin
		record<=0;
    end    
    else if(P==S_MAIN_COMP)
    begin
        if(dlabfind==1&&wfinish==0)
        begin
            if(   (word[0]==" "&&word[1]=="t"&&word[2]=="h"&&word[3]=="e"&&word[4]==" ")||
				  (word[0]==10 &&word[1]=="t"&&word[2]=="h"&&word[3]=="e"&&word[4]==" ")||
				  (word[0]==" "&&word[1]=="t"&&word[2]=="h"&&word[3]=="e"&&word[4]== 10)||
				  (word[0]==10 &&word[1]=="t"&&word[2]=="h"&&word[3]=="e"&&word[4]== 10)||
                  (word[0]==" "&&word[1]=="T"&&word[2]=="h"&&word[3]=="e"&&word[4]==" ")||
				  (word[0]==10 &&word[1]=="T"&&word[2]=="h"&&word[3]=="e"&&word[4]== 10)||				  
				  (word[0]==10 &&word[1]=="T"&&word[2]=="h"&&word[3]=="e"&&word[4]==" ")||
				  (word[0]==" "&&word[1]=="T"&&word[2]=="h"&&word[3]=="e"&&word[4]== 10)    )
                  begin
                  record<=record+1;
                  end               
        end
    end
end

always@(posedge clk)
begin
    if(reset_n==0)
    begin
		wfinish<=0;
    end
    
    else if(P==S_MAIN_COMP)
    begin
		if(word[0]=="D"&&word[1]=="L"&&word[2]=="A"&&word[3]=="B"&&word[4]=="_"&&word[5]=="E"&&word[6]=="N"&&word[7]=="D")      
		begin
			wfinish<=1;
		end                
	end
end

// FSM output logic: controls the 'rd_req' and 'rd_addr' signals.
always @(*) begin
  rd_req = (P == S_MAIN_WAIT);
  rd_addr = blk_addr;
end

always @(posedge clk) begin
  if (~reset_n) blk_addr <= 32'h2000;
  else if(sd_counter==512) blk_addr <= blk_addr+1; // In lab 6, change this line to scan all blocks
end

// FSM output logic: controls the 'sd_counter' signal.
// SD card read address incrementer
always @(posedge clk) begin
  if (~reset_n || (P == S_MAIN_READ && P_next == S_MAIN_DONE))
    sd_counter <= 0;
  else if ((P == S_MAIN_READ && sd_valid) ||
           (P == S_MAIN_FIND))
    sd_counter <= sd_counter + 1;
    else if(sd_counter==512)sd_counter<=0;
end

// FSM ouput logic: Retrieves the content of sram[] for display
always @(posedge clk) begin
  if (~reset_n) data_byte <= 8'b0;
  else if (sram_en && P == S_MAIN_DONE) data_byte <= data_out;
end
// End of the FSM of the SD card reader
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// LCD Display function.
always @(posedge clk) begin
  if (~reset_n) begin
    row_A = "SD2card2cannot  ";
    row_B = "be initialized! ";
  end 
  else if (P==S_MAIN_SHOW) begin
  	row_A[127:120]<="F";
	row_A[119:112]<="o";
	row_A[111:104]<="u";
	row_A[103:96]<="n";
	row_A[95:88]<="d";
	row_A[87:80]<=" ";
	row_A[79:72]<=(record/10)+48;
	row_A[71:64]<=48+record-(record/10)*10;
	row_A[63:56]<=" ";
	row_A[55:48]<="m";
	row_A[47:40]<="a";
	row_A[39:32]<="t";	
	row_A[31:24]<="c";
	row_A[23:16]<="h";
	row_A[15:8] <="e";
	row_A[7:0]  <="s";	
    row_B<="in the text file";
  end
  else if (P == S_MAIN_IDLE) begin
    row_A <= "Hit BTN2 to read";
    row_B <= "the SD card ... ";
  end
end

assign usr_led[0]=P[0];
assign usr_led[1]=P[1];
assign usr_led[2]=P[2];
assign usr_led[3]=P[3];
// End of the LCD display function
// ------------------------------------------------------------------------

endmodule
