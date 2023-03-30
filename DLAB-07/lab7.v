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
  input  uart_rx,
  output uart_tx,

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
                 S_MAIN_COMP = 4'b0110, S_MAIN_PROM = 4'b0111,
				 S_MAIN_MULT1= 4'b1000,S_MAIN_SHOW1 = 4'b1001,
				 S_MAIN_MULT2= 4'b1010,S_MAIN_SHOW2 = 4'b1011,
				 S_MAIN_MULT3= 4'b1100,S_MAIN_SHOW3 = 4'b1101,
				 S_MAIN_OVER = 4'b1110; 
                  
localparam [1:0] S_UART_IDLE = 0, S_UART_WAIT = 1,
                 S_UART_SEND = 2, S_UART_INCR = 3;

// Declare system variables
wire print_enable, print_done,finish;
wire btn_level, btn_pressed;
reg  prev_btn_level;
reg  [8:0] send_counter;
reg  [3:0] P, P_next;
reg [1:0] Q, Q_next;
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
// declare UART signals
wire transmit;
wire received;
wire [7:0] rx_byte;
reg  [7:0] rx_temp;
wire [7:0] tx_byte;
wire is_receiving;
wire is_transmitting;
wire recv_error;




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

/* The UART device takes a 100MHz clock to handle I/O at 9600 baudrate */
uart uart(
  .clk(clk),
  .rst(~reset_n),
  .rx(uart_rx),
  .tx(uart_tx),
  .transmit(transmit),
  .tx_byte(tx_byte),
  .received(received),
  .rx_byte(rx_byte),
  .is_receiving(is_receiving),
  .is_transmitting(is_transmitting),
  .recv_error(recv_error)
);



// Initializes some strings.
// System Verilog has an easier way to initialize an array,
// but we are using Verilog 2005 :(
//

localparam MEM_SIZE = 45;
localparam PROMPT = 0;
localparam BEG_PART = 17;
localparam MID_PART = 28;
localparam END_PART = 36;
reg [7:0] data[0:MEM_SIZE-1];

initial begin
  { data[ 0], data[ 1], data[ 2], data[ 3], data[ 4], data[ 5], data[ 6], data[ 7],
    data[ 8], data[ 9], data[10], data[11], data[12], data[13], data[14], data[15],
    data[16]}
  <= { 8'h0D, 8'h0A, "the result is:",8'h00};

  { data[17], data[18], data[19], data[20], data[21], data[22], data[23], data[24], data[25], data[26],data[27]}
  <= {8'h0D, 8'h0A,"["," "," "," "," "," "," ",",",8'h00};
  
   { data[28], data[29], data[30], data[31], data[32], data[33], data[34] ,data[35]}
  <= {" "," "," "," "," "," ",",",8'h00};
  
  { data[36], data[37], data[38], data[39], data[40], data[41], data[42], data[43],
    data[44]}
  <= {" "," "," "," "," "," "," ","]",8'h00};  
  
  
end

// Combinational I/O logics

assign tx_byte = ( (P==S_MAIN_COMP&&P_next==S_MAIN_PROM) || (P==S_MAIN_MULT1&&P_next==S_MAIN_SHOW1) ||  (P==S_MAIN_MULT2&&P_next==S_MAIN_SHOW2)  ||  (P==S_MAIN_MULT3&&P_next==S_MAIN_SHOW3)  )?rx_byte:data[send_counter];
// ------------------------------------------------------------------------



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
reg matfind=0;
reg [5:0]wfinish=0;
reg [4:0]multimes=0;
reg [7:0]temp1=0,temp2=0;
reg promfinish=0;

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
     if(wfinish<32&&sd_counter<512)P_next=S_MAIN_DONE;
	 else if(wfinish<32&&sd_counter==512)P_next=S_MAIN_WAIT;
     else if(wfinish==32)P_next=S_MAIN_PROM;
	S_MAIN_PROM:
	 if(promfinish==1)P_next=S_MAIN_MULT1;
	 else P_next=S_MAIN_PROM;
	S_MAIN_MULT1:
	if(multimes==0||multimes==4||multimes==8||multimes==12)P_next=S_MAIN_SHOW1;
    else P_next=S_MAIN_MULT1;
    S_MAIN_SHOW1:
	P_next=S_MAIN_OVER;
	S_MAIN_MULT2:
	if(multimes==1||multimes==2||multimes==5||multimes==6||multimes==9||multimes==10||multimes==13||multimes==14)P_next=S_MAIN_SHOW2;
	else P_next=S_MAIN_MULT2;
	S_MAIN_SHOW2:
	P_next=S_MAIN_OVER;
	S_MAIN_MULT3:
	if(multimes==3||multimes==7||multimes==11||multimes==15)P_next=S_MAIN_SHOW3;
	else P_next=S_MAIN_SHOW3;
	S_MAIN_SHOW3:
	P_next=S_MAIN_OVER;
	S_MAIN_OVER:
	if(multimes==15)P_next=S_MAIN_OVER;
	else if(multimes==3||multimes==7||multimes==11)P_next=S_MAIN_MULT1;
	else if(multimes==0||multimes==1||multimes==4||multimes==5||multimes==8||multimes==9||multimes==12||multimes==13)P_next=S_MAIN_MULT2;
	else if(multimes==2||multimes==6||multimes==10||multimes==14)P_next=S_MAIN_MULT3;
	else P_next=S_MAIN_OVER;
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

reg[7:0]A[0:15];
reg[7:0]B[0:15];
reg [17:0]C;
reg [3:0]matseq=0;
always@(posedge clk)
begin
    if(reset_n==0)
    begin
    matfind<=0;
    end
    
    else if(P==S_MAIN_COMP&&matfind==0)
    begin
            if(word[0]=="M"&&word[1]=="A"&&word[2]=="T"&&word[3]=="X"&&word[4]=="_"&&word[5]=="T"&&word[6]=="A"&&word[7]=="G")
            begin
                matfind<=1;        
            end
    end
end

always@(posedge clk)
begin
    if(reset_n==0)
    begin
	matseq<=0;
    end
    
    else if(P==S_MAIN_COMP)
    begin
        if(matfind==1&&wfinish<32)
        begin
            if(word[0]==8'h0D&&word[1]==8'h0A/*&&word[2]=="h"&&word[3]=="e"*/&&word[4]==8'h0D&&word[5]==8'h0A)
			  begin
					if(wfinish<=15)
					begin
						A[matseq]<=(  (word[2]<="9")?(word[2]-48):(word[2]-55)  )*16+(  (word[3]<="9")?(word[3]-48):(word[3]-55)  );
					end
					else if(wfinish>=16&&wfinish<32)
					begin
						B[matseq]<=(  (word[2]<="9")?(word[2]-48):(word[2]-55)  )*16+(  (word[3]<="9")?(word[3]-48):(word[3]-55)  );
					end
					wfinish<=wfinish+1;
					matseq<=matseq+1;
			  end               
        end
        else if(wfinish==32)
        begin
            wfinish<=32;
        end
    end
end

always@(posedge clk)
begin
    if(wfinish==32&& (P==S_MAIN_MULT1)||(P==S_MAIN_MULT2)||(P==S_MAIN_MULT3) )
	begin
		if(multimes==0)
		begin
			C=A[0]*B[0]+A[4]*B[1]+A[8]*B[2]+A[12]*B[3];
		end
		else if(multimes==1)
		begin
			C=A[1]*B[0]+A[5]*B[1]+A[9]*B[2]+A[13]*B[3];
		end
		else if(multimes==2)
		begin
			C=A[2]*B[0]+A[6]*B[1]+A[10]*B[2]+A[14]*B[3];
		end		
		else if(multimes==3)
		begin
			C=A[3]*B[0]+A[7]*B[1]+A[11]*B[2]+A[15]*B[3];
		end		
		else if(multimes==4)
		begin
			C=A[0]*B[4]+A[4]*B[5]+A[8]*B[6]+A[12]*B[7];
		end		
		else if(multimes==5)
		begin
			C=A[1]*B[4]+A[5]*B[5]+A[9]*B[6]+A[13]*B[7];
		end		
		else if(multimes==6)
		begin
			C=A[2]*B[4]+A[6]*B[5]+A[10]*B[6]+A[14]*B[7];
		end				
		else if(multimes==7)
		begin
			C=A[3]*B[4]+A[7]*B[5]+A[11]*B[6]+A[15]*B[7];
		end		
		else if(multimes==8)
		begin
			C=A[0]*B[8]+A[4]*B[9]+A[8]*B[10]+A[12]*B[11];
		end			
		else if(multimes==9)
		begin
			C=A[1]*B[8]+A[5]*B[9]+A[9]*B[10]+A[13]*B[11];
		end		
		else if(multimes==10)
		begin
			C=A[2]*B[8]+A[6]*B[9]+A[10]*B[10]+A[14]*B[11];
		end				
		else if(multimes==11)
		begin
			C=A[3]*B[8]+A[7]*B[9]+A[11]*B[10]+A[15]*B[11];
		end			
		else if(multimes==12)
		begin
			C=A[0]*B[12]+A[4]*B[13]+A[8]*B[14]+A[12]*B[15];
		end			
		else if(multimes==13)
		begin
			C=A[1]*B[12]+A[5]*B[13]+A[9]*B[14]+A[13]*B[15];
		end		               
		else if(multimes==14)  
		begin                  
			C=A[2]*B[12]+A[6]*B[13]+A[10]*B[14]+A[14]*B[15];
		end				       
		else if(multimes==15)  
		begin                  
			C=A[3]*B[12]+A[7]*B[13]+A[11]*B[14]+A[15]*B[15];
		end				
	end
end

always@(posedge clk)
begin
	if(P==S_MAIN_MULT1)
	begin
		data[21]<=(C[17:16]<10&&C[17:16]>=0)?C[17:16]+48 :C[17:16]+55;
		data[22]<=(C[15:12]<10&&C[15:12]>=0)?C[15:12]+48 :C[15:12]+55;
		data[23]<=(C[11:8]<10&&C[11:8]>=0)?C[11:8]+48 :C[11:8]+55;
		data[24]<=(C[7:4]<10&&C[7:4]>=0)?C[7:4]+48 :C[7:4]+55;
		data[25]<=(C[3:0]<10&&C[3:0]>=0)?C[3:0]+48 :C[3:0]+55;	
	end
	else if(P==S_MAIN_MULT2)
	begin
		data[29]<=(C[17:16]<10&&C[17:16]>=0)?C[17:16]+48 :C[17:16]+55;
		data[30]<=(C[15:12]<10&&C[15:12]>=0)?C[15:12]+48 :C[15:12]+55;
		data[31]<=(C[11:8]<10&&C[11:8]>=0)?C[11:8]+48 :C[11:8]+55;
		data[32]<=(C[7:4]<10&&C[7:4]>=0)?C[7:4]+48 :C[7:4]+55;
		data[33]<=(C[3:0]<10&&C[3:0]>=0)?C[3:0]+48 :C[3:0]+55;	
	end
	else if(P==S_MAIN_MULT3)
	begin
		data[37]<=(C[17:16]<10&&C[17:16]>=0)?C[17:16]+48 :C[17:16]+55;
		data[38]<=(C[15:12]<10&&C[15:12]>=0)?C[15:12]+48 :C[15:12]+55;
		data[39]<=(C[11:8]<10&&C[11:8]>=0)?C[11:8]+48 :C[11:8]+55;
		data[40]<=(C[7:4]<10&&C[7:4]>=0)?C[7:4]+48 :C[7:4]+55;
		data[41]<=(C[3:0]<10&&C[3:0]>=0)?C[3:0]+48 :C[3:0]+55;	
	end
end

always@(posedge clk)
begin
    if(reset_n==0)multimes=0;
    else if(P==S_MAIN_OVER)
    begin
        if(multimes==16)multimes=16;
        else multimes=multimes+1;
    end
    
end


always@(posedge clk)
begin
	if(reset_n==0)
	begin
		promfinish=0;
	end
	else if(P==S_MAIN_PROM)
	begin
		promfinish=1;
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

  	row_A[127:120]<=(multimes[4]<10&&multimes[4]>=0)?multimes[4]+48 :multimes[4]+55;
	row_A[119:112]<=(multimes[3:0]<10&&multimes[3:0]>=0)?multimes[3:0]+48 :multimes[3:0]+55;
	row_A[111:104]<="A";
	row_A[103:96]<=(A[15][7:4]<10&&A[15][7:4]>=0)?A[15][7:4]+48 :A[15][7:4]+55;
	row_A[95:88]<=(A[15][3:0]<10&&A[15][3:0]>=0)?A[15][3:0]+48 :A[15][3:0]+55;
	row_A[87:80]<="B";
	row_A[79:72]<=(B[15][7:4]<10&&B[15][7:4]>=0)?B[15][7:4]+48 :B[15][7:4]+55;
	row_A[71:64]<=(B[15][7:4]<10&&B[15][7:4]>=0)?B[15][7:4]+48 :B[15][7:4]+55;
	row_A[63:56]<="C";
	row_A[55:48]<=(C[17:16]<10&&C[17:16]>=0)?C[17:16]+48 :C[17:16]+55;
	row_A[47:40]<=(C[15:12]<10&&C[15:12]>=0)?C[15:12]+48 :C[15:12]+55;
	row_A[39:32]<=(C[11:8]<10&&C[11:8]>=0)?C[11:8]+48 :C[11:8]+55;
	row_A[31:24]<=(C[7:4]<10&&C[7:4]>=0)?C[7:4]+48 :C[7:4]+55;
	row_A[23:16]<=(C[3:0]<10&&C[3:0]>=0)?C[3:0]+48 :C[3:0]+55;	
	row_A[15:8] <="P";
	row_A[7:0]  <=(P[3:0]<10&&P[3:0]>=0)? P[3:0]+48 :P[3:0]+55;
	
	
	row_B[127:120]<="w";
	row_B[119:112]<=(wfinish[5:4]<10&&wfinish[5:4]>=0)? wfinish[5:4]+48 :wfinish[5:4]+55;
	row_B[111:104]<=(wfinish[3:0]<10&&wfinish[3:0]>=0)? wfinish[3:0]+48 :wfinish[3:0]+55;	
	row_B[103:96]<="P";
	row_B[95:88]<=	promfinish+48;
	row_B[87:80]<="f";
	row_B[79:72]<=matfind+48;
	row_B[71:64]<=" ";
	row_B[63:56]<=(word[7]<10&&word[7]>=0)? word[7]+48 :word[7]+55;
	row_B[55:48]<=(word[6]<10&&word[6]>=0)? word[6]+48 :word[6]+55;
	row_B[47:40]<=(word[5]<10&&word[5]>=0)? word[5]+48 :word[5]+55;	
	row_B[39:32]<=(word[4]<10&&word[4]>=0)? word[4]+48 :word[4]+55;
	row_B[31:24]<=(word[3]<10&&word[3]>=0)? word[3]+48 :word[3]+55;	
	row_B[23:16]<=(word[2]<10&&word[2]>=0)? word[2]+48 :word[2]+55;
	row_B[15:8] <=(word[1]<10&&word[1]>=0)? word[2]+48 :word[1]+55;
	row_B[7:0]  <=(word[0]<10&&word[0]>=0)? word[0]+48 :word[0]+55;
	

end

// End of the LCD display function
// ------------------------------------------------------------------------


// FSM output logics: print string control signals.
assign print_enable = (P != S_MAIN_PROM && P_next == S_MAIN_PROM) ||
                  (P == S_MAIN_MULT1 && P_next == S_MAIN_SHOW1)    ||
				  (P == S_MAIN_MULT2 && P_next == S_MAIN_SHOW2)	  ||
				  (P == S_MAIN_MULT3 && P_next == S_MAIN_SHOW3)
				  ;
assign print_done = (tx_byte == 8'h0);

// End of the FSM of the print string controller
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// FSM of the controller to send a string to the UART.
always @(posedge clk) begin
  if (~reset_n) Q <= S_UART_IDLE;
  else Q <= Q_next;
end

always @(*) begin // FSM next-state logic
  case (Q)
    S_UART_IDLE: // wait for the print_string flag
      if (print_enable) Q_next = S_UART_WAIT;
      else Q_next = S_UART_IDLE;
    S_UART_WAIT: // wait for the transmission of current data byte begins
      if (is_transmitting == 1) Q_next = S_UART_SEND;
      else Q_next = S_UART_WAIT;
    S_UART_SEND: // wait for the transmission of current data byte finishes
      if (is_transmitting == 0) Q_next = S_UART_INCR; // transmit next character
      else Q_next = S_UART_SEND;
    S_UART_INCR:
      if (tx_byte == 8'h0) Q_next = S_UART_IDLE; // string transmission ends
      else Q_next = S_UART_WAIT;
  endcase
end

// FSM output logics
assign transmit = ( (P==S_MAIN_COMP&&P_next==S_MAIN_PROM) || (P==S_MAIN_MULT1&&P_next==S_MAIN_SHOW1) ||  (P==S_MAIN_MULT2&&P_next==S_MAIN_SHOW2)  ||  (P==S_MAIN_MULT3&&P_next==S_MAIN_SHOW3)  )? received : (Q_next == S_UART_WAIT || print_enable);
assign tx_byte = ( (P==S_MAIN_COMP&&P_next==S_MAIN_PROM) || (P==S_MAIN_MULT1&&P_next==S_MAIN_SHOW1) ||  (P==S_MAIN_MULT2&&P_next==S_MAIN_SHOW2)  ||  (P==S_MAIN_MULT3&&P_next==S_MAIN_SHOW3)  )?rx_byte:data[send_counter];

// UART send_counter control circuit
always @(posedge clk) begin
  case (P_next)
    S_MAIN_COMP: send_counter <= PROMPT;
    S_MAIN_MULT1:send_counter <= BEG_PART;
	S_MAIN_MULT2:send_counter <= MID_PART;
	S_MAIN_MULT3:send_counter <= END_PART;
	
    default: send_counter <= send_counter + (Q_next == S_UART_INCR);
  endcase
end
// End of the FSM of the print string controller
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// The following logic stores the UART input in a temporary buffer.
// The input character will stay in the buffer for one clock cycle.
always @(posedge clk) begin
  rx_temp <= (received)? rx_byte : 8'h0;
  
end
endmodule
