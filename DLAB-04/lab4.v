`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Dept. of CS, National Chiao Tung University
// Engineer: Chun-Jen Tsai
// 
// Create Date: 2017/04/27 15:06:57
// Design Name: UART I/O example for Arty
// Module Name: lab4
// Project Name: 
// Target Devices: Xilinx FPGA @ 100MHz
// Tool Versions: 
// Description: 
// 
// The parameters for the UART controller are 9600 baudrate, 8-N-1-N
//
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module lab4(
  input  clk,
  input  reset_n,
  input  [3:0] usr_btn,
  output [3:0] usr_led,
  input  uart_rx,
  output uart_tx
);

localparam [3:0] MAININIT=0, MAINPROMPT = 1,WAITKEY1 = 2, SECONDINIT=3, WAITKEY2=4, GCD=5, ANS=6 ;
localparam [1:0] S_UART_IDLE = 0, S_UART_WAIT = 1,
                 S_UART_SEND = 2, S_UART_INCR = 3;

// declare system variables
wire print_enable, print_done,finish;
reg [8:0] send_counter;
reg [4:0] P, P_next;
reg [1:0] Q, Q_next;
reg [23:0] init_counter;
reg [16:0]firnum;
reg [16:0]secnum;

// declare UART signals
wire transmit;
wire received;
wire [7:0] rx_byte;
reg  [7:0] rx_temp;
wire [7:0] tx_byte;
wire is_receiving;
wire is_transmitting;
wire recv_error;

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
localparam MEM_SIZE = 92;
localparam PROMPT = 0;
localparam SECOND_PROMPT = 35;
localparam ENTER_GCD = 71;
reg [7:0] data[0:MEM_SIZE-1];

initial begin
  { data[ 0], data[ 1], data[ 2], data[ 3], data[ 4], data[ 5], data[ 6], data[ 7],
    data[ 8], data[ 9], data[10], data[11], data[12], data[13], data[14], data[15],
    data[16], data[17], data[18], data[19], data[20], data[21], data[22], data[23],
    data[24], data[25], data[26], data[27], data[28], data[29], data[30], data[31],
    data[32], data[33], data[34] }
  <= { 8'h0D, 8'h0A, "Enter the first decimal number: ", 8'h00 };

  { data[35], data[36], data[37], data[38], data[39], data[40], data[41], data[42],
    data[43], data[44], data[45], data[46], data[47], data[48], data[49], data[50],
    data[51], data[52], data[53], data[54], data[55], data[56], data[57], data[58],
    data[59], data[60], data[61], data[62], data[63], data[64], data[65], data[66],
    data[67], data[68], data[69],data[70] }
  <= { 8'h0D, 8'h0A,"Enter the second decimal number: ",   8'h00 };
  
  { data[71], data[72], data[73], data[74], data[75], data[76], data[77],
    data[78], data[79], data[80], data[81], data[82], data[83], data[84],
    data[85], data[86], data[87], data[88], data[89], data[90], data[91]}
  <= { 8'h0D, 8'h0A, "The GCD is: 0x", 8'h00, 8'h00, 8'h00, 8'h00, 8'h00};
end

// Combinational I/O logics
assign enter_pressed = (rx_temp == 8'h0D);
assign num_pressed = (rx_temp>=8'h30&&rx_temp<=8'h39);
assign tx_byte =(P==WAITKEY1||P==WAITKEY2)?rx_byte:data[send_counter];

// ------------------------------------------------------------------------
// Main FSM that reads the UART input and triggers
// the output of the string "Hello, World!".
always @(posedge clk) begin
  if (~reset_n) P <= MAININIT;
  else P <= P_next;
end

reg [15:0]num1;
reg [15:0]num2;
reg [15:0]num11;
reg [15:0]num22;
reg [15:0]result;
reg f1=0;
reg f2=0;
reg start=0;
wire finish;

always@(posedge clk)
begin
	if(reset_n==0||P==MAININIT)
	begin
		num1=0;
		num2=0;
	end
	else if(P==WAITKEY1&&num_pressed) 
	begin
		num1=(num1*10)+(rx_temp-8'h30);
	end
	else if(P==WAITKEY2&&num_pressed)
	begin
		num2 <= (num2*10) + (rx_temp-8'h30);
	end			
	else
	begin
		num1=num1;
		num2=num2;
	end
end

always@(posedge clk)
begin
	if(reset_n==0||P==MAININIT)
	begin
		num11=0;
		num22=0;
		start=0;
		result=0;
	end
	else if(P==WAITKEY2&&enter_pressed)
	begin
		num11=num1;
		num22=num2;
	end
	else if(num11>num22)
	begin
		num11=num11-num22;
	end
	else if(num11<num22)
	begin
		num22=num22-num11;
	end
	else if(num22==num11)
	begin
		result=num11;
		start=1;
	end
end

always@(posedge clk)
begin
	if(start==1)
	begin
		data[87]<=(result[15:12]>=0&&result[15:12]<10)?result[15:12]+48:result[15:12]+55;
		data[88]<=(result[11:8]>=0&&result[11:8]<10)?result[11:8]+48:result[11:8]+55;
		data[89]<=(result[7:4]>=0&&result[7:4]<10)?result[7:4]+48:result[7:4]+55;
		data[90]<=(result[3:0]>=0&&result[3:0]<10)?result[3:0]+48:result[3:0]+55;
	end
end
assign finish=start;
always @(*) begin // FSM next-state logic
case (P)
	MAININIT: // Delay 10 us.
	if (init_counter < 1000) P_next = MAININIT;
	else P_next = MAINPROMPT;
	MAINPROMPT: // Print the prompt message.
	if (print_done) P_next = WAITKEY1;
	else P_next = MAINPROMPT;
	WAITKEY1: // wait for <Enter> key.
	if (enter_pressed) P_next = SECONDINIT;
	else P_next = WAITKEY1;
	SECONDINIT:
	if(print_done)P_next= WAITKEY2;
	else P_next=SECONDINIT;
	WAITKEY2:
	if(enter_pressed)P_next=GCD;
	else P_next=WAITKEY2;
	GCD:
	if(finish)P_next=ANS;
	else P_next=GCD;
	ANS:  
	if(print_done)P_next=MAININIT;
	else P_next=ANS;
endcase
end
assign usr_led=P;

// FSM output logics: print string control signals.
assign print_enable = (P != MAINPROMPT && P_next == MAINPROMPT) ||
					(P == WAITKEY1 && P_next == SECONDINIT)     ||
					(P == WAITKEY2 && P_next == GCD)				||
					(P==GCD&&P_next==ANS);
assign print_done = (tx_byte == 8'h0);

// Initialization counter.
always @(posedge clk) begin
  if (P == MAININIT) init_counter <= init_counter + 1;
  else init_counter <= 0;
end
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
assign transmit = (P==WAITKEY1||P==WAITKEY2)? received : (Q_next == S_UART_WAIT || print_enable);
assign tx_byte =(P==WAITKEY1||P==WAITKEY2)?rx_byte:data[send_counter];

// UART send_counter control circuit
always @(posedge clk) begin
  case (P_next)
    MAININIT: send_counter <= PROMPT;
    WAITKEY1: send_counter <= SECOND_PROMPT;
	WAITKEY2:send_counter <= ENTER_GCD;
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
// ------------------------------------------------------------------------

endmodule
