module vga_on_de2_115
	(
		////////////////////	Clock Input	 	////////////////////	 
		CLOCK_27,						//	27 MHz
		CLOCK_50,						//	50 MHz
		EXT_CLOCK,						//	External Clock
		////////////////////	Push Button		////////////////////
		KEY,							//	Pushbutton[3:0]
		////////////////////	DPDT Switch		////////////////////
		SW,								//	Toggle Switch[17:0]
		////////////////////	7-SEG Dispaly	////////////////////
		HEX0,							//	Seven Segment Digit 0
		HEX1,							//	Seven Segment Digit 1
		HEX2,							//	Seven Segment Digit 2
		HEX3,							//	Seven Segment Digit 3
		HEX4,							//	Seven Segment Digit 4
		HEX5,							//	Seven Segment Digit 5
		HEX6,							//	Seven Segment Digit 6
		HEX7,							//	Seven Segment Digit 7
		////////////////////////	LED		////////////////////////
		LEDG,							//	LED Green[8:0]
		LEDR,							//	LED Red[17:0]
		////////////////////////	UART	////////////////////////
		UART_TXD,						//	UART Transmitter
		UART_RXD,						//	UART Receiver
		////////////////////////	IRDA	////////////////////////
		IRDA_TXD,						//	IRDA Transmitter
		IRDA_RXD,						//	IRDA Receiver
		/////////////////////	SDRAM Interface		////////////////
		DRAM_DQ,						//	SDRAM Data bus 16 Bits
		DRAM_ADDR,						//	SDRAM Address bus 12 Bits
		DRAM_LDQM,						//	SDRAM Low-byte Data Mask 
		DRAM_UDQM,						//	SDRAM High-byte Data Mask
		DRAM_WE_N,						//	SDRAM Write Enable
		DRAM_CAS_N,						//	SDRAM Column Address Strobe
		DRAM_RAS_N,						//	SDRAM Row Address Strobe
		DRAM_CS_N,						//	SDRAM Chip Select
		DRAM_BA_0,						//	SDRAM Bank Address 0
		DRAM_BA_1,						//	SDRAM Bank Address 0
		DRAM_CLK,						//	SDRAM Clock
		DRAM_CKE,						//	SDRAM Clock Enable
		////////////////////	Flash Interface		////////////////
		FL_DQ,							//	FLASH Data bus 8 Bits
		FL_ADDR,						//	FLASH Address bus 22 Bits
		FL_WE_N,						//	FLASH Write Enable
		FL_RST_N,						//	FLASH Reset
		FL_OE_N,						//	FLASH Output Enable
		FL_CE_N,						//	FLASH Chip Enable
		////////////////////	SRAM Interface		////////////////
		SRAM_DQ,						//	SRAM Data bus 16 Bits
		SRAM_ADDR,						//	SRAM Address bus 18 Bits
		SRAM_UB_N,						//	SRAM High-byte Data Mask 
		SRAM_LB_N,						//	SRAM Low-byte Data Mask 
		SRAM_WE_N,						//	SRAM Write Enable
		SRAM_CE_N,						//	SRAM Chip Enable
		SRAM_OE_N,						//	SRAM Output Enable
		////////////////////	ISP1362 Interface	////////////////
		OTG_DATA,						//	ISP1362 Data bus 16 Bits
		OTG_ADDR,						//	ISP1362 Address 2 Bits
		OTG_CS_N,						//	ISP1362 Chip Select
		OTG_RD_N,						//	ISP1362 Write
		OTG_WR_N,						//	ISP1362 Read
		OTG_RST_N,						//	ISP1362 Reset
		OTG_FSPEED,						//	USB Full Speed,	0 = Enable, Z = Disable
		OTG_LSPEED,						//	USB Low Speed, 	0 = Enable, Z = Disable
		OTG_INT0,						//	ISP1362 Interrupt 0
		OTG_INT1,						//	ISP1362 Interrupt 1
		OTG_DREQ0,						//	ISP1362 DMA Request 0
		OTG_DREQ1,						//	ISP1362 DMA Request 1
		OTG_DACK0_N,					//	ISP1362 DMA Acknowledge 0
		OTG_DACK1_N,					//	ISP1362 DMA Acknowledge 1
		////////////////////	LCD Module 16X2		////////////////
		LCD_ON,							//	LCD Power ON/OFF
		LCD_BLON,						//	LCD Back Light ON/OFF
		LCD_RW,							//	LCD Read/Write Select, 0 = Write, 1 = Read
		LCD_EN,							//	LCD Enable
		LCD_RS,							//	LCD Command/Data Select, 0 = Command, 1 = Data
		LCD_DATA,						//	LCD Data bus 8 bits
		////////////////////	SD_Card Interface	////////////////
		SD_DAT,							//	SD Card Data
		SD_DAT3,						//	SD Card Data 3
		SD_CMD,							//	SD Card Command Signal
		SD_CLK,							//	SD Card Clock
		////////////////////	USB JTAG link	////////////////////
		TDI,  							// CPLD -> FPGA (data in)
		TCK,  							// CPLD -> FPGA (clk)
		TCS,  							// CPLD -> FPGA (CS)
		TDO,  							// FPGA -> CPLD (data out)
		////////////////////	I2C		////////////////////////////
		I2C_SDAT,						//	I2C Data
		I2C_SCLK,						//	I2C Clock
		////////////////////	PS2		////////////////////////////
		PS2_DAT,						//	PS2 Data
		PS2_CLK,						//	PS2 Clock
		////////////////////	VGA		////////////////////////////
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK,						//	VGA BLANK
		VGA_SYNC,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,  						//	VGA Blue[9:0]
		////////////	Ethernet Interface	////////////////////////
		ENET_DATA,						//	DM9000A DATA bus 16Bits
		ENET_CMD,						//	DM9000A Command/Data Select, 0 = Command, 1 = Data
		ENET_CS_N,						//	DM9000A Chip Select
		ENET_WR_N,						//	DM9000A Write
		ENET_RD_N,						//	DM9000A Read
		ENET_RST_N,						//	DM9000A Reset
		ENET_INT,						//	DM9000A Interrupt
		ENET_CLK,						//	DM9000A Clock 25 MHz
		////////////////	Audio CODEC		////////////////////////
		AUD_ADCLRCK,					//	Audio CODEC ADC LR Clock
		AUD_ADCDAT,						//	Audio CODEC ADC Data
		AUD_DACLRCK,					//	Audio CODEC DAC LR Clock
		AUD_DACDAT,						//	Audio CODEC DAC Data
		AUD_BCLK,						//	Audio CODEC Bit-Stream Clock
		AUD_XCK,						//	Audio CODEC Chip Clock
		////////////////	TV Decoder		////////////////////////
		TD_DATA,    					//	TV Decoder Data bus 8 bits
		TD_HS,							//	TV Decoder H_SYNC
		TD_VS,							//	TV Decoder V_SYNC
		TD_RESET,						//	TV Decoder Reset
		////////////////////	GPIO	////////////////////////////
		GPIO							//	GPIO Connection 0
		
	);

////////////////////////	Clock Input	 	////////////////////////
input			CLOCK_27;				//	27 MHz
input			CLOCK_50;				//	50 MHz
input			EXT_CLOCK;				//	External Clock
////////////////////////	Push Button		////////////////////////
input	[3:0]	KEY;					//	Pushbutton[3:0]
////////////////////////	DPDT Switch		////////////////////////
input	[17:0]	SW;						//	Toggle Switch[17:0]
////////////////////////	7-SEG Dispaly	////////////////////////
output	[6:0]	HEX0;					//	Seven Segment Digit 0
output	[6:0]	HEX1;					//	Seven Segment Digit 1
output	[6:0]	HEX2;					//	Seven Segment Digit 2
output	[6:0]	HEX3;					//	Seven Segment Digit 3
output	[6:0]	HEX4;					//	Seven Segment Digit 4
output	[6:0]	HEX5;					//	Seven Segment Digit 5
output	[6:0]	HEX6;					//	Seven Segment Digit 6
output	[6:0]	HEX7;					//	Seven Segment Digit 7
////////////////////////////	LED		////////////////////////////
output	[8:0]	LEDG;					//	LED Green[8:0]
output	[17:0]	LEDR;					//	LED Red[17:0]
////////////////////////////	UART	////////////////////////////
output			UART_TXD;				//	UART Transmitter
input			UART_RXD;				//	UART Receiver
////////////////////////////	IRDA	////////////////////////////
output			IRDA_TXD;				//	IRDA Transmitter
input			IRDA_RXD;				//	IRDA Receiver
///////////////////////		SDRAM Interface	////////////////////////
inout	[15:0]	DRAM_DQ;				//	SDRAM Data bus 16 Bits
output	[11:0]	DRAM_ADDR;				//	SDRAM Address bus 12 Bits
output			DRAM_LDQM;				//	SDRAM Low-byte Data Mask 
output			DRAM_UDQM;				//	SDRAM High-byte Data Mask
output			DRAM_WE_N;				//	SDRAM Write Enable
output			DRAM_CAS_N;				//	SDRAM Column Address Strobe
output			DRAM_RAS_N;				//	SDRAM Row Address Strobe
output			DRAM_CS_N;				//	SDRAM Chip Select
output			DRAM_BA_0;				//	SDRAM Bank Address 0
	output			DRAM_BA_1;				//	SDRAM Bank Address 0
output			DRAM_CLK;				//	SDRAM Clock
output			DRAM_CKE;				//	SDRAM Clock Enable
////////////////////////	Flash Interface	////////////////////////
inout	[7:0]	FL_DQ;					//	FLASH Data bus 8 Bits
output	[21:0]	FL_ADDR;				//	FLASH Address bus 22 Bits
output			FL_WE_N;				//	FLASH Write Enable
output			FL_RST_N;				//	FLASH Reset
output			FL_OE_N;				//	FLASH Output Enable
output			FL_CE_N;				//	FLASH Chip Enable
////////////////////////	SRAM Interface	////////////////////////
inout	[15:0]	SRAM_DQ;				//	SRAM Data bus 16 Bits
output	[17:0]	SRAM_ADDR;				//	SRAM Address bus 18 Bits
output			SRAM_UB_N;				//	SRAM High-byte Data Mask 
output			SRAM_LB_N;				//	SRAM Low-byte Data Mask 
output			SRAM_WE_N;				//	SRAM Write Enable
output			SRAM_CE_N;				//	SRAM Chip Enable
output			SRAM_OE_N;				//	SRAM Output Enable
////////////////////	ISP1362 Interface	////////////////////////
inout	[15:0]	OTG_DATA;				//	ISP1362 Data bus 16 Bits
output	[1:0]	OTG_ADDR;				//	ISP1362 Address 2 Bits
output			OTG_CS_N;				//	ISP1362 Chip Select
output			OTG_RD_N;				//	ISP1362 Write
output			OTG_WR_N;				//	ISP1362 Read
output			OTG_RST_N;				//	ISP1362 Reset
output			OTG_FSPEED;				//	USB Full Speed,	0 = Enable, Z = Disable
output			OTG_LSPEED;				//	USB Low Speed, 	0 = Enable, Z = Disable
input			OTG_INT0;				//	ISP1362 Interrupt 0
input			OTG_INT1;				//	ISP1362 Interrupt 1
input			OTG_DREQ0;				//	ISP1362 DMA Request 0
input			OTG_DREQ1;				//	ISP1362 DMA Request 1
output			OTG_DACK0_N;			//	ISP1362 DMA Acknowledge 0
output			OTG_DACK1_N;			//	ISP1362 DMA Acknowledge 1
////////////////////	LCD Module 16X2	////////////////////////////
inout	[7:0]	LCD_DATA;				//	LCD Data bus 8 bits
output			LCD_ON;					//	LCD Power ON/OFF
output			LCD_BLON;				//	LCD Back Light ON/OFF
output			LCD_RW;					//	LCD Read/Write Select, 0 = Write, 1 = Read
output			LCD_EN;					//	LCD Enable
output			LCD_RS;					//	LCD Command/Data Select, 0 = Command, 1 = Data
////////////////////	SD Card Interface	////////////////////////
inout			SD_DAT;					//	SD Card Data
inout			SD_DAT3;				//	SD Card Data 3
inout			SD_CMD;					//	SD Card Command Signal
output			SD_CLK;					//	SD Card Clock
////////////////////////	I2C		////////////////////////////////
inout			I2C_SDAT;				//	I2C Data
output			I2C_SCLK;				//	I2C Clock
////////////////////////	PS2		////////////////////////////////
input		 	PS2_DAT;				//	PS2 Data
input			PS2_CLK;				//	PS2 Clock
////////////////////	USB JTAG link	////////////////////////////
input  			TDI;					// CPLD -> FPGA (data in)
input  			TCK;					// CPLD -> FPGA (clk)
input  			TCS;					// CPLD -> FPGA (CS)
output 			TDO;					// FPGA -> CPLD (data out)
////////////////////////	VGA			////////////////////////////
output			VGA_CLK;   				//	VGA Clock
output			VGA_HS;					//	VGA H_SYNC
output			VGA_VS;					//	VGA V_SYNC
output			VGA_BLANK;				//	VGA BLANK
output			VGA_SYNC;				//	VGA SYNC
output	[9:0]	VGA_R;   				//	VGA Red[9:0]
output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
////////////////	Ethernet Interface	////////////////////////////
inout	[15:0]	ENET_DATA;				//	DM9000A DATA bus 16Bits
output			ENET_CMD;				//	DM9000A Command/Data Select, 0 = Command, 1 = Data
output			ENET_CS_N;				//	DM9000A Chip Select
output			ENET_WR_N;				//	DM9000A Write
output			ENET_RD_N;				//	DM9000A Read
output			ENET_RST_N;				//	DM9000A Reset
input			ENET_INT;				//	DM9000A Interrupt
output			ENET_CLK;				//	DM9000A Clock 25 MHz
////////////////////	Audio CODEC		////////////////////////////
inout			AUD_ADCLRCK;			//	Audio CODEC ADC LR Clock
input			AUD_ADCDAT;				//	Audio CODEC ADC Data
inout			AUD_DACLRCK;			//	Audio CODEC DAC LR Clock
output			AUD_DACDAT;				//	Audio CODEC DAC Data
inout			AUD_BCLK;				//	Audio CODEC Bit-Stream Clock
output			AUD_XCK;				//	Audio CODEC Chip Clock
////////////////////	TV Devoder		////////////////////////////
input	[7:0]	TD_DATA;    			//	TV Decoder Data bus 8 bits
input			TD_HS;					//	TV Decoder H_SYNC
input			TD_VS;					//	TV Decoder V_SYNC
output			TD_RESET;				//	TV Decoder Reset
////////////////////////	GPIO	////////////////////////////////
output	[35:0]	GPIO;					//	GPIO Connection 0


//	Turn on all display
assign	HEX0		=	7'h00;
assign	HEX1		=	7'h00;
assign	HEX2		=	7'h00;
assign	HEX3		=	7'h00;
assign	HEX4		=	7'h00;
assign	HEX5		=	7'h00;
assign	HEX6		=	7'h00;
assign	HEX7		=	7'h00;
assign	LCD_ON		=	1'b1;
assign	LCD_BLON	=	1'b1;

//	All inout port turn to tri-state
assign	DRAM_DQ		=	16'hzzzz;
assign	FL_DQ		=	8'hzz;
assign	SRAM_DQ		=	16'hzzzz;
assign	OTG_DATA	=	16'hzzzz;
assign	LCD_DATA	=	8'hzz;
assign	SD_DAT		=	1'bz;
assign	I2C_SDAT	=	1'bz;
assign	AUD_ADCLRCK	=	1'bz;
assign	AUD_DACLRCK	=	1'bz;
assign	AUD_BCLK	=	1'bz;

vga actual_module(CLOCK_50, SW[0], KEY[0], GPIO[29], GPIO[27], GPIO[31], LEDG[0], LEDR[17:0]);

endmodule

module vga(clk, rst, en, SS, SCLK, MOSI, done, debug);
input clk, rst;
input en;
output reg done;
output reg SS, SCLK, MOSI;
output [17:0]debug;

reg [3:0]S;
reg [3:0]NS;

parameter START = 4'd0,
			WAIT_FOR_EN = 4'd1,
			WAIT_FOR_EN_PULSE = 4'd9,
			SS_HIGH = 4'd2,
			SS_LOW = 4'd3,
			MOSI_DATA_SEND = 4'd4,
			MOSI_DATA = 4'd5,
			SCLK_HIGH = 4'd6,
			SCLK_LOW = 4'd7,
			DONE_SIGNAL_PULSE = 4'd8,
			ERROR = 4'hFF;
parameter MOSI_OFFSET_TICKS = 10'd50, // 2 ticks, but 1 less since state MOSI_DATA_SEND to update packets
				SCLK_TICKS_HALF_PERIOD_MINUS_OFFSET = 10'd200,
				SCLK_TICKS_HALF_PERIOD = 10'd250,
				SCLK_TICKS_FULL_PERIOD = 10'd500; // 100kHz SPI clock
			 
// hard coded data to send
parameter DATA = 8'b10110011;
parameter DATA_SIZE = 4'd8;
		
// variables to record ticks and data sent
reg [9:0] clk_50MHz_ticks_ss_high;
reg [9:0] clk_50MHz_ticks_ss_low;
reg [9:0] clk_50MHz_ticks_MOSI_DATA;
reg [9:0] clk_50MHz_ticks_SCLK_high;
reg [9:0] clk_50MHz_ticks_SCLK_low;
reg [3:0] sending_bit;
reg [7:0] data_to_send;	
	
assign debug[3:0] = S;
assign debug[4] = 1'b0;
assign debug[5] = 1'b1;
assign debug[9:6] = sending_bit;
assign debug[17:10] = clk_50MHz_ticks_SCLK_low;

always @(*)
begin
	case (S)
		START:
		begin
			NS = WAIT_FOR_EN;
		end
		WAIT_FOR_EN: 
		begin
			if (en == 1'b0)
				NS = WAIT_FOR_EN_PULSE;
			else
				NS = WAIT_FOR_EN;
		end
		WAIT_FOR_EN_PULSE: 
		begin
			if (en == 1'b1)
				NS = SS_HIGH;
			else
				NS = WAIT_FOR_EN_PULSE;
		end
		SS_HIGH:
		begin
			if (clk_50MHz_ticks_ss_high == SCLK_TICKS_FULL_PERIOD) // WAIT a whole SPI clock period
				NS = SS_LOW;
			else
				NS = SS_HIGH;
		end
		SS_LOW:
		begin
			if (clk_50MHz_ticks_ss_low == SCLK_TICKS_FULL_PERIOD) // WAIT a whole SPI clock period
				NS = MOSI_DATA_SEND;
			else
				NS = SS_LOW;
		end
		MOSI_DATA_SEND:
			NS = MOSI_DATA;
		MOSI_DATA:
		begin
			if (clk_50MHz_ticks_MOSI_DATA == MOSI_OFFSET_TICKS) // WAIT a short period - MOSI_TICKS
				NS = SCLK_HIGH;
			else
				NS = MOSI_DATA;
		end
		SCLK_HIGH:
		begin
			if (clk_50MHz_ticks_SCLK_high == SCLK_TICKS_HALF_PERIOD) // WAIT for a half period of the 250KHz
				NS = SCLK_LOW;
			else
				NS = SCLK_HIGH;
		end		
		SCLK_LOW:
		begin
			if (clk_50MHz_ticks_SCLK_low == SCLK_TICKS_HALF_PERIOD_MINUS_OFFSET) // WAIT for a half period of the SPI Clock minus the MOSI OFFSET
				if (sending_bit == DATA_SIZE)
					NS = DONE_SIGNAL_PULSE;
				else
					NS = MOSI_DATA_SEND;
			else
				NS = SCLK_LOW;
		end
		DONE_SIGNAL_PULSE:
		begin
			NS = WAIT_FOR_EN;
		end
		default NS = ERROR;
	endcase
end

always @(posedge clk or negedge rst)
begin
	if (rst == 1'b0)
	begin
		done <= 1'b0;
		clk_50MHz_ticks_ss_high <= 10'd0;
		clk_50MHz_ticks_ss_low <= 10'd0;
		clk_50MHz_ticks_MOSI_DATA <= 10'd0;
		clk_50MHz_ticks_SCLK_high <= 10'd0;
		clk_50MHz_ticks_SCLK_low <= 10'd0;
		sending_bit <= 4'd0;
		SS <= 1'b1;
		SCLK <= 1'b0;
		MOSI <= 1'b0;
	end
	else
		case (S)
			START:
				data_to_send = DATA;
			WAIT_FOR_EN: 
			begin
				done <= 1'b0;
				clk_50MHz_ticks_ss_high <= 10'd0;
				clk_50MHz_ticks_ss_low <= 10'd0;
				clk_50MHz_ticks_MOSI_DATA <= 10'd0;
				clk_50MHz_ticks_SCLK_high <= 10'd0;
				clk_50MHz_ticks_SCLK_low <= 10'd0;
				sending_bit <= 4'd0;
			end
			SS_HIGH:
			begin
				clk_50MHz_ticks_ss_high <= clk_50MHz_ticks_ss_high + 1'b1;
				SS <= 1'b1;
			end
			SS_LOW:
			begin
				clk_50MHz_ticks_ss_low <= clk_50MHz_ticks_ss_low + 1'b1;
				SS <= 1'b0;
			end
			MOSI_DATA_SEND:
			begin
				clk_50MHz_ticks_SCLK_low <= 10'd0; // reset
				sending_bit <= sending_bit + 1'b1;
				SCLK <= 1'b0;
				MOSI <= data_to_send[8'd7-sending_bit];
			end
			MOSI_DATA:
			begin
				clk_50MHz_ticks_MOSI_DATA <= clk_50MHz_ticks_MOSI_DATA + 1'b1;
			end
			SCLK_HIGH:
			begin
				clk_50MHz_ticks_MOSI_DATA <= 10'd0; // reset
				clk_50MHz_ticks_SCLK_high <= clk_50MHz_ticks_SCLK_high + 1'b1;
				SCLK <= 1'b1;
			end		
			SCLK_LOW:
			begin
				clk_50MHz_ticks_SCLK_high <= 10'd0; // reset
				clk_50MHz_ticks_SCLK_low <= clk_50MHz_ticks_SCLK_low + 1'b1;
				SCLK <= 1'b0;
			end
			DONE_SIGNAL_PULSE:
			begin
				done <= 1'b1;
				MOSI <= 1'b0; // turn off, don't need as defined by waveform
				data_to_send <= data_to_send + 1'b1;
			end
		endcase
end

/* FSM init and NS always */
always @(posedge clk or negedge rst)
begin
	if (rst == 1'b0)
	begin
		S <= START;
	end
	else
	begin
		S <= NS;
	end
end

	
endmodule
