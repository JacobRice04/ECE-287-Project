module vga_driver_to_frame_buf	(
    	//////////// ADC //////////
	//output		          		ADC_CONVST,
	//output		          		ADC_DIN,
	//input 		          		ADC_DOUT,
	//output		          		ADC_SCLK,

	//////////// Audio //////////
	//input 		          		AUD_ADCDAT,
	//inout 		          		AUD_ADCLRCK,
	//inout 		          		AUD_BCLK,
	//output		          		AUD_DACDAT,
	//inout 		          		AUD_DACLRCK,
	//output		          		AUD_XCK,

	//////////// CLOCK //////////
	//input 		          		CLOCK2_50,
	//input 		          		CLOCK3_50,
	//input 		          		CLOCK4_50,
	input 		          		CLOCK_50,

	//////////// SDRAM //////////
	//output		    [12:0]		DRAM_ADDR,
	//output		     [1:0]		DRAM_BA,
	//output		          		DRAM_CAS_N,
	//output		          		DRAM_CKE,
	//output		          		DRAM_CLK,
	//output		          		DRAM_CS_N,
	//inout 		    [15:0]		DRAM_DQ,
	//output		          		DRAM_LDQM,
	//output		          		DRAM_RAS_N,
	//output		          		DRAM_UDQM,
	//output		          		DRAM_WE_N,

	//////////// I2C for Audio and Video-In //////////
	//output		          		FPGA_I2C_SCLK,
	//inout 		          		FPGA_I2C_SDAT,

	//////////// SEG7 //////////
	output		     [6:0]		HEX0,
	output		     [6:0]		HEX1,
	output		     [6:0]		HEX2,
	output		     [6:0]		HEX3,
	//output		     [6:0]		HEX4,
	//output		     [6:0]		HEX5,

	//////////// IR //////////
	//input 		          		IRDA_RXD,
	//output		          		IRDA_TXD,

	//////////// KEY //////////
	input 		     [3:0]		KEY,

	//////////// LED //////////
	output		     [9:0]		LEDR,

	//////////// PS2 //////////
	//inout 		          		PS2_CLK,
	//inout 		          		PS2_CLK2,
	//inout 		          		PS2_DAT,
	//inout 		          		PS2_DAT2,

	//////////// SW //////////
	input 		     [9:0]		SW,

	//////////// Video-In //////////
	//input 		          		TD_CLK27,
	//input 		     [7:0]		TD_DATA,
	//input 		          		TD_HS,
	//output		          		TD_RESET_N,
	//input 		          		TD_VS,

	//////////// VGA //////////
	output		          		VGA_BLANK_N,
	output		     [7:0]		VGA_B,
	output		          		VGA_CLK,
	output		     [7:0]		VGA_G,
	output		          		VGA_HS,
	output		     [7:0]		VGA_R,
	output		          		VGA_SYNC_N,
	output		          		VGA_VS

	//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
	//inout 		    [35:0]		GPIO_0,

	//////////// GPIO_1, GPIO_1 connect to GPIO Default //////////
	//inout 		    [35:0]		GPIO_1

);

// Turn off all displays.
assign	HEX0		=	7'h00;
assign	HEX1		=	7'h00;
assign	HEX2		=	7'h00;
assign	HEX3		=	7'h00;

// DONE STANDARD PORT DECLARATION ABOVE
/* HANDLE SIGNALS FOR CIRCUIT */
wire clk;
wire rst;

assign clk = CLOCK_50;
assign rst = KEY[0];

wire [9:0]SW_db;

debounce_switches db(
.clk(clk),
.rst(rst),
.SW(SW), 
.SW_db(SW_db)
);

// VGA DRIVER
wire active_pixels; // is on when we're in the active draw space
wire frame_done;
wire [9:0]x; // current x
wire [9:0]y; // current y - 10 bits = 1024 ... a little bit more than we need

/* the 3 signals to set to write to the picture */
reg [14:0] the_vga_draw_frame_write_mem_address;
reg [23:0] the_vga_draw_frame_write_mem_data;
reg the_vga_draw_frame_write_a_pixel;

/* This is the frame driver point that you can write to the draw_frame */
vga_frame_driver my_frame_driver(
	.clk(clk),
	.rst(rst),

	.active_pixels(active_pixels),
	.frame_done(frame_done),

	.x(x),
	.y(y),

	.VGA_BLANK_N(VGA_BLANK_N),
	.VGA_CLK(VGA_CLK),
	.VGA_HS(VGA_HS),
	.VGA_SYNC_N(VGA_SYNC_N),
	.VGA_VS(VGA_VS),
	.VGA_B(VGA_B),
	.VGA_G(VGA_G),
	.VGA_R(VGA_R),

	/* writes to the frame buf - you need to figure out how x and y or other details provide a translation */
	.the_vga_draw_frame_write_mem_address(address),
	.the_vga_draw_frame_write_mem_data(data),
	.the_vga_draw_frame_write_a_pixel(writepixel)
);

reg [15:0]i;
reg [7:0]S;
reg [7:0]NS;
parameter 
	START 			= 8'd0,
	// W2M is write to memory
	W2M_INIT 		= 8'd1,
	W2M_COND 		= 8'd2,
	W2M_INC 			= 8'd3,
	W2M_DONE 		= 8'd4,
	// The RFM = READ_FROM_MEMOERY reading cycles
	RFM_INIT_START = 8'd5,
	RFM_INIT_WAIT 	= 8'd6,
	RFM_DRAWING 	= 8'd7,
	ERROR 			= 8'hFF;

parameter MEMORY_SIZE = 16'd19200; // 160*120 // Number of memory spots ... highly reduced since memory is slow
parameter PIXEL_VIRTUAL_SIZE = 16'd4; // Pixels per spot - therefore 4x4 pixels are drawn per memory location

/* ACTUAL VGA RESOLUTION */
parameter VGA_WIDTH = 16'd640; 
parameter VGA_HEIGHT = 16'd480;

/* Our reduced RESOLUTION 160 by 120 needs a memory of 19,200 words each 24 bits wide */
parameter VIRTUAL_PIXEL_WIDTH = VGA_WIDTH/PIXEL_VIRTUAL_SIZE; // 160
parameter VIRTUAL_PIXEL_HEIGHT = VGA_HEIGHT/PIXEL_VIRTUAL_SIZE; // 120

/* idx_location stores all the locations in the */
reg [14:0] idx_location;
/* !!!!!!!!!NOTE!!!!!!!
 - FLAG logic is a bad way to approach this, but I was lazy - I should implement this as an FSM for the button grabs.  */
reg flag1;
reg flag2;

// Just so I can see the address being calculated
assign LEDR = idx_location;

reg [9:0] square_x; // x position of the square
reg [14:0] square_start_addr; // start address of the square in memory
reg [4:0] square_row, square_col;

reg [3:0] current_state, next_state;

// State definitions
parameter 
    INIT            = 4'd0,
    FILL_WHITE      = 4'd1,
    DRAW_SQUARE     = 4'd2,
    DONE            = 4'd3;

// State Register
always @(posedge clk or negedge rst)
begin
    if (rst == 1'b0)
    begin
        current_state <= INIT;
    end
    else
    begin
        current_state <= next_state;
    end
end

// Next State Logic
always @(*)
begin
    case (current_state)
        INIT:
            next_state = FILL_WHITE;
        
        FILL_WHITE:
            if (square_done == 1'b1)
					next_state = DRAW_SQUARE;
				else 
					next_state = FILL_WHITE;
       
        
        DRAW_SQUARE:
            next_state = (idx_location == MEMORY_SIZE + (20 * 20) - 1) ? DONE : DRAW_SQUARE;
        
        DONE:
            next_state = DONE;
        
        default:
            next_state = INIT;
    endcase
end

reg [0:0]square_start;
wire [0:0]square_done;
reg [19:0]square_width;
reg [19:0]square_height;
reg [19:0]x_start;
reg [19:0]y_start;
reg [23:0]color;
wire [14:0]address;
wire [23:0]data;
wire [0:0]writepixel;
reg [19:0]pixel;
// Output Logic

square_drawer drawmysquare(clk, rst, square_height, square_width, square_start, pixel, color, address, data,
					 writepixel, square_done);



always @(posedge clk or negedge rst)
begin
    if (rst == 1'b0)
    begin
        the_vga_draw_frame_write_mem_address <= 15'd0;
        the_vga_draw_frame_write_mem_data <= 24'd0;
        the_vga_draw_frame_write_a_pixel <= 1'b0;
        idx_location <= 15'd0;
        square_start_addr <= 15'd0;
        square_row <= 5'd0;
        square_col <= 5'd0;
    end
    else
    begin
        case (current_state)
            INIT:
            begin
                the_vga_draw_frame_write_mem_address <= 15'd0;
                the_vga_draw_frame_write_mem_data <= 24'd0;
                the_vga_draw_frame_write_a_pixel <= 1'b0;
                idx_location <= 15'd0;
					 pixel <= (x_start - 1) * 120 + y_start;
            end
            
            FILL_WHITE:
            begin
                /*the_vga_draw_frame_write_mem_address <= idx_location;
                the_vga_draw_frame_write_mem_data <= 24'hFFFFFF; // White
                the_vga_draw_frame_write_a_pixel <= 1'b1;
                idx_location <= idx_location + 1'b1;*/
					 square_width <= 20'd160;
					 square_height <= 20'd120;
					 x_start <= 1'b0;
					 y_start <= 1'b0;
					 color <= 24'hFFFFFF;
					 square_start <= 1'b1;
				end 
            
          
            DRAW_SQUARE:
            begin
                // Calculate row and column within square
                square_width = (pixel - MEMORY_SIZE) / 20;
                square_height = (pixel - MEMORY_SIZE) % 20;
                
                // Draw the square
						  x_start <= 1'd0;
						  y_start <= 1'd0;
						  square_width <= 20'd20;
						  square_height <= 20'd20;
						  color <=24'd0;
						  square_start <= 1'b1;
                
                
                
            end
            
            DONE:
            begin
                the_vga_draw_frame_write_a_pixel <= 1'b0;
            end
        endcase
    end
end
endmodule

module square_drawer(
    input clk, 
    input rst, 
    input [19:0] height, 
    input [19:0] width, 
    input start, 
    input [19:0] pixel, 
    input [23:0] color, 
    output reg [14:0] address,  // Specify 15-bit width
    output reg [23:0] data,     // Specify 24-bit width
    output reg writepixel, 
    output reg done
);
	parameter START = 3'd0,
				OUTER_FOR = 3'd1,
				INNER_FOR = 3'd2,
				EXECUTE = 3'd3,
				J_UPDATE = 3'd4,
				I_UPDATE = 3'd5,
				DONE = 3'd6;

		reg [2:0]S;
		reg [2:0]NS;
		reg [20:0]i;
		reg [20:0]j;
		reg [19:0]updatedpixel;
		always@(posedge clk or negedge rst) begin
			if (rst == 1'b0)
				S <= START;
			else 
				S <= NS;
		end
	
		always@(*) begin
			case(S)
				START: if (start == 1'b1)
							NS = OUTER_FOR;
						else 
							NS = START;
				OUTER_FOR: if(i < width)
									NS = INNER_FOR;
								else
									NS = DONE;
				INNER_FOR: if (j < height)
									NS = EXECUTE;
							else 
									NS = I_UPDATE;
				EXECUTE: NS = J_UPDATE;
				J_UPDATE: NS = INNER_FOR;
				I_UPDATE: NS = OUTER_FOR;
				DONE: NS = DONE;
			endcase
		end
		always@(posedge clk or negedge rst) begin
		
			if (rst == 1'b0) begin
				i <= 1'b0;
				j <= 1'b0;
				updatedpixel <= pixel;
			end else begin
				case(S)
					START: begin
						i <= 1'b0;
						j <= 1'b0;
					end
					
					J_UPDATE: j <= j+1'b1;
					I_UPDATE: i <= i + 1'b1;
					EXECUTE: begin
						writepixel <= 1'b1;
						data <= color;
						address <= ((i *120) + j + updatedpixel);
					end
					DONE: 
						done <= 1'b1;
			 
			
			
				endcase
			end
		
		
		end
	

endmodule