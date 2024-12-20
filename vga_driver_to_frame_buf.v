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
	.the_vga_draw_frame_write_mem_address(the_vga_draw_frame_write_mem_address),
	.the_vga_draw_frame_write_mem_data(the_vga_draw_frame_write_mem_data),
	.the_vga_draw_frame_write_a_pixel(the_vga_draw_frame_write_a_pixel)
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
assign LEDR = current_state;

reg [9:0] square_x; // x position of the square
reg [14:0] square_start_addr; // start address of the square in memory
reg [4:0] square_row, square_col;

reg [6:0] current_state, next_state;

// State definitions
parameter 
    INIT            = 4'd0,
    FILL_WHITE      = 4'd1,
	 DRAW_WAIT       = 4'd2,
	 DRAW_RESET      = 4'd3,
    DRAW_SQUARE     = 4'd4,
    DONE            = 4'd5;

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
            if (idx_location >= MEMORY_SIZE)
                next_state = DRAW_WAIT;
            else 
                next_state = FILL_WHITE;
        DRAW_WAIT : next_state = DRAW_RESET;
        DRAW_RESET: 
            next_state = DRAW_SQUARE;
        
        DRAW_SQUARE:
            if (square_done)
                next_state = DONE;
            else 
                next_state = DRAW_SQUARE;
        
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
reg [8:0]x_start;
reg [8:0]y_start;
reg [23:0]color;
wire [14:0]address;
wire write_en;
// Output Logic


parameter SCREEN_WIDTH = 16'd160; // was set at 640
parameter SCREEN_HEIGHT = 16'd120; // was at 480
parameter DEFAULT_SQUARE_SIZE = 16'd50;

//square_drawer drawmysquare(clk, rst, square_height, square_width, square_start, pixel, color, address, data,
					 //writepixel, square_done);
					 
rectangle_drawer draw(clk, rst, square_start, square_width, square_height, x_start, y_start, color, address, write_en, square_done);

//assign the_vga_draw_fram_write_mem_address = address;
//assign the_vga_draw_fram_write_mem_data = data;
//assign the_vga_draw_fram_write_a_pixel = writepixel;

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
            end
            
            FILL_WHITE:
            begin
                // Fill entire screen with white
                the_vga_draw_frame_write_mem_address <= idx_location;
                the_vga_draw_frame_write_mem_data <= 24'hFFFFFF; // White
                the_vga_draw_frame_write_a_pixel <= 1'b1;
                idx_location <= idx_location + 1'b1;
            end 
            
				DRAW_WAIT:
				
            DRAW_RESET: 
				begin
					square_height <= 20'd5;    // Just 5 pixels tall
					square_width <= 20'd5;     // and 5 pixels wide
					x_start <= 8'd0;          // Start at the very top-left
					y_start <= 8'd0;
					square_start <= 1'b1;
				end

				DRAW_SQUARE: begin
					// Connect rectangle drawer outputs to frame buffer inputs
					the_vga_draw_frame_write_mem_address <= address;
					the_vga_draw_frame_write_mem_data <= color;  // Use the color input from rectangle_drawer
					the_vga_draw_frame_write_a_pixel <= write_en;
                
					if (square_done)
					begin
						square_start <= 1'b0;
					end
				end
            
            DONE:
            begin
                the_vga_draw_frame_write_a_pixel <= 1'b0;
                square_start <= 1'b0;
            end
        endcase
    end
end
endmodule

module rectangle_drawer(
    input clk,
    input rst,
    input start,
    input [7:0]width,      // Rectangle width
    input [7:0]height,     // Rectangle height
    input [7:0]start_x,    // Starting X position
    input [7:0]start_y,    // Starting Y position
    input [23:0]color,
	 output reg [14:0]address,
	 output reg write_en,// Rectangle color
    output reg done
);

    // State definitions
    localparam IDLE = 3'd0;
    localparam INIT = 3'd1;
    localparam SETUP_PIXEL = 3'd2;
    localparam WRITE_BUFFER = 3'd3;
    localparam FINISHED = 3'd4;

    // Screen parameters
    localparam SCREEN_WIDTH = 160;
    localparam SCREEN_HEIGHT = 120;
    
    // State and counter registers
    reg [2:0] current_state;
    reg [7:0] x_counter;
    reg [7:0] y_counter;

    // Bounds checking
    wire [7:0] end_x = start_x + width;
    wire [7:0] end_y = start_y + height;
    wire x_valid = (end_x <= SCREEN_WIDTH);
    wire y_valid = (end_y <= SCREEN_HEIGHT);

    // Next state logic
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            current_state <= IDLE;
            x_counter <= 0;
            y_counter <= 0;
            address <= 0;
            write_en <= 0;
            done <= 0;
        end
        else begin
            case (current_state)
                IDLE: begin
                    if (start && x_valid && y_valid) begin
                        current_state <= INIT;
                        done <= 0;
                    end
                    else if (start) begin
                        // If dimensions are invalid, signal done without drawing
                        done <= 1'b1;
                        current_state <= IDLE;
                    end
                end

                INIT: begin
                    x_counter <= 0;
                    y_counter <= 0;
                    write_en <= 0;
                    current_state <= SETUP_PIXEL;
                end

                SETUP_PIXEL: begin
                    // Setup the address and data for the next pixel
                    //address <= (start_y + y_counter) + ((start_x + x_counter) * SCREEN_HEIGHT);
                    address <= 200;
                    write_en <= 1'b1;
                    current_state <= WRITE_BUFFER;
                end

                WRITE_BUFFER: begin
                    // Wait one clock cycle for memory write
                    write_en <= 1'b0;
                    // Update counters and determine next state
                    if (x_counter < width - 1) begin
                        x_counter <= x_counter + 1;
                        current_state <= SETUP_PIXEL;
                    end
                    else begin
                        x_counter <= 0;
                        if (y_counter < height - 1) begin
                            y_counter <= y_counter + 1;
                            current_state <= SETUP_PIXEL;
                        end
                        else begin
                            current_state <= FINISHED;
                        end
                    end
                end

                FINISHED: begin
                    done <= 1'b1;
                    write_en <= 1'b0;
                    current_state <= IDLE;
                end

                default: current_state <= IDLE;
            endcase
        end
    end

endmodule



// Below are the previous attempts at the square_drawer modules
/*module drawer(input clk, input rst, input start, input color, input height, input width, input location, output reg drawer_done);
	reg [4:0]S;
	reg [4:0]NS;
	
	always@(posedge clk or negedge rst) begin
		if (rst == 1'b0) begin
			the_vga_draw_frame_write_a_pixel <= 1'b0;
			drawer_done <= 1'b0;
			S <= START;
		end else begin
			S <= NS;
		end
	end
	
	always@(posedge clk or negedge rst) begin
		
		case(S)
			f
	
	end



endmodule*/

/*module square_drawer(
    input clk, 
    input rst, 
    input [19:0] height, 
    input [19:0] width, 
    input start, 
    input [19:0] pixel, 
    input [23:0] color, 
    output reg [14:0] address,  
    output reg [23:0] data,     
    output reg writepixel, 
    output reg done
);
    // More robust state machine
    reg [2:0] state;
    parameter 
        IDLE = 3'd0, 
        START_DRAW = 3'd1, 
        DRAWING = 3'd2, 
        FINISHED = 3'd3;

    reg [20:0] x, y;
    reg [20:0] max_x, max_y;

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            // Reset everything
            state <= IDLE;
            x <= 0;
            y <= 0;
            max_x <= 0;
            max_y <= 0;
            done <= 0;
            writepixel <= 0;
        end
        else begin
            case (state)
                IDLE: begin
                    done <= 0;
                    writepixel <= 0;
                    if (start) begin
                        // Initialize drawing parameters
                        x <= 0;
                        y <= 0;
                        max_x <= width;
                        max_y <= height;
                        state <= START_DRAW;
                    end
                end

                START_DRAW: begin
                    state <= DRAWING;
                end

                DRAWING: begin
                    // Draw current pixel
                    writepixel <= 1'b1;
                    data <= color;
                    address <= (y * 120) + x + pixel;

                    // Increment coordinates
                    if (x < max_x - 1) begin
                        x <= x + 1;
                    end
                    else begin
                        x <= 0;
                        if (y < max_y - 1)
                            y <= y + 1;
                        else begin
                            state <= FINISHED;
                            writepixel <= 1'b0;
                        end
                    end
                end

                FINISHED: begin
                    done <= 1'b1;
                    state <= IDLE;
                end
            endcase
        end
    end
endmodule*/