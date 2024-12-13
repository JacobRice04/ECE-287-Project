module vga_driver_to_frame_buf	(

	//////////// CLOCK //////////

	input 		          		CLOCK_50,



	//////////// SEG7 //////////
	output		     [6:0]		HEX0,
	output		     [6:0]		HEX1,
	output		     [6:0]		HEX2,
	output		     [6:0]		HEX3,
	//output		     [6:0]		HEX4,
	//output		     [6:0]		HEX5,


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

reg square_start;
wire [0:0]square_done;
wire [14:0]address;
wire write_en;


parameter SCREEN_WIDTH = 16'd160; // was set at 640
parameter SCREEN_HEIGHT = 16'd120; // was at 480
parameter DEFAULT_SQUARE_SIZE = 16'd50;

					 
rectangle_drawer draw(clk, rst, square_start, address, write_en, square_done);

always @(posedge clk or negedge rst)
begin
    if (rst == 1'b0)
    begin
        the_vga_draw_frame_write_mem_address <= 15'd0;
        the_vga_draw_frame_write_mem_data <= 24'd0;
        the_vga_draw_frame_write_a_pixel <= 1'b0;
        idx_location <= 15'd0;

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
            
				DRAW_WAIT: begin
					 the_vga_draw_frame_write_a_pixel <= 1'b0;
                square_start <= 1'b0;
				end
            DRAW_RESET: 
				begin
			
					square_start <= 1'b1;
				end

				DRAW_SQUARE: begin
					// Connect rectangle drawer outputs to frame buffer inputs
					the_vga_draw_frame_write_mem_address <= address;
					the_vga_draw_frame_write_mem_data <= 24'h000000;  // Use the color input from rectangle_drawer
					the_vga_draw_frame_write_a_pixel <= write_en;
					square_start <= 1'b0;
					
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
    output reg [14:0] address,
    output reg write_en,
    output reg done
);
    
    localparam IDLE = 3'd0;
    localparam INIT = 3'd1;
    localparam SETUP_PIXEL = 3'd2;
    localparam WRITE_PIXEL = 3'd3;
    localparam WAIT_WRITE = 3'd4;
    localparam FINISHED = 3'd5;

    reg [2:0] current_state, next_state;
    reg [3:0] wait_counter;  // Add a wait counter

    // State register
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            current_state <= IDLE;
            wait_counter <= 0;
        end else begin
            current_state <= next_state;
            if (current_state == WAIT_WRITE) begin
                wait_counter <= wait_counter + 1;
            end else begin
                wait_counter <= 0;
            end
        end
    end

    // Next state logic
    always @(*) begin
        case (current_state)
            IDLE: begin
                if (start)
                    next_state = INIT;
                else
                    next_state = IDLE;
            end
            
            INIT: begin
                next_state = SETUP_PIXEL;
            end
            
            SETUP_PIXEL: begin
                next_state = WRITE_PIXEL;
            end
            
            WRITE_PIXEL: begin
                next_state = WAIT_WRITE;
            end

            WAIT_WRITE: begin
                if (wait_counter >= 4'd10)  // Wait for 10 clock cycles
                    next_state = FINISHED;
                else
                    next_state = WAIT_WRITE;
            end
            
            FINISHED: begin
                next_state = IDLE;
            end
            
            default: next_state = IDLE;
        endcase
    end

    // Output logic
    always @(posedge clk or negedge rst) begin
        if (rst == 1'b0) begin
            write_en <= 0;
            done <= 0;
            address <= 0;
        end else begin
            case (current_state)
                IDLE: begin
                    write_en <= 0;
                    done <= 0;
                end

                INIT: begin
                    write_en <= 0;
                    done <= 0;
                end

                SETUP_PIXEL: begin
                    // Calculate center pixel address
                    // For 160x120 screen, center is at (80,60)
                    // Address = y * width + x = 60 * 160 + 80 = 9680
                    address <= 15'd9680;  // Center of screen
                    write_en <= 0;
                end

                WRITE_PIXEL: begin
                    write_en <= 1;  // Enable writing
                end

                WAIT_WRITE: begin
                    write_en <= 1;  // Keep write enabled during wait
                end

                FINISHED: begin
                    done <= 1;
                    write_en <= 0;
                end
            endcase
        end
    end

endmodule


/* module rectangle_drawer(
    input clk,
    input rst,
    input start,
    output reg [14:0] address,
    output reg write_en,
    output reg done
);
    
    localparam IDLE = 3'd0;
    localparam INIT = 3'd1;
    localparam SETUP_PIXEL = 3'd2;
    localparam WRITE_BUFFER = 3'd3;
    localparam FINISHED = 3'd4;

    reg [2:0] current_state, next_state;

    // State register
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            current_state <= IDLE;
        end else begin
            current_state <= next_state;
        end
    end

    // Next state logic
    always @(*) begin
        case (current_state)
            IDLE: begin
                if (start)
                    next_state = INIT;
                else
                    next_state = IDLE;
            end
            
            INIT: begin
                next_state = SETUP_PIXEL;
            end
            
            SETUP_PIXEL: begin
                next_state = WRITE_BUFFER;
            end
            
            WRITE_BUFFER: begin
                next_state = FINISHED;
            end
            
            FINISHED: begin
                next_state = IDLE;
            end
            
            default: 
					if (start)  // Only reset done when new start pulse comes
						next_state = IDLE;
					else
						next_state = FINISHED;
        endcase
    end

    // Output logic
    always @(posedge clk or negedge rst) begin
        if (rst == 1'b0) begin
            write_en <= 0;
            done <= 0;
            address <= 0;
        end else begin
            case (current_state)
                IDLE: begin
                    write_en <= 0;
                    done <= 0;
                end

                INIT: begin
                    write_en <= 0;
                    done <= 0;
                end

                SETUP_PIXEL: begin
                    // Calculate center pixel address
                    // For 160x120 screen, center is at (80,60)
                    // Address = y * width + x = 60 * 160 + 80 = 9680
                    address <= 15'd0;
                    write_en <= 1;
                end

                WRITE_BUFFER: begin
                    write_en <= 0;
                end

                FINISHED: begin
                    done <= 1;
                    write_en <= 0;
                end
            endcase
        end
    end

endmodule
*/
