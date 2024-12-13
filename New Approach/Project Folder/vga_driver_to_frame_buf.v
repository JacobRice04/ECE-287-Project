module vga_driver_to_frame_buf	(

	//////////// CLOCK //////////

	input 		          		CLOCK_50,

	//////////// SEG7 //////////
	output		     [6:0]		HEX0,
	output		     [6:0]		HEX1,
	output		     [6:0]		HEX2,
	output		     [6:0]		HEX3,
	output		     [6:0]		HEX4,
	output		     [6:0]		HEX5,


	//////////// KEY //////////
	input 		     [3:0]		KEY,

	//////////// LED //////////
	output		     [9:0]		LEDR,

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

);

// Turn off all displays.
//assign	HEX0		=	7'h00;
//assign	HEX1		=	7'h00;
//assign	HEX2		=	7'h00;
//assign	HEX3		=	7'h00;


three_decimal_vals_w_neg ship_locator(
.val(ship),
.seg7_neg_sign(HEX5),
.seg7_dig0(HEX0),
.seg7_dig1(HEX1),
.seg7_dig2(HEX2),
.seg7_dig3(HEX3),
.seg7_dig4(HEX4)
);


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
reg [14:0] ship;
/* !!!!!!!!!NOTE!!!!!!!
 - FLAG logic is a bad way to approach this, but I was lazy - I should implement this as an FSM for the button grabs.  */
reg flag1;
reg flag2;

// Just so I can see the address being calculated
assign LEDR = current_state;

reg [6:0] current_state, next_state;

// State definitions
parameter 
        INIT        = 4'd0,
        FILL_WHITE  = 4'd1,
        DRAW_WAIT   = 4'd2,
        DRAW_PIXEL  = 4'd3,
        DONE        = 4'd4;


    reg pixel_done;

    // State Register
    always @(posedge clk or negedge rst) begin
        if (rst == 1'b0) begin
            current_state <= INIT;
        end
        else begin
            current_state <= next_state;
        end
    end

    // Next State Logic
    always @(*) begin
        case (current_state)
            INIT:        next_state = FILL_WHITE;
            FILL_WHITE:  next_state = (ship >= MEMORY_SIZE) ? DRAW_WAIT : FILL_WHITE;
            DRAW_WAIT:   next_state = DRAW_PIXEL;
            DRAW_PIXEL:  next_state = pixel_done ? DONE : DRAW_PIXEL;
            DONE:        next_state = DONE;
            default:     next_state = INIT;
        endcase
    end

    // Output Logic
    always @(posedge clk or negedge rst) begin
        if (rst == 1'b0) begin
            the_vga_draw_frame_write_mem_address <= 15'd0;
            the_vga_draw_frame_write_mem_data <= 24'd0;
            the_vga_draw_frame_write_a_pixel <= 1'b0;
            ship <= 15'd7500;  // Center position
            pixel_done <= 1'b0;
        end
        else begin
            case (current_state)
                INIT: begin
                    the_vga_draw_frame_write_mem_address <= 15'd0;
                    the_vga_draw_frame_write_mem_data <= 24'd0;
                    the_vga_draw_frame_write_a_pixel <= 1'b0;
                    ship <= 15'd7500;  // Center position
                    pixel_done <= 1'b0;
                end

                FILL_WHITE: begin
                    // Fill screen with white
                    the_vga_draw_frame_write_mem_address <= ship;
                    the_vga_draw_frame_write_mem_data <= 24'hFFFFFF;  // White
                    the_vga_draw_frame_write_a_pixel <= 1'b1;
                    ship <= ship + 1'b1;
                end

                DRAW_WAIT: begin
                    the_vga_draw_frame_write_a_pixel <= 1'b0;
                    pixel_done <= 1'b0;
                end

                DRAW_PIXEL: begin
                    // Draw single black pixel at center position
                    the_vga_draw_frame_write_mem_address <= 15'd7500;  // Center position
                    the_vga_draw_frame_write_mem_data <= 24'h000000;  // Black
                    the_vga_draw_frame_write_a_pixel <= 1'b1;
                    pixel_done <= 1'b1;
                end

                DONE: begin
                    the_vga_draw_frame_write_a_pixel <= 1'b0;
                end
            endcase
        end
    end

endmodule
/*
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
            if (ship >= MEMORY_SIZE)
                next_state = DRAW_WAIT;
            else 
                next_state = FILL_WHITE;
        DRAW_WAIT : next_state = DRAW_SQUARE;
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
reg square_done;
wire [14:0]address;
wire write_en;

					 
//rectangle_drawer draw(clk, rst, square_start, address, write_en, square_done);

always @(posedge clk or negedge rst)
begin
    if (rst == 1'b0)
    begin
        the_vga_draw_frame_write_mem_address <= 15'd0;
        the_vga_draw_frame_write_mem_data <= 24'd0;
        the_vga_draw_frame_write_a_pixel <= 1'b0;
        ship <= 15'd7500; // draw pixel in center
		  square_done <= 1'b0;

    end
    else
    begin
        case (current_state) 
            INIT:
            begin
                the_vga_draw_frame_write_mem_address <= 15'd0;
                the_vga_draw_frame_write_mem_data <= 24'd0;
                the_vga_draw_frame_write_a_pixel <= 1'b0;
                ship <= 15'd7500;
            end
            
            FILL_WHITE:
            begin
                // Fill entire screen with white
                the_vga_draw_frame_write_mem_address <= ship;
                the_vga_draw_frame_write_mem_data <= 24'hFFFFFF; // White
                the_vga_draw_frame_write_a_pixel <= 1'b1;
                ship <= ship + 1'b1;
            end 
            
				DRAW_WAIT: begin
					 the_vga_draw_frame_write_a_pixel <= 1'b0;
					 square_done <= 1'b0;
				end
				DRAW_SQUARE: begin
					// Connect rectangle drawer outputs to frame buffer inputs
					the_vga_draw_frame_write_mem_address <= ship;
					the_vga_draw_frame_write_mem_data <= 24'h000000;  // black color
					the_vga_draw_frame_write_a_pixel <= 1'b1;
					square_done <= 1'b1;
					
				end
            
            DONE:
            begin
                the_vga_draw_frame_write_a_pixel <= 1'b0;
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
            address <= 15'd0;
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
                    address <= 15'd0;  // Center of screen
                    write_en <= 0;
                end

                WRITE_PIXEL: begin
             
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
*/


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
