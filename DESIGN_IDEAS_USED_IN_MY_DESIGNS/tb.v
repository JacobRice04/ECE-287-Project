`timescale 1 ps / 1 ps

module tb;
	parameter simdelay = 20;
	parameter clock_delay = 5;

reg clk, rst;
reg en;
wire done;
wire SS, SCLK, MISO;
SPI_parent DUT(clk, rst, en, SS, SCLK, MISO, done);

		
	initial
	begin
		
		#(simdelay) clk = 1'b0; rst = 1'b0;
		#(simdelay) rst = 1'b1;
		
		#(simdelay) en = 1'b1; 
						
		#100; // let simulation finish
	
	end

/* this checks done every clock and when it goes high ends the simulation */
	always @(clk)
	begin
		if (done == 1'b1)
		begin
			$write("TRANMISSION_COMPLETE:"); 
			$stop;
		end
	end
	
	// this generates a clock
	always
	begin
		#(clock_delay) clk = !clk;
	end
endmodule
