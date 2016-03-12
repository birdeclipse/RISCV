/****************************************************************************
 * flop_e.v
 ****************************************************************************/

/**
 * Module: flop_e
 * 
 * TODO: Add module documentation
 */
module sram#(
	parameter RAM_WIDTH	=1,
	parameter RAM_DEPTH =6,
	parameter ADDR_WIDTH = 64
	)
	(
	input clk,
	input reset,
	input we,
	input [RAM_DEPTH-1:0] addr,
	input [RAM_WIDTH-1:0] d,
	output[RAM_WIDTH-1:0] q
	);

reg [RAM_WIDTH-1:0] sram [ADDR_WIDTH-1:0];
reg [RAM_WIDTH-1:0] sram_out = {RAM_WIDTH{1'b0}};		
reg	[RAM_DEPTH-1:0] reset_counter = 'd0;		
		
		
always @(posedge clk)begin
	if(reset)
		begin
			sram[reset_counter] <= 'd0;
			reset_counter	<= reset_counter+1;
		end
	else
		begin
			if(we)
				sram[addr] <= d;
		end
end


always@(*)begin
	sram_out	=	sram[addr];
end


assign	q = sram_out;



endmodule
