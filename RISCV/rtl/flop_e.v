/****************************************************************************
 * flop_e.v
 ****************************************************************************/

/**
 * Module: flop_e
 * 
 * TODO: Add module documentation
 */
module flop_e
		#(parameter Bits=1)
		(
		input clk,
		input reset,
		input we,
		input [Bits-1:0] d,
		output reg [Bits-1:0] q
		);

always @(posedge clk)begin
	if (reset)
		begin
			q <= 0;
		end
	else
		begin
			if(we)
				q <= d;
		end
end

endmodule


