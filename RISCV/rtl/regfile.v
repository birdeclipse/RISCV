/****************************************************************************
 * regfile.v
 ****************************************************************************/

/**
 * Module: regfile
 * 
 * TODO: Add module documentation
 */
/* verilator lint_off UNUSED */
module regfile(
	input 				clk,
	input 				reset,

	input  [4:0] 		rs1_sel,
	input  [4:0] 		rs2_sel,
		
	/*mem interface */
	input  [63:0] 		dcache_ack_data,
	input  [4:0]  		dcache_ack_rd,  // destination register for the load
	input         		dcache_ack_valid,
	output        		dcache_ack_retry, // ALWAYS_FALSE
	
	
	input  [4:0] 		rd_sel,
	input				dest_enable,
	input				dest_long_enable,
	input  [63:0]		dest,
	
	
	output [63:0] 		rs1_data,
	output [63:0] 		rs2_data

);

reg	[63:0]	rf[31:0];
reg	[4:0]	reset_counter	=	5'b0;


reg	[63:0]	rs1_data_next;
reg	[63:0]	rs2_data_next;



assign		dcache_ack_retry	=	1'b0;


always@(posedge clk)begin
	if(reset)
		begin
			rf[reset_counter]	<=	64'b0;
			reset_counter		<=	reset_counter+1'b1;
		end
	else
		begin
			if(dcache_ack_valid)
				begin
					rf[dcache_ack_rd]	<=	dcache_ack_data;
				end
			if(dest_enable)
				begin
					if(dest_long_enable)
						rf[rd_sel]	<=	dest;
					else
						rf[rd_sel][31:0]	<=	dest[31:0];
				end

		end
end





always@(*)begin

		begin
			rs1_data_next	=	rf[rs1_sel];
		end
end

always@(*)begin
		begin
			rs2_data_next	=	rf[rs2_sel];
		end
end

assign	rs1_data	=	rs1_data_next;
assign	rs2_data	=	rs2_data_next;


endmodule