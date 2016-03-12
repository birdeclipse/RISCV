/* verilator lint_off UNUSED */
module pending_check(
	
	input		clk,
	input		reset,
	
	input		instr_is_load,
	input [4:0] decode_dest_sel_next,
	
	
	input [4:0]	decode_rs1_sel,
	input		decode_rs1_sel_valid,
	
	input [4:0]	decode_rs2_sel,
	input		decode_rs2_sel_valid,
	
	output reg	src1_is_pending,
	output reg	src2_is_pending,

  // dcache interface from testbench to decode
	input [4:0]	dcache_ack_rd,  
	input		dcache_ack_valid
);

/* verilator lint_off UNUSED */

reg	[31:0]	rf_pending;
reg	[31:0]	rf_pending_shadow;
reg	[31:0]	rf_pending_shadow_next0;
reg	[31:0]	rf_pending_shadow_next1;

reg	[1:0]	rf_pending_ckeck_src1;
reg	[1:0]	rf_pending_ckeck_src2;


always@(*)begin

	if(dcache_ack_valid)
		begin
			case(dcache_ack_rd)
				5'd0:	rf_pending_shadow_next0	=	{rf_pending_shadow[31:1],1'b0};
				5'd1:	rf_pending_shadow_next0	=	{rf_pending_shadow[31:2],1'b0,rf_pending_shadow[0]};
				5'd2:	rf_pending_shadow_next0	=	{rf_pending_shadow[31:3],1'b0,rf_pending_shadow[1:0]};
				5'd3:	rf_pending_shadow_next0	=	{rf_pending_shadow[31:4],1'b0,rf_pending_shadow[2:0]};
				5'd4:	rf_pending_shadow_next0	=	{rf_pending_shadow[31:5],1'b0,rf_pending_shadow[3:0]};
				5'd5:	rf_pending_shadow_next0	=	{rf_pending_shadow[31:6],1'b0,rf_pending_shadow[4:0]};
				5'd6:	rf_pending_shadow_next0	=	{rf_pending_shadow[31:7],1'b0,rf_pending_shadow[5:0]};
				5'd7:	rf_pending_shadow_next0	=	{rf_pending_shadow[31:8],1'b0,rf_pending_shadow[6:0]};
				5'd8:	rf_pending_shadow_next0	=	{rf_pending_shadow[31:9],1'b0,rf_pending_shadow[7:0]};
				5'd9:	rf_pending_shadow_next0	=	{rf_pending_shadow[31:10],1'b0,rf_pending_shadow[8:0]};
				5'd10:	rf_pending_shadow_next0	=	{rf_pending_shadow[31:11],1'b0,rf_pending_shadow[9:0]};
				5'd11:	rf_pending_shadow_next0	=	{rf_pending_shadow[31:12],1'b0,rf_pending_shadow[10:0]};
				5'd12:	rf_pending_shadow_next0	=	{rf_pending_shadow[31:13],1'b0,rf_pending_shadow[11:0]};
				5'd13:	rf_pending_shadow_next0	=	{rf_pending_shadow[31:14],1'b0,rf_pending_shadow[12:0]};
				5'd14:	rf_pending_shadow_next0	=	{rf_pending_shadow[31:15],1'b0,rf_pending_shadow[13:0]};
				5'd15:	rf_pending_shadow_next0	=	{rf_pending_shadow[31:16],1'b0,rf_pending_shadow[14:0]};
				5'd16:	rf_pending_shadow_next0	=	{rf_pending_shadow[31:17],1'b0,rf_pending_shadow[15:0]};
				5'd17:	rf_pending_shadow_next0	=	{rf_pending_shadow[31:18],1'b0,rf_pending_shadow[16:0]};
				5'd18:	rf_pending_shadow_next0	=	{rf_pending_shadow[31:19],1'b0,rf_pending_shadow[17:0]};
				5'd19:	rf_pending_shadow_next0	=	{rf_pending_shadow[31:20],1'b0,rf_pending_shadow[18:0]};
				5'd20:  rf_pending_shadow_next0	=	{rf_pending_shadow[31:21],1'b0,rf_pending_shadow[19:0]};
				5'd21:  rf_pending_shadow_next0	=	{rf_pending_shadow[31:22],1'b0,rf_pending_shadow[20:0]};
				5'd22:  rf_pending_shadow_next0	=	{rf_pending_shadow[31:23],1'b0,rf_pending_shadow[21:0]};
				5'd23:  rf_pending_shadow_next0	=	{rf_pending_shadow[31:24],1'b0,rf_pending_shadow[22:0]};
				5'd24:  rf_pending_shadow_next0	=	{rf_pending_shadow[31:25],1'b0,rf_pending_shadow[23:0]};
				5'd25:  rf_pending_shadow_next0	=	{rf_pending_shadow[31:26],1'b0,rf_pending_shadow[24:0]};
				5'd26:  rf_pending_shadow_next0	=	{rf_pending_shadow[31:27],1'b0,rf_pending_shadow[25:0]};
				5'd27:  rf_pending_shadow_next0	=	{rf_pending_shadow[31:28],1'b0,rf_pending_shadow[26:0]};
				5'd28:  rf_pending_shadow_next0	=	{rf_pending_shadow[31:29],1'b0,rf_pending_shadow[27:0]};
				5'd29:  rf_pending_shadow_next0	=	{rf_pending_shadow[31:30],1'b0,rf_pending_shadow[28:0]};
				5'd30:  rf_pending_shadow_next0	=	{rf_pending_shadow[31],1'b0,rf_pending_shadow[29:0]};
				5'd31:	rf_pending_shadow_next0	=	{1'b0,rf_pending_shadow[30:0]};	
			endcase	
		end
	else
		begin
			rf_pending_shadow_next0	=	rf_pending_shadow;
		end
end








always@(*)begin

	if(instr_is_load)
		begin
			case(decode_dest_sel_next)
				5'd0:	rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:1],1'b1};
				5'd1:	rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:2],1'b1, rf_pending_shadow_next0[0]};
				5'd2:	rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:3],1'b1, rf_pending_shadow_next0[1:0]};
				5'd3:	rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:4],1'b1, rf_pending_shadow_next0[2:0]};
				5'd4:	rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:5],1'b1, rf_pending_shadow_next0[3:0]};
				5'd5:	rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:6],1'b1, rf_pending_shadow_next0[4:0]};
				5'd6:	rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:7],1'b1, rf_pending_shadow_next0[5:0]};
				5'd7:	rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:8],1'b1, rf_pending_shadow_next0[6:0]};
				5'd8:	rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:9],1'b1, rf_pending_shadow_next0[7:0]};
				5'd9:	rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:10],1'b1,rf_pending_shadow_next0[8:0]};
				5'd10:	rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:11],1'b1,rf_pending_shadow_next0[9:0]};
				5'd11:	rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:12],1'b1,rf_pending_shadow_next0[10:0]};
				5'd12:	rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:13],1'b1,rf_pending_shadow_next0[11:0]};
				5'd13:	rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:14],1'b1,rf_pending_shadow_next0[12:0]};
				5'd14:	rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:15],1'b1,rf_pending_shadow_next0[13:0]};
				5'd15:	rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:16],1'b1,rf_pending_shadow_next0[14:0]};
				5'd16:	rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:17],1'b1,rf_pending_shadow_next0[15:0]};
				5'd17:	rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:18],1'b1,rf_pending_shadow_next0[16:0]};
				5'd18:	rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:19],1'b1,rf_pending_shadow_next0[17:0]};
				5'd19:	rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:20],1'b1,rf_pending_shadow_next0[18:0]};
				5'd20:  rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:21],1'b1,rf_pending_shadow_next0[19:0]};
				5'd21:  rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:22],1'b1,rf_pending_shadow_next0[20:0]};
				5'd22:  rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:23],1'b1,rf_pending_shadow_next0[21:0]};
				5'd23:  rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:24],1'b1,rf_pending_shadow_next0[22:0]};
				5'd24:  rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:25],1'b1,rf_pending_shadow_next0[23:0]};
				5'd25:  rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:26],1'b1,rf_pending_shadow_next0[24:0]};
				5'd26:  rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:27],1'b1,rf_pending_shadow_next0[25:0]};
				5'd27:  rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:28],1'b1,rf_pending_shadow_next0[26:0]};
				5'd28:  rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:29],1'b1,rf_pending_shadow_next0[27:0]};
				5'd29:  rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31:30],1'b1,rf_pending_shadow_next0[28:0]};
				5'd30:  rf_pending_shadow_next1	=	{rf_pending_shadow_next0[31],1'b1,rf_pending_shadow_next0[29:0]};
				5'd31:	rf_pending_shadow_next1	=	{1'b1,rf_pending_shadow_next0[30:0]};	
			endcase	
		end
	else
		begin
			rf_pending_shadow_next1	=	rf_pending_shadow_next0;
		end
end




always@(posedge clk)begin
	if(reset)
		begin
			rf_pending_shadow	<=	32'b0;
		end
	else
		begin
			rf_pending_shadow	<=	rf_pending_shadow_next1;
		end
end

always@(posedge clk)begin
	if(reset)
		begin
			rf_pending			<=	32'b0;
		end
	else
		begin
			rf_pending			<=	rf_pending_shadow;
		end
end









always@(*)begin
	case(decode_rs1_sel)
	5'd0:	rf_pending_ckeck_src1	=	 {rf_pending[0], rf_pending_shadow[0]};
	5'd1:	rf_pending_ckeck_src1	=	 {rf_pending[1], rf_pending_shadow[1]};
	5'd2:	rf_pending_ckeck_src1   =    {rf_pending[2], rf_pending_shadow[2]};
	5'd3:   rf_pending_ckeck_src1   =    {rf_pending[3], rf_pending_shadow[3]};
	5'd4:   rf_pending_ckeck_src1   =    {rf_pending[4], rf_pending_shadow[4]};
	5'd5:   rf_pending_ckeck_src1   =    {rf_pending[5], rf_pending_shadow[5]};
	5'd6:   rf_pending_ckeck_src1   =    {rf_pending[6], rf_pending_shadow[6]};
	5'd7:   rf_pending_ckeck_src1   =    {rf_pending[7], rf_pending_shadow[7]};
    5'd8:   rf_pending_ckeck_src1   =    {rf_pending[8], rf_pending_shadow[8]};
    5'd9:   rf_pending_ckeck_src1   =    {rf_pending[9], rf_pending_shadow[9]};
    5'd10:  rf_pending_ckeck_src1   =    {rf_pending[10],rf_pending_shadow[10]};
    5'd11:  rf_pending_ckeck_src1   =    {rf_pending[11],rf_pending_shadow[11]};
    5'd12:  rf_pending_ckeck_src1   =    {rf_pending[12],rf_pending_shadow[12]};
    5'd13:  rf_pending_ckeck_src1   =    {rf_pending[13],rf_pending_shadow[13]};
    5'd14:  rf_pending_ckeck_src1   =    {rf_pending[14],rf_pending_shadow[14]};
    5'd15:  rf_pending_ckeck_src1   =    {rf_pending[15],rf_pending_shadow[15]};
    5'd16:  rf_pending_ckeck_src1   =    {rf_pending[16],rf_pending_shadow[16]};
    5'd17:  rf_pending_ckeck_src1   =    {rf_pending[17],rf_pending_shadow[17]};
    5'd18:  rf_pending_ckeck_src1   =    {rf_pending[18],rf_pending_shadow[18]};
    5'd19:  rf_pending_ckeck_src1   =    {rf_pending[19],rf_pending_shadow[19]};
    5'd20:  rf_pending_ckeck_src1   =    {rf_pending[20],rf_pending_shadow[20]};
    5'd21:  rf_pending_ckeck_src1   =    {rf_pending[21],rf_pending_shadow[21]};
    5'd22:  rf_pending_ckeck_src1   =    {rf_pending[22],rf_pending_shadow[22]};
    5'd23:  rf_pending_ckeck_src1   =    {rf_pending[23],rf_pending_shadow[23]};
    5'd24:  rf_pending_ckeck_src1   =    {rf_pending[24],rf_pending_shadow[24]};
    5'd25:  rf_pending_ckeck_src1   =    {rf_pending[25],rf_pending_shadow[25]};
    5'd26:  rf_pending_ckeck_src1   =    {rf_pending[26],rf_pending_shadow[26]};
    5'd27:  rf_pending_ckeck_src1   =    {rf_pending[27],rf_pending_shadow[27]};
    5'd28:  rf_pending_ckeck_src1   =    {rf_pending[28],rf_pending_shadow[28]};
    5'd29:  rf_pending_ckeck_src1   =    {rf_pending[29],rf_pending_shadow[29]};
	5'd30:  rf_pending_ckeck_src1   =    {rf_pending[30],rf_pending_shadow[30]};
    5'd31:  rf_pending_ckeck_src1   =    {rf_pending[31],rf_pending_shadow[31]};
	endcase

end


always@(*)begin
	case(decode_rs2_sel)
	5'd0:	rf_pending_ckeck_src2	=	 {rf_pending[0], rf_pending_shadow[0]};
	5'd1:	rf_pending_ckeck_src2	=	 {rf_pending[1], rf_pending_shadow[1]};
	5'd2:	rf_pending_ckeck_src2   =    {rf_pending[2], rf_pending_shadow[2]};
	5'd3:   rf_pending_ckeck_src2   =    {rf_pending[3], rf_pending_shadow[3]};
	5'd4:   rf_pending_ckeck_src2   =    {rf_pending[4], rf_pending_shadow[4]};
	5'd5:   rf_pending_ckeck_src2   =    {rf_pending[5], rf_pending_shadow[5]};
	5'd6:   rf_pending_ckeck_src2   =    {rf_pending[6], rf_pending_shadow[6]};
	5'd7:   rf_pending_ckeck_src2   =    {rf_pending[7], rf_pending_shadow[7]};
    5'd8:   rf_pending_ckeck_src2   =    {rf_pending[8], rf_pending_shadow[8]};
    5'd9:   rf_pending_ckeck_src2   =    {rf_pending[9], rf_pending_shadow[9]};
    5'd10:  rf_pending_ckeck_src2   =    {rf_pending[10],rf_pending_shadow[10]};
    5'd11:  rf_pending_ckeck_src2   =    {rf_pending[11],rf_pending_shadow[11]};
    5'd12:  rf_pending_ckeck_src2   =    {rf_pending[12],rf_pending_shadow[12]};
    5'd13:  rf_pending_ckeck_src2   =    {rf_pending[13],rf_pending_shadow[13]};
    5'd14:  rf_pending_ckeck_src2   =    {rf_pending[14],rf_pending_shadow[14]};
    5'd15:  rf_pending_ckeck_src2   =    {rf_pending[15],rf_pending_shadow[15]};
    5'd16:  rf_pending_ckeck_src2   =    {rf_pending[16],rf_pending_shadow[16]};
    5'd17:  rf_pending_ckeck_src2   =    {rf_pending[17],rf_pending_shadow[17]};
    5'd18:  rf_pending_ckeck_src2   =    {rf_pending[18],rf_pending_shadow[18]};
    5'd19:  rf_pending_ckeck_src2   =    {rf_pending[19],rf_pending_shadow[19]};
    5'd20:  rf_pending_ckeck_src2   =    {rf_pending[20],rf_pending_shadow[20]};
    5'd21:  rf_pending_ckeck_src2   =    {rf_pending[21],rf_pending_shadow[21]};
    5'd22:  rf_pending_ckeck_src2   =    {rf_pending[22],rf_pending_shadow[22]};
    5'd23:  rf_pending_ckeck_src2   =    {rf_pending[23],rf_pending_shadow[23]};
    5'd24:  rf_pending_ckeck_src2   =    {rf_pending[24],rf_pending_shadow[24]};
    5'd25:  rf_pending_ckeck_src2   =    {rf_pending[25],rf_pending_shadow[25]};
    5'd26:  rf_pending_ckeck_src2   =    {rf_pending[26],rf_pending_shadow[26]};
    5'd27:  rf_pending_ckeck_src2   =    {rf_pending[27],rf_pending_shadow[27]};
    5'd28:  rf_pending_ckeck_src2   =    {rf_pending[28],rf_pending_shadow[28]};
    5'd29:  rf_pending_ckeck_src2   =    {rf_pending[29],rf_pending_shadow[29]};
	5'd30:  rf_pending_ckeck_src2   =    {rf_pending[30],rf_pending_shadow[30]};
    5'd31:  rf_pending_ckeck_src2   =    {rf_pending[31],rf_pending_shadow[31]};
	endcase

end



always@(*)begin

	if(decode_rs1_sel_valid)
		begin
			case(rf_pending_ckeck_src1)
			2'b00:	 src1_is_pending		= 1'b0;
			
			2'b01:	 src1_is_pending		= 1'b1;
			
			2'b10:	 src1_is_pending		= 1'b0;
			
			2'b11:	 src1_is_pending		= 1'b1;
			endcase
		end
	else
		begin
			src1_is_pending	= 1'b0;
		end

end



always@(*)begin

	if(decode_rs2_sel_valid)
		begin
			case(rf_pending_ckeck_src2)
			2'b00:	 src2_is_pending	= 1'b0;
			
			2'b01:	 src2_is_pending	= 1'b1;
			
			2'b10:	 src2_is_pending	= 1'b0;
			
			2'b11:	 src2_is_pending	= 1'b1;
			endcase
		end
	else
		begin
			src2_is_pending	= 1'b0;
		end

end



endmodule