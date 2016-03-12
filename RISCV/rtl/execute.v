/****************************************************************************
 * execute.v
 ****************************************************************************/

/**
 * Module: execute
 * 
 * TODO: Add module documentation
 */
/* verilator lint_off UNUSED */
module execute(
	
	input				clk,
	input				reset,

	
	input				execute_ack_data_valid,/*execute_ack_data_valid <- decode_ack_data_valid*/
	output				execute_ack_data_rety,


	input	[4:0]		exe_rs1_sel,
	input	[4:0]		exe_rs2_sel,
	
	
	
	
	input	[4:0]		decode_dest_sel,
	input				imm_rs2_sel,
	input				comp_is_unsigned,
	input	[63:0]		sign_ex_imm,
	input	[5:0]		shift_amount,	
	input	[19:0]		U_imm,
	input	[19:0]		UJ_imm,
	input	[11:0]		S_imm,
	input	[11:0]		SB_imm,
	
	input	[6:0]		op_code,
	input	[2:0]		funct3,
	input	[6:0]		funct7,
	input	[63:0]		PC,
	
	input	[63:0]		rs1_data,
	input	[63:0]		rs2_data,
	
	
	
	
	output	[4:0]		wb_rd_sel,
	output				wb_dest_enable,
	output				wb_dest_long_enable,
	output	[63:0]		wb_dest,
	
	output	[63:0]		branch_target,	
	output				branch_target_enable,
	
	
	output [63:0]		mem_req_addr,
	output [63:0]		mem_req_data,
	output [3:0]		mem_req_op,
	output [4:0]		mem_req_rd,
	
	output				mem_req_valid,
	input				mem_req_retry	
	

);

//`define COMPILATION_SWITCH_EXECUTE


/* verilator lint_off UNUSED */
`define OP_IMM				7'b0010_011
`define OP_IMM_32			7'b0011_011
`define OP_32				7'b0111_011
`define OP					7'b0110_011
`define LUI					7'b0110_111
`define AUIPC				7'b0010_111
`define JAL					7'b1101_111
`define JALR				7'b1100_111
`define BRANCH				7'b1100_011
`define LOAD				7'b0000_011
`define STORE				7'b0100_011

`define OP_IS_ADD_SUB		3'b000
`define OP_IS_SLL			3'b001
`define OP_IS_SLT			3'b010
`define OP_IS_SLTU			3'b011
`define OP_IS_XOR			3'b100
`define OP_IS_SR			3'b101
`define OP_IS_OR			3'b110
`define OP_IS_AND			3'b111

`define BRANCH_IS_BEQ		3'b000
`define BRANCH_IS_BNE		3'b001
`define BRANCH_IS_BLT		3'b100
`define BRANCH_IS_BGE		3'b101
`define BRANCH_IS_BLTU		3'b110
`define BRANCH_IS_BGEU		3'b111

wire						instr_is_SUB;
wire						instr_is_SUBW;
wire [63:0]					op_1;
wire [63:0]					op_2;
wire [ 5:0]					shamt;	
	

wire [63:0]					ADD_RESULT;
wire [63:0]					SUB_RESULT;
wire [63:0]					ADD_SUB_RESULT;
			
wire [63:0]					XOR_RESULT;
wire [63:0]					OR_RESULT;
wire [63:0]					AND_RESULT;

wire [63:0]					SLL_RESULT;
wire [63:0]					SRL_RESULT;
wire [63:0]					SRA_RESULT;
wire [63:0]					SR_RESULT;
wire [63:0]					BRANCH_TEMP;

wire [63:0]					last_dest;
wire [4:0]					last_rd_sel;
wire						last_dest_enable;
wire						last_dest_long_enable;
wire [63:0]					JALR_TEMP;


reg	[63:0]					dest_temp;
reg							LT_FLAG;
reg							BGE_FLAG;
reg	[63:0]					PC_relative_value;
reg	[63:0]					branch_target_temp;
reg							branch_target_enable_temp;
reg							execute_ack_data_stop_next;
reg	[63:0]					src1;
reg	[63:0]					src2;

reg	[63:0]					mem_req_addr_next;
reg	[63:0]					mem_req_data_next;
reg	[3:0]					mem_req_op_next;
reg	[4:0]					mem_req_rd_next;
reg							mem_req_valid_next;
wire						execute_mem_rety;
wire						mem_ask_rety;


always@(*)begin

	if((last_rd_sel == exe_rs1_sel)&(last_dest_enable))
		begin
			if(last_dest_long_enable)
				src1	=	last_dest;
			else
				src1	=	{rs1_data[63:32],last_dest[31:0]};
		end
	else
		src1	=	rs1_data;
end

always@(*)begin
	if((last_rd_sel == exe_rs2_sel)&(last_dest_enable))
		begin
			if(last_dest_long_enable)
				src2	=	last_dest;
			else
				src2	=	{rs2_data[63:32],last_dest[31:0]};
		end
	else
		src2	=	rs2_data;
end	
	
	
	
	
	
assign 	instr_is_SUB  	=  	(funct3 == `OP_IS_ADD_SUB)&(op_code == `OP)&(funct7 == 7'b0100_000);
assign 	instr_is_SUBW  	= 	(funct3 == `OP_IS_ADD_SUB)&(op_code == `OP_32)&(funct7 == 7'b0100_000);

assign	op_1		=	src1;
assign	op_2		=	imm_rs2_sel?(sign_ex_imm):(src2);
assign	shamt		=	imm_rs2_sel?(shift_amount):({1'b0,(src2[4:0])});

assign	ADD_RESULT	=	op_1 + op_2;
assign	SUB_RESULT	=	op_1 - op_2;
assign	ADD_SUB_RESULT	=	(instr_is_SUB|instr_is_SUBW)?(SUB_RESULT):(ADD_RESULT);
assign	XOR_RESULT	=	op_1 ^ op_2;
assign	OR_RESULT	=	op_1 | op_2;
assign	AND_RESULT	=	op_1 & op_2;
assign	SLL_RESULT	=	op_1<<shamt;
assign	SRL_RESULT	=	op_1>>shamt;
assign	SRA_RESULT	=  	((op_1))>>>shamt;
assign	SR_RESULT	=	(funct7[5])?SRA_RESULT:SRL_RESULT;
assign	BRANCH_TEMP	=	PC	+	{{51{SB_imm[11]}},SB_imm,1'b0};//{{51{SB_imm[11]}},SB_imm,1'b0}
assign	JALR_TEMP 	= src1	+	sign_ex_imm;
/*state machine to control the execute stage to function properly*/



/*main*/
always@(*)begin

	case({op_1[63],op_2[63]})
		2'b01:
		begin
			if(comp_is_unsigned)
				begin
					LT_FLAG		= 	1'b1;
					BGE_FLAG	= 	1'b0;
				end
			else
				begin
					LT_FLAG		= 	1'b0;
					BGE_FLAG	= 	1'b1;												
				end
		end
		2'b10:
		begin
			if(comp_is_unsigned)
				begin
					LT_FLAG		= 	1'b0;
					BGE_FLAG	= 	1'b1;	
				end
			else
				begin
					LT_FLAG		= 	1'b0;
					BGE_FLAG	= 	1'b1;													
				end
		end
		default:
		begin
			if(op_1<op_2)
				begin
					LT_FLAG		=	1'b1;
					BGE_FLAG	=	1'b0;						
				end
			else
				begin
					LT_FLAG		=	1'b0;
					BGE_FLAG	=	1'b1;												
				end
		end
	endcase

end

always@(*)begin
	case(op_code)
		`AUIPC:		PC_relative_value	=	PC + {{32{U_imm[19]}},U_imm,12'b0};
		`JAL,`JALR:	PC_relative_value	=	PC + 4;
		default:	PC_relative_value	=	PC;
	endcase
end

always@(*)begin
	case(op_code)
		`JAL:	
			begin
				branch_target_enable_temp = 1'b1;
				branch_target_temp	=	PC  + {{43{UJ_imm[19]}},UJ_imm,1'b0};
			end
		`JALR:
			begin
				branch_target_enable_temp = 1'b1;
				branch_target_temp	=	{JALR_TEMP[63:1],1'b0};
			end
		`BRANCH:
			begin
				case(funct3)
				`BRANCH_IS_BEQ:	
					begin
						if(src1 == src2)
							begin
								branch_target_enable_temp	=	1'b1;
								branch_target_temp	= BRANCH_TEMP;
							end
						else
							begin
								branch_target_enable_temp	=	1'b0;
								branch_target_temp	= PC;										
							end
					end
				`BRANCH_IS_BNE:
					begin
						if(src1 != src2)
							begin
								branch_target_enable_temp	=	1'b1;
								branch_target_temp	= BRANCH_TEMP;
							end
						else
							begin
								branch_target_enable_temp	=	1'b0;
								branch_target_temp	= PC;										
							end
					end		
				`BRANCH_IS_BLT,`BRANCH_IS_BLTU:		
					begin
						if(LT_FLAG)
							begin
								branch_target_enable_temp	=	1'b1;
								branch_target_temp	= BRANCH_TEMP;						
							end
						else
							begin
								branch_target_enable_temp	=	1'b0;
								branch_target_temp	= PC;
							end
					end
				`BRANCH_IS_BGE,`BRANCH_IS_BGEU:
					begin
						if(BGE_FLAG)
							begin
								branch_target_enable_temp	=	1'b1;
								branch_target_temp	= BRANCH_TEMP;												
							end
						else
							begin
								branch_target_enable_temp	=	1'b0;
								branch_target_temp	= PC;										
							end
					end
				default:	
						begin
							branch_target_enable_temp	=	1'b0;
							branch_target_temp 	=	PC;
						end
				endcase
			end
		default: 
			begin
				branch_target_enable_temp = 1'b0;
				branch_target_temp	=	PC;
			end
	endcase
end

always@(*)begin

	case(op_code)
		`LUI					:	dest_temp	= {{32{U_imm[19]}},U_imm,12'b0};
		`AUIPC,`JAL,`JALR		:	dest_temp	= PC_relative_value;
		`OP_IMM	,`OP_IMM_32	,`OP_32	,`OP 	:
											begin
												case(funct3)
														`OP_IS_ADD_SUB 	: 	dest_temp 	= 	ADD_SUB_RESULT;
														`OP_IS_SLL	   	: 	dest_temp 	= 	SLL_RESULT;
														`OP_IS_SLT	   	: 	dest_temp	=	{63'b0,LT_FLAG};
														`OP_IS_SLTU	   	:	dest_temp	=	{63'b0,LT_FLAG};
														`OP_IS_XOR	   	:	dest_temp	=	XOR_RESULT;
														`OP_IS_SR		:	dest_temp  	= 	SR_RESULT;
														`OP_IS_OR		:	dest_temp	=	OR_RESULT;
														`OP_IS_AND		:	dest_temp	=	AND_RESULT;
												endcase	
											end	
		default :	dest_temp = 64'b0;
	endcase

end	




assign	mem_ask_rety			=	 execute_mem_rety & ((op_code == `STORE)|(op_code == `LOAD));

assign	wb_dest					= 	(execute_ack_data_valid&(~mem_ask_rety))	?	dest_temp	:	64'b0;	
assign	wb_rd_sel 				= 	(execute_ack_data_valid&(~mem_ask_rety))	?	decode_dest_sel	:	5'b0;
assign	wb_dest_enable 			= 	(execute_ack_data_valid&(~mem_ask_rety))	&	( ((op_code == `OP_IMM)|(op_code == `OP_IMM_32)|(op_code == `OP_32)|(op_code == `OP)|(op_code == `LUI)|(op_code == `AUIPC)|(op_code == `JAL)|(op_code == `JALR)) );	
assign	wb_dest_long_enable		= 	(execute_ack_data_valid&(~mem_ask_rety))	&	( ((op_code == `OP_IMM)|(op_code == `OP)|(op_code == `LUI)|(op_code == `AUIPC)|(op_code == `JAL)|(op_code == `JALR)) );

	
assign	branch_target			=	(execute_ack_data_valid)	?	branch_target_temp:64'b0;
assign	branch_target_enable	=	(execute_ack_data_valid)	&	branch_target_enable_temp;
assign	execute_ack_data_rety	=	(execute_ack_data_valid)	&	execute_mem_rety & ((op_code == `STORE)|(op_code == `LOAD));//(op_code == `STORE)&(op_code == `LOAD)& execute_mem_rety;






flop #(.Bits(64+5+1+1)) WXBYPASS(
	
	.clk(clk),
	.reset(reset),
	
	.d({wb_dest,wb_rd_sel,wb_dest_enable,wb_dest_long_enable}),
	.q({last_dest,last_rd_sel,last_dest_enable,last_dest_long_enable})
);





always@(*)begin

	if((!execute_ack_data_rety)&execute_ack_data_valid)
		begin
			case(op_code)
				`LOAD:
							begin
								mem_req_addr_next	=	src1 +	sign_ex_imm;
								mem_req_data_next	=	src2;
								mem_req_op_next		=	{1'b0,funct3};
								mem_req_rd_next		=	decode_dest_sel;
								mem_req_valid_next	=	1'b1;
							end
				`STORE:
							begin
								mem_req_addr_next	=	src1 +	{{52{S_imm[11]}},S_imm};
								mem_req_data_next	=	src2;
								mem_req_op_next		=	{1'b1,funct3};
								mem_req_rd_next		=	decode_dest_sel;
								mem_req_valid_next	=	1'b1;
							end
				default:	
						begin
								mem_req_addr_next	=	64'b0;
								mem_req_data_next	=	src2;
								mem_req_op_next		=	{1'b1,funct3};
								mem_req_rd_next		=	decode_dest_sel;
								mem_req_valid_next	=	1'b0;							
						end
			endcase
		end
	else
		begin
			mem_req_addr_next	=	64'b0;
			mem_req_data_next	=	src2;
			mem_req_op_next		=	{1'b1,funct3};
			mem_req_rd_next		=	decode_dest_sel;
			mem_req_valid_next	=	1'b0;	
		end
		
end






Fluid_Flop#(.Size(64+64+4+5))
MEM_ACCESS(

    .clk		(clk),
    .reset		(reset),

    .din		({mem_req_addr_next,mem_req_data_next,mem_req_op_next,mem_req_rd_next}),
    .dinValid	(mem_req_valid_next),
    .dinRetry	(execute_mem_rety),

    .q			({mem_req_addr,mem_req_data,mem_req_op,mem_req_rd}),
    .qRetry		(mem_req_retry),
    .qValid		(mem_req_valid)
    
); 











//synopsys translate_off

`ifdef COMPILATION_SWITCH_EXECUTE

always@(posedge clk)begin
	if(!reset&execute_ack_data_valid)
		begin
			case(op_code)
				`LUI	:	$display("execute.v: PC:0X%h instr is LUI \n",PC);		//dest_temp	= {{32{U_imm[19]}},U_imm,12'b0};
				`AUIPC	:	$display("execute.v: PC:0X%h instr is AUIPC \n",PC);		//dest_temp	= PC_relative_value;
				`JAL	:	$display("execute.v: PC:0X%h instr is JAL \n",PC);		//dest_temp	= PC_relative_value;
				`JALR	:	$display("execute.v: PC:0X%h instr is JALR \n",PC);		//dest_temp	= PC_relative_value;
				`STORE	:	$display("execute.v: PC:0X%h instr is STORE \n",PC);
				`LOAD	:	$display("execute.v: PC:0X%h instr is LOAD \n",PC);
				`BRANCH	:	
					begin
						case(funct3)
							`BRANCH_IS_BEQ:		$display("execute.v: PC:0X%h instr is BEQ \n",PC);
							`BRANCH_IS_BNE:		$display("execute.v: PC:0X%h instr is BNE \n",PC);
							`BRANCH_IS_BLT:		$display("execute.v: PC:0X%h instr is BLT \n",PC);
							`BRANCH_IS_BLTU:	$display("execute.v: PC:0X%h instr is BLTU \n",PC);		
							`BRANCH_IS_BGE:		$display("execute.v: PC:0X%h instr is BGE \n",PC);
							`BRANCH_IS_BGEU:	$display("execute.v: PC:0X%h instr is BGEU \n",PC);
							default: 			$display("execute.v: Invalid not defined instr!!! \n");
						endcase
					end
				`OP_IMM	,`OP_IMM_32	,`OP_32	,`OP :
				begin
					case(funct3)
							`OP_IS_ADD_SUB 	: 	begin
													if(instr_is_SUB|instr_is_SUBW)
														begin
															$display("execute.v: PC:0X%h instr is SUB \n",PC);
														end
													else
														begin
															$display("execute.v: PC:0X%h instr is ADD \n",PC);
														end
												end
							`OP_IS_SLL	   	: 	$display("execute.v: PC:0X%h instr is SLL \n",PC);
							`OP_IS_SLT	   	: 	$display("execute.v: PC:0X%h instr is SLT \n",PC);
							`OP_IS_SLTU	   	:	$display("execute.v: PC:0X%h instr is SLTU \n",PC);
							`OP_IS_XOR	   	:	$display("execute.v: PC:0X%h instr is XOR \n",PC);
							`OP_IS_SR		:	begin
													if(funct7[5])
														begin
															$display("execute.v: PC:0X%h instr is SRA \n",PC);
														end
													else
														begin
															$display("execute.v: PC:0X%h instr is SRL",PC);
														end
												end
							`OP_IS_OR		:	$display("execute.v: PC:0X%h instr is OR \n",PC);
							`OP_IS_AND		:	$display("execute.v: PC:0X%h instr is AND \n",PC);
							default : $display ("\n");
					endcase	
				end
				default : $display ("\n");
			endcase
		end
end


always@(posedge clk)begin
	if(!reset)
		begin
			if(branch_target_enable)
				begin
					$display("execute.v: Branch Target enable!!\n");
					$display("execute.v: Target address is : 0X%h\n",branch_target);
				end
		end
end

always@(posedge clk)begin
	if(!reset)
		begin
			if((execute_ack_data_valid)	&	execute_mem_rety)
				begin
					if(((op_code != `STORE)|(op_code != `LOAD)))
						begin
							$display("execute.v: Program Executed Out of Order for memroy stalls. This operation is valid!!!\n");
							$display("execute.v: Current PC = 0X%h",PC);
						end
				end
		end
end

always@(posedge clk)begin
	if(!reset)
		begin
			if(execute_ack_data_rety)
				begin
					$display("execute.v: Execute stage stall because of memroy stalls. This operation is valid!!!\n");
				end
		end
end


`endif

//synopsys translate on

















endmodule
