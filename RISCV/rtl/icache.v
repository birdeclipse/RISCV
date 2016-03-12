/****************************************************************************
 * icache.v
 ****************************************************************************/

// Main Characteristics:
//
// cache hit in 1 cycle
// cache miss random delay provided by testbench (2-20 cycles)
//
// 64 entries in cache (64 cache lines). Index has 6 bits
//
// cache line has 256bits == 32bytes, offset (byte in slides) has 5bits
//
// tag has 64-6-5 == 53bits tag
//
// Align core_req_pc, Not allowed address 31,30,29, 27 25 23 21 19...
/****************************************************************************/
/* verilator lint_off WIDTH */
module icache(
	input	clk,
	input 	reset,
	
	output				icache_ack_data_retry,
	input				icache_ack_data_valid,
	input  [255:0] 		icache_ack_data,

	output [63-5:0] 	icache_req_addr,
	output        		icache_req_addr_valid,/*when miss*/
	input         		icache_req_addr_retry,
		
		
	/*pc interface*/
	input  [63-1:0]		core_req_pc,
	input				core_req_pc_valid,
	output				icache_req_next_pc_rety,

	/*Decoder interface*/
	output [31:0]		core_ack_insn,
	output 				core_ack_insn_valid/*when hit*/
);

/* verilator lint_off UNUSED */	

reg				ic_cache_hit;
reg				ic_cache_miss;

wire [63:0]		fetch_req_next_pc;
wire			fetch_req_next_pc_valid;


wire [4:0]		reg_offset;
wire [5:0]		ic_index;

wire [255:0]	ic_data;	
wire [63-5-6:0]	ic_tag;
wire			ic_tag_valid;
reg				ic_data_we;

reg [255:0]   	ic_data_next;
reg [63-5-6:0]  ic_tag_next;
reg				ic_tag_valid_next;

reg	[31:0]		core_ack_insn_temp;

assign	icache_ack_data_retry	=	1'b0;
assign	reg_offset				=	fetch_req_next_pc[ 4:0];
assign	ic_index				=	fetch_req_next_pc[10:5];

assign	fetch_req_next_pc 		= 	{core_req_pc,1'b0};
assign	fetch_req_next_pc_valid = 	core_req_pc_valid;

assign	core_ack_insn			=	core_ack_insn_temp;
assign	core_ack_insn_valid		=	ic_cache_hit;



assign	icache_req_addr			=	fetch_req_next_pc[63:5];
assign	icache_req_addr_valid	=	ic_cache_miss;
assign	icache_req_next_pc_rety	=	ic_cache_miss;


always@(*)begin
	if(fetch_req_next_pc_valid)
		begin
			if(fetch_req_next_pc[63:11]==ic_tag)
				begin
					if(ic_tag_valid)
						ic_cache_hit	=	1'b1;
					else
						ic_cache_hit	=	1'b0;
				end
			else
				begin
					ic_cache_hit	=	1'b0;
				end
		end
	else
		begin
			ic_cache_hit	=	1'b0;
		end
end

always@(*)begin

	if(fetch_req_next_pc_valid)
		begin
			if((fetch_req_next_pc[63:11] == ic_tag)&(ic_tag_valid == 1))
				begin
					ic_cache_miss	=	1'b0;
				end
			else
				begin
					ic_cache_miss	=	1'b1;
				end
		end
	else
		begin
			ic_cache_miss	=	1'b0;
		end
end

always@(*)begin
	case(reg_offset)
		0:core_ack_insn_temp	=	ic_data[(32*1-1):(32*0)];
			2:core_ack_insn_temp	=	ic_data[(32*1+16-1):(32*0+16)];
		4:core_ack_insn_temp	=	ic_data[(32*2-1):(32*1)];
			6:core_ack_insn_temp	=	ic_data[(32*2+16-1):(32*1+16)];
		8:core_ack_insn_temp	=	ic_data[(32*3-1):(32*2)];
			10:core_ack_insn_temp	=	ic_data[(32*3+16-1):(32*2+16)];
		12:core_ack_insn_temp	=	ic_data[(32*4-1):(32*3)];
			14:core_ack_insn_temp	=	ic_data[(32*4+16-1):(32*3+16)];
		16:core_ack_insn_temp	=	ic_data[(32*5-1):(32*4)];
			18:core_ack_insn_temp	=	ic_data[(32*5+16-1):(32*4+16)];
		20:core_ack_insn_temp	=	ic_data[(32*6-1):(32*5)];
			22:core_ack_insn_temp	=	ic_data[(32*6+16-1):(32*5+16)];
		24:core_ack_insn_temp	=	ic_data[(32*7-1):(32*6)];
			26:core_ack_insn_temp	=	ic_data[(32*7+16-1):(32*6+16)];
		28:core_ack_insn_temp	=	ic_data[(32*8-1):(32*7)];	
	default:core_ack_insn_temp	=	32'b0;	
	endcase
end

always@(*)begin
	if(icache_ack_data_valid)
		begin
			ic_data_next		=	icache_ack_data;
			ic_tag_next			=	fetch_req_next_pc[63:11];
			ic_tag_valid_next	=	1'b1;
			ic_data_we			=	1'b1;
		end
	else
		begin
			ic_data_next		=	ic_data;
			ic_tag_next			=	ic_tag;
			ic_tag_valid_next	=	ic_tag_valid;
			ic_data_we			=	1'b0;				
		end
end


//synopsys translate_off
always@(posedge clk)begin
	if(icache_req_addr_retry)
		begin
			$display("Icache mem retry\n");
		end
end
//synopsys translate on


sram #(
	.RAM_WIDTH(256),
	.RAM_DEPTH(6),
	.ADDR_WIDTH(64)	
)
cache_data(
	.clk(clk),
	.reset(reset),
	.we(ic_data_we),
	.addr(ic_index),
	
	.d(ic_data_next),
	.q(ic_data)
);	




sram #(
	.RAM_WIDTH(53),
	.RAM_DEPTH(6),
	.ADDR_WIDTH(64)
)

cache_tag(
	.clk(clk),
	.reset(reset),
	.we(ic_data_we),
	.addr(ic_index),
	
	.d(ic_tag_next),
	.q(ic_tag)
);





sram #(
    .RAM_WIDTH(1),
    .RAM_DEPTH(6),
    .ADDR_WIDTH(64)
) 
cache_tag_valid(
	.clk(clk),
	.reset(reset),
	.we(ic_data_we),
	.addr(ic_index),
	
	.d(ic_tag_valid_next),
	.q(ic_tag_valid)
);



	
endmodule
