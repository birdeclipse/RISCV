`timescale 1ns/1ns
module core_top_test();


reg 		clk = 0;
reg 		reset = 1;
reg		icache_ack_data_valid = 0;

reg  [255:0] 	rom [4:0];

reg  [255:0] 	icache_ack_data = 256'h 00151513_00b787b3_00f507b3_00000793_08060693_00060713_00158593_00150513;
reg  [4:0]	rom_counter = 4'd8;


wire [63:0]	icache_req_addr;
wire        	icache_req_addr_valid;/*when miss*/	

wire [63:0] 	debug_pc_ex;
wire        	debug_dest_valid;
wire [63:0] 	debug_dest;
wire [4:0] 	debug_dest_rd;
wire        	debug_dest_long;



wire [63:0] 	dcache_req_addr;  	// load or store address computed at execute
wire [63:0] 	dcache_req_data; 	// data just for the store going to from exe to testbench
wire [3:0]  	dcache_req_op;		// RVMOP_*
wire [4:0]  	dcache_req_rd; 		// destination register for the load
wire        	dcache_req_valid;
wire			dcache_req_retry;

reg  [63:0]	ram[63:0];

reg  [63:0]	write_next;

reg  [63:0]	dcache_ack_data = 0;
reg  [4:0]	dcache_ack_rd = 0;
reg		dcache_ack_valid = 0;	

wire [63:0] 	mem_gen_req_addr;  		// load or store address computed at execute
wire [63:0] 	mem_gen_req_data; 		// data just for the store going to from exe to testbench
wire [3:0]  	mem_gen_req_op;			// RVMOP_*
wire [4:0]  	mem_gen_req_rd; 		// destination register for the load
wire        	mem_gen_req_valid;
reg				mem_gen_req_retry;




core core_top_test(

	.clk(clk),
	.reset(reset),
	
	// i cache interface
	.icache_ack_data_retry(),
	.icache_ack_data_valid(icache_ack_data_valid),
	.icache_ack_data(icache_ack_data),
	
	.icache_req_addr(icache_req_addr),
	.icache_req_addr_valid(icache_req_addr_valid),
	.icache_req_addr_retry(1'b0),
	
	// dcache interface (from execute to testbench)
	.dcache_req_addr(dcache_req_addr),  	// load or store address computed at execute
	.dcache_req_data(dcache_req_data), 	// data just for the store going to from exe to testbench
	.dcache_req_op(dcache_req_op),  		// RVMOP_*
	.dcache_req_rd(dcache_req_rd),  		// destination register for the load
	.dcache_req_valid(dcache_req_valid),
	
	.dcache_req_retry(dcache_req_retry),
	// dcache interface from testbench to decode
	.dcache_ack_data(dcache_ack_data),
	.dcache_ack_rd(dcache_ack_rd),  	// destination register for the load
	
	
	.dcache_ack_valid(dcache_ack_valid),
	.dcache_ack_retry(), 	// ALWAYS_FALSE
	
	// Debug interface
	.debug_pc_ex(debug_pc_ex),
	
	.debug_dest_valid(debug_dest_valid),
	.debug_dest(debug_dest),
	.debug_dest_rd(debug_dest_rd),
	.debug_dest_long(debug_dest_long)

);





Fluid_Flop#(.Size(64+64+4+5))
MEM_GEN(

	.clk		(clk),
	.reset		(reset),
	
	.din		({dcache_req_addr,dcache_req_data,dcache_req_op,dcache_req_rd}),
	.dinValid	(dcache_req_valid),
	.dinRetry	(dcache_req_retry),
	
	.q			({mem_gen_req_addr,mem_gen_req_data,mem_gen_req_op,mem_gen_req_rd}),
	.qRetry		(mem_gen_req_retry),
	.qValid		(mem_gen_req_valid)
    
);


initial begin
	//rom[0] = 256'h 00151513_00b787b3_00f507b3_00000793_08060693_00060713_00158593_00150513;
	//rom[1] = 256'h 0006b503_04060613_4075d593_40655793_fed714e3_00870713_00f73023_00159593;
	//rom[2] = 256'h 00008067_fec692e3_ff868693_4015d593_4017d793_00a6b023_40b50533_40f50533;
	//rom[3] = 256'h 00000000_00000000_00000000_00000000_00000000_00000000_00000000_00000000;
	
	rom[0] = 256'h 06100793000547030005849300050413009134230081382300113c23fe010113;
	rom[1] = 256'h 001707130ff7f7930017879b00f7002307b00693061007930005071302f70263;
	rom[2] = 256'h 000485130004059300050c63000300e7000003170004051300048593fed798e3;
	rom[3] = 256'h 00050413000300e70000031700040513000485930180006f000300e700000317;
	rom[4] = 256'h 0000000000000000000080670201011300813483010134030181308300040513;	
	
end

integer i;
initial begin
for(i=0;i<128;i=i+1)
    begin
	ram[i] = 64'h0;
    end
end


always@(*)
	begin
		case(mem_gen_req_op)
			4'b1_000: 	write_next = 	{ram[mem_gen_req_addr[63:3]][63:8],mem_gen_req_data[7:0]};					/*byte*/
			4'b1_001:	write_next =	{ram[mem_gen_req_addr[63:3]][63:16],mem_gen_req_data[15:0]};				/*half word*/
			4'b1_010:	write_next =	{ram[mem_gen_req_addr[63:3]][63:32],mem_gen_req_data[31:0]};				/*word*/
			4'b1_011:	write_next =	mem_gen_req_data;															/*long word*/
		endcase
	end



always@(posedge clk)begin

	if(mem_gen_req_valid)
		begin
			if(mem_gen_req_op[3])
				begin
					ram[mem_gen_req_addr[63:3]]	<= write_next;
				end
		end	
end



always@(posedge clk)begin
	if(mem_gen_req_valid)
		begin
			case(mem_gen_req_op)
				4'b0_000:	dcache_ack_data	<=	{56'b0,ram[mem_gen_req_addr[63:3]][7:0]};										/**/
				4'b0_001:	dcache_ack_data	<=	{48'b0,ram[mem_gen_req_addr[63:3]][15:0]};										/**/
				4'b0_010:	dcache_ack_data	<=	{32'b0,ram[mem_gen_req_addr[63:3]][31:0]};										/**/
				4'b0_100:	dcache_ack_data	<=	{{56{ram[mem_gen_req_addr[63:3]][7]}},ram[mem_gen_req_addr[63:3]][7:0]};		/**/
				4'b0_101:	dcache_ack_data	<=	{{48{ram[mem_gen_req_addr[63:3]][15]}},ram[mem_gen_req_addr[63:3]][15:0]};		/**/
				4'b0_110:	dcache_ack_data	<=	{{32{ram[mem_gen_req_addr[63:3]][31]}},ram[mem_gen_req_addr[63:3]][31:0]};		/**/
				4'b0_011:	dcache_ack_data	<=	ram[mem_gen_req_addr[63:3]];													/**/
			endcase
				if(mem_gen_req_op[3] == 0)
					begin
						dcache_ack_valid	<= 1'b1;
						dcache_ack_rd		<= mem_gen_req_rd;
					end
				else
					begin
						dcache_ack_valid	<= 1'b0;
						dcache_ack_rd		<= mem_gen_req_rd;						
					end
		end
	else
		begin
			dcache_ack_valid	<= 0;
			dcache_ack_rd		<= 0;
		end
end



always@(posedge clk)begin
	if(reset)
		begin
			mem_gen_req_retry	<= 1'b0;
		end
	else
		begin
			if(mem_gen_req_valid)
				begin
					mem_gen_req_retry	<= 1'b0;
				end
			else if(!mem_gen_req_retry)
					mem_gen_req_retry	<= 1'b1;
		end
end



initial begin
    #640  reset =0;

end

always clk = #5 ~clk;


always@(*)begin

	icache_ack_data		<=	rom [icache_req_addr[63:5]];

end


always@(posedge clk)begin
	if(reset)
		begin
			icache_ack_data_valid	<=	1'b0;
		end
	else 
		begin
			if(icache_req_addr_valid)
				begin
					if(icache_ack_data_valid)
						begin
							icache_ack_data_valid 	<= 1'b0;
						end
					else
						begin
							icache_ack_data_valid	<=	(rom_counter == 4'd0);
						end
					if(rom_counter == 4'd0)
						begin
							rom_counter	<= $random();	
						end
					else
						rom_counter	<=	rom_counter -1;
				end
			else
				begin
					icache_ack_data_valid	<=	1'b0;
				end
		end
end

always@(posedge clk)begin

	if((debug_dest == 64'd2034)&(debug_dest_valid))
		begin
			$display("program executed successfully !!! answer is %d\n",debug_dest);
			$display("PASSED at time=%dns!!!\n",$time);
		end

end



endmodule