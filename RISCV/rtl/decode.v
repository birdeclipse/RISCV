//==============================================================================
//      File:           decode.v
//      Version:        $Revision$
//      Author:         yuxun Qiu
//==============================================================================

//==============================================================================
//      Section:        License
//==============================================================================
//      Copyright (c) 2011, Regents of the University of California
//      All rights reserved.
//
//      Redistribution and use in source and binary forms, with or without modification,
//      are permitted provided that the following conditions are met:
//
//              - Redistributions of source code must retain the above copyright notice,
//                      this list of conditions and the following disclaimer.
//              - Redistributions in binary form must reproduce the above copyright
//                      notice, this list of conditions and the following disclaimer
//                      in the documentation and/or other materials provided with the
//                      distribution.
//              - Neither the name of the University of California, Santa Cruz nor the
//                      names of its contributors may be used to endorse or promote
//                      products derived from this software without specific prior
//                      written permission.
//
//      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//      ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//      WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//      DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
//      ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//      (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//      LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
//      ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//      SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//==============================================================================

/****************************************************************************
 * decode.v
 ****************************************************************************/

/* verilator lint_off UNUSED */
module decode (
	           input         clk,
	           input         reset,


	           input [63:0]  decode_ack_pc,
	           input [31:0]  decode_ack_instr,
	           input         decode_ack_data_valid,
	           output        decode_ack_data_rety,

	           output [4:0]  decode_rs1_sel,
	           output [4:0]  decode_rs2_sel,
	           input [63:0]  rs1_data,
	           input [63:0]  rs2_data,


	           output [4:0]  exe_rs1_sel,
	           output [4:0]  exe_rs2_sel,
	           output [63:0] exe_rs1_data,
	           output [63:0] exe_rs2_data,


	           output [4:0]  decode_dest_sel,
	           output        imm_rs2_sel,
	           output        comp_is_unsigned,
	           output [63:0] sign_ex_imm,
	           output [5:0]  shift_amount,

	           output [19:0] U_imm,
	           output [19:0] UJ_imm,
	           output [11:0] S_imm,
	           output [11:0] SB_imm,

	           output [6:0]  op_code,
	           output [2:0]  funct3,
	           output [6:0]  funct7,
	           output [63:0] pc_latch,

	           input         decode_flush,

	           output        execute_ack_data_valid,
	           input         execute_ack_data_rety,

	           input [4:0]   dcache_ack_rd,
	           input         dcache_ack_valid
	           );

`define COMPILATION_SWITCH
`define OP_IMM			7'b0010_011
`define OP_IMM_32		7'b0011_011
`define OP_32			7'b0111_011
`define OP			    7'b0110_011
`define LUI			    7'b0110_111
`define AUIPC			7'b0010_111
`define JAL			    7'b1101_111
`define JALR			7'b1100_111
`define BRANCH			7'b1100_011
`define LOAD			7'b0000_011
`define STORE			7'b0100_011




`define OP_IS_ADD_SUB	3'b000
`define OP_IS_SLL		3'b001
`define OP_IS_SLT		3'b010
`define OP_IS_SLTU		3'b011

`define OP_IS_XOR		3'b100
`define OP_IS_SR		3'b101
`define OP_IS_OR		3'b110
`define OP_IS_AND		3'b111

`define BRANCH_IS_BEQ	3'b000
`define BRANCH_IS_BNE	3'b001
`define BRANCH_IS_BLT	3'b100
`define BRANCH_IS_BGE	3'b101
`define BRANCH_IS_BLTU	3'b110
`define BRANCH_IS_BGEU	3'b111




   logic                     decode_ack_rety;
   logic [31:0]              instr;
   logic [31:0]              pending_instr;
   logic [63:0]              pending_PC;
   logic [11:0]              Imm12;
   logic [4:0]               shamt;
   logic                     instr_is_SLTIU;
   logic                     instr_is_SLTU;
   logic                     instr_is_BGEU;
   logic                     instr_is_BLTU;
   logic [4:0]               decode_dest_sel_next;
   logic                     imm_rs2_sel_next;
   logic                     instr_is_unsigned;
   logic [63:0]              sign_ex_imm_next;
   logic [5:0]               shift_amount_next;
   logic [19:0]              U_imm_next;
   logic [19:0]              UJ_imm_next;
   logic [11:0]              S_imm_next;
   logic [11:0]              SB_imm_next;
   logic [6:0]               op_code_next;
   logic [2:0]               funct3_next;
   logic [6:0]               funct7_next;
   logic [63:0]              pc_latch_next;
   logic                     decode_rs1_sel_valid;
   logic                     decode_rs2_sel_valid;
   logic                     instr_is_load;
   logic                     src1_is_pending;
   logic                     src2_is_pending;
   logic                     last_src1_pending;
   logic                     last_src2_pending;
   logic                     instr_need_rs1;
   logic                     instr_need_rs2;


   assign	instr			=	(last_src1_pending|last_src2_pending) ? pending_instr : decode_ack_instr;
   assign	pc_latch_next	=	(last_src1_pending|last_src2_pending) ? (pending_PC) :(decode_ack_pc);
   assign	Imm12			=  	instr[31:20];
   assign	shamt			=	instr[24:20];

   assign	U_imm_next		=	(instr[31:12]);
   assign	UJ_imm_next		=	({instr[31],instr[19:12],instr[20],instr[30:21]});
   assign	S_imm_next		=	({instr[31:25],instr[11:7]});
   assign	SB_imm_next		=	({instr[31],instr[7],instr[30:25],instr[11:8]});
   assign	op_code_next	=	(instr[ 6: 0]);
   assign	funct3_next		=  	(instr[14:12]);
   assign 	funct7_next 	=  	(instr[31:25]);


   assign	instr_is_SLTIU	    = 	(funct3_next == `OP_IS_SLTU)&(op_code_next == `OP_IMM);
   assign	instr_is_SLTU 		= 	(funct3_next == `OP_IS_SLTU)&(op_code_next == `OP)&(funct7_next == 7'b0000_000);
   assign	instr_is_BGEU		= 	(funct3_next == `BRANCH_IS_BGEU)&(op_code_next == `BRANCH);
   assign	instr_is_BLTU		= 	(funct3_next == `BRANCH_IS_BLTU)&(op_code_next == `BRANCH);
   assign	instr_is_unsigned	=	(instr_is_BLTU|instr_is_BGEU|instr_is_SLTIU|instr_is_SLTU);
   assign	sign_ex_imm_next	=	((instr_is_unsigned)?({52'b0,Imm12}):({{52{Imm12[11]}},Imm12}));
   assign	imm_rs2_sel_next	=	((op_code_next == `OP_IMM)|(op_code_next == `OP_IMM_32));
   assign	shift_amount_next	=	({instr[25],shamt});
   assign	decode_dest_sel_next	=	(instr[11: 7]);



   assign	instr_is_load		=	(decode_ack_data_valid&(!decode_flush))&(op_code_next == `LOAD);

   assign   instr_need_rs1      =    ((op_code_next == `OP_IMM)|
					                  (op_code_next == `OP_IMM_32)|
					                  (op_code_next == `OP_32)|
					                  (op_code_next == `OP)|
					                  (op_code_next == `JALR)|
					                  (op_code_next == `BRANCH)|
					                  (op_code_next == `LOAD)|
					                  (op_code_next == `STORE));

   assign   instr_need_rs2      =    ((op_code_next == `OP)|
					                  (op_code_next == `OP_32)|
					                  (op_code_next == `BRANCH)|
					                  (op_code_next == `STORE));

   assign	decode_rs1_sel_valid	=	(last_src1_pending|last_src2_pending) ? 1'b1 : (decode_ack_data_valid&(!decode_flush))&instr_need_rs1;
   assign	decode_rs1_sel		    =	(instr[19:15]);


   assign	decode_rs2_sel_valid	=	(last_src1_pending|last_src2_pending) ? 1'b1 : (decode_ack_data_valid&(!decode_flush))&instr_need_rs2;
   assign	decode_rs2_sel		    =	(instr[24:20]);


   assign	decode_ack_data_rety	=	(!decode_flush)&((decode_ack_rety)|(src1_is_pending)|(src2_is_pending));


   pending_check
     CHECK_PENDING(
		           .clk(clk),
		           .reset(reset),

		           .instr_is_load(instr_is_load),
		           .decode_dest_sel_next(decode_dest_sel_next),


		           .decode_rs1_sel(decode_rs1_sel),
		           .decode_rs1_sel_valid(decode_rs1_sel_valid),

		           .decode_rs2_sel(decode_rs2_sel),
		           .decode_rs2_sel_valid(decode_rs2_sel_valid),
		           .src1_is_pending(src1_is_pending),
		           .src2_is_pending(src2_is_pending),

		           .dcache_ack_rd(dcache_ack_rd),
		           .dcache_ack_valid(dcache_ack_valid)
		           );




   flop #(.Bits(2))
   LAST_PENDING(

		        .clk(clk),
		        .reset(reset),

		        .d({src1_is_pending,src2_is_pending}),
		        .q({last_src1_pending,last_src2_pending})
		        );



   flop_e #(.Bits(32+64))
   PENDING_INSTRUCTION(
		               .clk(clk),
		               .reset(reset),

		               .we(src1_is_pending|src2_is_pending),
		               .d({instr,pc_latch_next}),
		               .q({pending_instr,pending_PC})
		               );




   Fluid_Flop#(.Size(5+5+210+12+128))
   EXECUTE(

	       .clk		(clk),
	       .reset	(reset),

	       .din		({decode_rs1_sel,
                      decode_rs2_sel,
                      rs1_data,
                      rs2_data,
                      decode_dest_sel_next,
                      imm_rs2_sel_next,
                      instr_is_unsigned,
                      sign_ex_imm_next,
                      shift_amount_next,
                      U_imm_next,
                      UJ_imm_next,
                      S_imm_next,
                      SB_imm_next,
                      op_code_next,
                      funct3_next,
                      funct7_next,
                      pc_latch_next}),

	       .dinValid	(decode_ack_data_valid&(!decode_flush)&(!src1_is_pending)&(!src2_is_pending)),
	       .dinRetry	(decode_ack_rety),

	       .q		({exe_rs1_sel,
                      exe_rs2_sel,
                      exe_rs1_data,
                      exe_rs2_data,
                      decode_dest_sel,
                      imm_rs2_sel,
                      comp_is_unsigned,
                      sign_ex_imm,
                      shift_amount,
                      U_imm,
                      UJ_imm,
                      S_imm,
                      SB_imm,
                      op_code,
                      funct3,
                      funct7,
                      pc_latch}),

	       .qRetry	(execute_ack_data_rety|last_src1_pending|last_src2_pending),
	       .qValid	(execute_ack_data_valid)

           );



   //synopsys translate_off

`ifdef COMPILATION_SWITCH

   always@(posedge clk)begin
      if(src1_is_pending | src2_is_pending)
	    if((!last_src1_pending) & (!last_src2_pending))
	      begin
	         $display("Decode.v: Decode Stage detected an DATA Dependence Decode stage Stalling.\n");
	         if(src1_is_pending)
	           $display("Decode.v: Pending Rs%d; \n",decode_rs1_sel);
	         if(src2_is_pending)
	           $display("Decode.v: Pending Rs%d; \n",decode_rs2_sel);
	         $display("Decode.v: Current PC = 0X%h\n",decode_ack_pc);
	      end
   end

`endif
   //synopsys translate on



endmodule

