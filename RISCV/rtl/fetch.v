//==============================================================================
//      File:           fetch.v
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
 * fetch.v
 ****************************************************************************/

module fetch(
	         input           clk,
	         input           reset,

	         output          icache_ack_data_retry,
	         input           icache_ack_data_valid,
	         input [255:0]   icache_ack_data,

	         output [63-5:0] icache_req_addr,
	         output          icache_req_addr_valid,
	         input           icache_req_addr_retry,
             /*branch and stall*/
	         input [63:0]    branch_target,
	         input           branch_target_enable,

	         output [63:0]   decode_ack_pc,
	         output [31:0]   decode_ack_instr,
	         output          decode_ack_data_valid,
	         input           decode_ack_data_rety
             );

   /* verilator lint_off UNUSED */

   wire [63:0]               program_counter;
   wire                      fetch_req_next_pc_valid;
   wire                      icache_req_next_pc_rety;
   wire [31:0]               icache_ack_instr;
   wire                      icahce_ack_instr_valid;
   wire                      icache_ack_instr_rety;


   reg [63:0]                program_counter_next;
   reg [63:0]                fetch_ack_pc_next;
   reg [63:0]                fetch_req_next_pc;

   assign	fetch_req_next_pc_valid	= 1'b1;/*always valid*/



   always_comb
     begin
	    if(branch_target_enable)
	      begin
	         fetch_req_next_pc	=	branch_target;
	      end
	    else if(icache_req_next_pc_rety|icache_ack_instr_rety|decode_ack_data_rety)
	      begin
	         fetch_req_next_pc	=	program_counter;
	      end
	    else
	      begin
	         fetch_req_next_pc	=	program_counter+4;
	      end
     end

   always_comb
     begin
	    program_counter_next	= fetch_req_next_pc;
     end



   always_comb
     begin
	    fetch_ack_pc_next = program_counter;
     end






   icache ICACHE(

	             .clk(clk),
	             .reset(reset),

	             .icache_ack_data_retry(icache_ack_data_retry),
	             .icache_ack_data_valid(icache_ack_data_valid),
	             .icache_ack_data(icache_ack_data),

	             .icache_req_addr(icache_req_addr),
	             .icache_req_addr_valid(icache_req_addr_valid),/*when miss*/
	             .icache_req_addr_retry(icache_req_addr_retry),

	             /*pc interface*/
	             .core_req_pc(fetch_ack_pc_next[63:1]),
	             .core_req_pc_valid(fetch_req_next_pc_valid),
	             .icache_req_next_pc_rety(icache_req_next_pc_rety),

	             /*Decoder interface*/
	             .core_ack_insn(icache_ack_instr),
	             .core_ack_insn_valid(icahce_ack_instr_valid)/*when hit*/

                 );



   flop #(.Bits(64)) PC(

	                    .clk(clk),
	                    .reset(reset),

	                    .d(program_counter_next),
	                    .q(program_counter)
                        );


   Fluid_Flop#(.Size(96))
   DECODE(

          .clk(clk),
          .reset(reset),

          .din({fetch_ack_pc_next,icache_ack_instr}),
          .dinValid(icahce_ack_instr_valid&(~branch_target_enable)&(~decode_ack_data_rety)),
          .dinRetry(icache_ack_instr_rety),

          .q({decode_ack_pc,decode_ack_instr}),
          .qRetry(decode_ack_data_rety),
          .qValid(decode_ack_data_valid)

          );

endmodule
