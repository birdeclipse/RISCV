//==============================================================================
//      File:           regfile.v
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
 * regfile.v
 ****************************************************************************/

/* verilator lint_off UNUSED */
module regfile(

	           input         clk,
	           input         reset,
	           input [4:0]   rs1_sel,
	           input [4:0]   rs2_sel,

	           input [63:0]  dcache_ack_data,
	           input [4:0]   dcache_ack_rd, // destination register for the load
	           input         dcache_ack_valid,
	           output        dcache_ack_retry, // ALWAYS_FALSE

	           input [4:0]   rd_sel,
	           input         dest_enable,
	           input         dest_long_enable,
	           input [63:0]  dest,

	           output [63:0] rs1_data,
	           output [63:0] rs2_data
               );

   reg [63:0]                rf[31:0];
   reg [4:0]                 reset_counter	=	5'b0;


   reg [63:0]                rs1_data_next;
   reg [63:0]                rs2_data_next;

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

   assign   rs1_data	        =	rs1_data_next;
   assign   rs2_data	        =	rs2_data_next;
   assign   dcache_ack_retry	=	1'b0;

endmodule
