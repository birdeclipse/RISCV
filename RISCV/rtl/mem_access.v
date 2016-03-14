//==============================================================================
//      File:           mem_access.v
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

module mem_access(

	              input          clk,
	              input          reset,

                  /*execute to memory statge interface*/
	              output         mem_req_valid,
	              input          mem_req_retry,
	              output [63:0]  wb_dest,
	              output [4:0]   wb_rd_sel,
	              output         wb_dest_enable,
	              output         wb_dest_long_enable,


	              output [63:0]  mem_req_addr,
	              output [63:0]  mem_req_data,
	              output [3:0]   mem_req_op,
	              output [4:0]   mem_req_rd,
                  output         mem_req_enable,



                  /*decache_interface*/
	              output [63:0]  dcache_req_addr,
	              output [255:0] dcache_req_data,
	              output         dcache_req_valid,
	              input          dcache_req_retry

                  );

`define COMPILATION_SWITCH




























































`ifdef COMPILATION_SWITCH

   always@(posedge clk)begin

      if(dcache_req_valid&!dcache_req_retry)
        begin
	       case(dcache_req_op)
	         4'b1_000: 	$display("mem_access.v: processor made an SB request: type: %b; Data: %X; address: 0X%h;\n",dcache_req_op,dcache_req_data,dcache_req_addr);/*byte*/
	         4'b1_001:	$display("mem_access.v: processor made an SH request: type: %b; Data: %X; address: 0X%h;\n",dcache_req_op,dcache_req_data,dcache_req_addr);/*half word*/
	         4'b1_010:	$display("mem_access.v: processor made an SW request: type: %b; Data: %X; address: 0X%h;\n",dcache_req_op,dcache_req_data,dcache_req_addr);/*word*/
	         4'b1_011:	$display("mem_access.v: processor made an SD request: type: %b; Data: %X; address: 0X%h;\n",dcache_req_op,dcache_req_data,dcache_req_addr);/*long word*/

	         4'b0_000:	$display("mem_access.v: processor made an LB request: type: %b; Destination: R%d; address: 0X%h;\n",dcache_req_op,dcache_req_rd,dcache_req_addr);/*LB*/
	         4'b0_001:	$display("mem_access.v: processor made an LH request: type: %b; Destination: R%d; address: 0X%h;\n",dcache_req_op,dcache_req_rd,dcache_req_addr);/*LH*/
	         4'b0_010:	$display("mem_access.v: processor made an LW request: type: %b; Destination: R%d; address: 0X%h;\n",dcache_req_op,dcache_req_rd,dcache_req_addr);/*LW*/
	         4'b0_100:	$display("mem_access.v: processor made an LBU request: type: %b; Destination: R%d; address: 0X%h;\n",dcache_req_op,dcache_req_rd,dcache_req_addr);/*LBU*/
	         4'b0_101:	$display("mem_access.v: processor made an LHU request: type: %b; Destination: R%d; address: 0X%h;\n",dcache_req_op,dcache_req_rd,dcache_req_addr);/*LHU*/
	         4'b0_110:	$display("mem_access.v: processor made an LWU request: type: %b; Destination: R%d; address: 0X%h;\n",dcache_req_op,dcache_req_rd,dcache_req_addr);/*LWU*/
	         4'b0_011:	$display("mem_access.v: processor made an LD request: type: %b; Destination: R%d; address: 0X%h;\n",dcache_req_op,dcache_req_rd,dcache_req_addr);/*LD*/

	         default:$display("\n");
	       endcase
        end
   end

   always@(posedge clk)begin
      if(dcache_req_retry)
        begin
	       $display("mem_access.v: testbench ask to retry!!!\n");
        end
   end


`endif



endmodule
