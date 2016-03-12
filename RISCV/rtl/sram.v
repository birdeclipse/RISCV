//==============================================================================
//      File:           sram.v
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

module sram#(
	         parameter RAM_WIDTH  = 1,
	         parameter RAM_DEPTH  = 6,
	         parameter ADDR_WIDTH = 64
	         )
   (
    input                  clk,
    input                  reset,
    input                  we,
    input [RAM_DEPTH-1:0]  addr,
    input [RAM_WIDTH-1:0]  d,
    output [RAM_WIDTH-1:0] q
    );

   reg [RAM_WIDTH-1:0]     sram [ADDR_WIDTH-1:0];
   reg [RAM_WIDTH-1:0]     sram_out = {RAM_WIDTH{1'b0}};
   reg [RAM_DEPTH-1:0]     reset_counter = 'd0;


   always @(posedge clk)begin
      if(reset)
	    begin
	       sram[reset_counter] <= 'd0;
	       reset_counter	<= reset_counter+1;
	    end
      else
	    begin
	       if(we)
	         sram[addr] <= d;
	    end
   end

   always@(*)begin
      sram_out	=	sram[addr];
   end

   assign	q = sram_out;

endmodule
