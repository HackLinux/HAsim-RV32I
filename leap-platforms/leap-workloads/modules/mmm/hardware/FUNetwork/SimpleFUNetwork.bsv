/*
Copyright (c) 2007 MIT

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

Author: Michael Pellauer
*/

import GetPut::*;
import Vector::*;
import FIFO::*;

`include "awb/provides/soft_connections.bsh"
`include "awb/provides/soft_services.bsh"
`include "awb/provides/soft_services_lib.bsh"
`include "awb/provides/soft_services_deps.bsh"

`include "awb/provides/mmm_common.bsh"

//Simple network, a switch and FIFOs.
//Only does one active forward at a time.
//Lots of room for improvement.


module mkSimpleNetwork#(Vector#(FunctionalUnitNumber, FUNetworkLink) fus) (FUNetwork);

  FIFO#(FUNetworkCommand) insQ <- mkFIFO();
  
  Reg#(Bit#(LogBlockElements)) count <- mkReg(0);
  Reg#(Bool) transferring <- mkReg(False);
  Reg#(FunctionalUnitAddr) srcFU <- mkRegU();
  Reg#(FunctionalUnitMask) dstFUs <- mkRegU();
  Reg#(MatrixRegister) srcR <- mkRegU();
  Reg#(MatrixRegister) dstR <- mkRegU();
 
  rule initiateTransfer (!transferring);
  
    let cmd = insQ.first();
    insQ.deq();
    
    transferring <= True;
    srcFU <= cmd.fuSrc;
    dstFUs <= cmd.fuDests;
    srcR <= cmd.regSrc;
    dstR <= cmd.regDest;
    if(valueof(FunctionalUnitNumber) > 1)
      begin
        $display("FUNetwork: initXfer from: %d to: %h", cmd.fuSrc, cmd.fuDests);    
      end
    else
      begin
        $display("FUNetwork: initXfer from: %d to: %h", 0, cmd.fuDests); 
      end
  endrule
 
  
  rule doTransfer (transferring);
  
    $display("FUNetwork: xfer from %d.%s to %d.%s", srcFU, showReg(srcR), dstFUs, showReg(dstR));

    //Get the packet from the srcQ
  
    let fu_src = fus[srcFU];
    
    let srcQ = case (srcR) matches
                 tagged A:
	           return fu_src.a_out;
		 tagged B:
		   return fu_src.b_out;
		 tagged C:
		   return fu_src.c_out;
	       endcase;
  

    let pkt <- srcQ.get();
   
    //Put it into all the dests.
   
    for (Integer x = 0; x < valueof(FunctionalUnitNumber); x = x + 1)
    begin

      let dstQ = case (dstR) matches
                   tagged A:
	             return fus[x].a_in;
		   tagged B:
		     return fus[x].b_in;
		   tagged C:
		     return fus[x].c_in;
		 endcase;

      if (dstFUs[x] == 1)
        dstQ.put(pkt);
    end
    
    let newcount = count + 1;
    
    count <= newcount;
    
     if (newcount == 0)
	begin
	   transferring <= False;
         if(valueof(FunctionalUnitNumber) > 1)
           begin  
	     $display("FUNetwork: doneXfer from: %d to: %h", srcFU, dstFUs);
           end
         else
           begin  
	     $display("FUNetwork: doneXfer from: %d to: %h", 0, dstFUs);
           end
	end

  endrule
  
  interface fuNetworkCommandInput = fifoToPut(insQ);
    

endmodule
