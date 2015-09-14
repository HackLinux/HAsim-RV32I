/*
Copyright (c) 2009 MIT

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

Author: Myron King
*/

import MemTypes::*;
import PLBMasterWires::*;
import PLBMaster::*;
import PLBMasterDefaultParameters::*;
import FIFO::*;
import GetPut::*;
import mkCacheTesterCore::*;
import BRAMFeeder::*;
import PLBMasterEmulator::*;
import RegFile::*;
import ConfigReg::*;

// number of BusWords in memory
typedef (TExp#(22)) MemSz;
typedef (TExp#(12)) MemRdSz;

typedef enum{
   Init ,
   Running
   } FeederState deriving (Bits,Eq);

module mkBRAMFeederDummy(Feeder);
   
   Reg#(FeederState)  stage <- mkConfigReg(Init);
   FIFO#(PPCMessage)  hw_to_ppc <- mkFIFO();
   FIFO#(PPCMessage)  ppc_to_hw <- mkFIFO();
   let count <- mkReg(fromInteger(valueof(NumTests)));
   
   rule startA(stage == Init);
      ppc_to_hw.enq(count);
      stage <= Running;
   endrule

   rule finishA(stage == Running);
      hw_to_ppc.deq();
      let rv = hw_to_ppc.first; 
      if(rv != 1)
         begin 
            $display("FAIL: %h",rv);
            $finish(0);
         end
      else
	 if (count==0)
	    begin
	       $display("PASS");
	       $finish(0);
	    end
	 else
	    begin
	       stage <= Init;
	       let t <- $time;
	       $display("intermediate pass %h", t/10);
	       count <= count - 1;
	    end
   endrule
   
   interface ppcMessageInput    = fifoToPut(hw_to_ppc);
   interface ppcMessageOutput   = fifoToGet(ppc_to_hw);
   interface bramInitiatorWires = ?;
      
endmodule

(* synthesize *)
module mkCacheTesterSim(Empty);
   RegFile#(Bit#(TLog#(MemSz)),BusWord) simMemoryInit = 
   interface RegFile;
      method Action upd(Bit#(TLog#(MemSz)) addr, BusWord word);
         $display("Calling upd? Srsly? WTF?");
         $finish;
      endmethod
      method BusWord sub(Bit#(TLog#(MemSz)) addr);
	 Bit#(TDiv#(SizeOf#(BusWord),2)) hi = zeroExtend({addr>>1,1'b1}); 
	 Bit#(TDiv#(SizeOf#(BusWord),2)) lo = zeroExtend({addr>>1,1'b0});
	 return  {lo,hi};
      endmethod
   endinterface;
   
   PLBMasterEmulator#(MemSz) plbMasterEmulator <- mkPLBMasterEmulator(simMemoryInit);
   Feeder                               feeder <- mkBRAMFeederDummy;
   mkCacheTesterCore(feeder, plbMasterEmulator.plbmaster);
      
endmodule