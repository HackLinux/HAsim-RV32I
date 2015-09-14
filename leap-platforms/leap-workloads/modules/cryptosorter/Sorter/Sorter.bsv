/*
Copyright (c) 2008 MIT

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

Author: Kermin Fleming
*/

import FIFO::*;
import GetPut::*;
import LFSR::*;
import Vector::*;

// Local Imports
`include "asim/provides/librl_bsv.bsh"
`include "awb/provides/soft_connections.bsh"
`include "awb/provides/soft_services.bsh"
`include "awb/provides/soft_services_lib.bsh"
`include "awb/provides/soft_services_deps.bsh"
`include "awb/provides/cryptosorter_common.bsh"
`include "awb/provides/cryptosorter_control.bsh"
`include "awb/provides/cryptosorter_sort_tree.bsh"
`include "awb/provides/cryptosorter_memory_wrapper.bsh"
`include "awb/rrr/remote_server_stub_CRYPTOSORTERCONTROLRRR.bsh"


typedef enum {
  Init, 
  Idle,
  Waiting
} TopState deriving (Bits,Eq);

module [CONNECTED_MODULE] mkConnectedApplication (Empty);

  ServerStub_CRYPTOSORTERCONTROLRRR serverStub <- mkServerStub_CRYPTOSORTERCONTROLRRR();
  ExternalMemory extMem <- mkExternalMemory();
  Control  controller <- mkControl(extMem);
  Reg#(Bit#(2)) style <- mkReg(0);  
  Reg#(Bit#(5)) size <- mkReg(0);
  Reg#(Bit#(3)) passes <- mkReg(1);
  Reg#(TopState) state <- mkReg(Idle);
  Reg#(Bit#(40)) counter <- mkReg(0);
  Reg#(Bit#(32)) initCtrl <- mkReg(0);
  Reg#(Bit#(32)) initData    <- mkReg(0);
  LFSR#(Bit#(32)) lfsr <- mkLFSR_32(); 

  rule getfinished((state == Waiting) && controller.finished);
    state <= Idle;
  endrule

  rule countUp(state == Waiting);
    counter <= counter + 1;
  endrule

  rule dropCountReq(state != Idle);
     let inst <- serverStub.acceptRequest_ReadCycleCount();       
     serverStub.sendResponse_ReadCycleCount(0,?);
  endrule

  rule readCount(state == Idle);
     let inst <- serverStub.acceptRequest_ReadCycleCount();       
     serverStub.sendResponse_ReadCycleCount(1,zeroExtend(counter));
  endrule

  rule sendCommand(controller.finished && (state == Idle));    
    let inst <- serverStub.acceptRequest_PutInstruction();       
    serverStub.sendResponse_PutInstruction(?);
    size <= truncate(pack(inst.size));
    style <= truncate(pack(inst.style));
    state <= Init;
    counter <= 0;
    passes <= 1;
    initCtrl <= 0;
    initData <= 0;
    lfsr.seed(inst.seed);
  endrule

  rule doInitCtrl(state == Init && initCtrl < 1<<size);
     initCtrl <= initCtrl + 1;
     Bit#(TLog#(RecordsPerBlock)) burstCount = truncate(initCtrl);
     if(burstCount == 0)
     begin
        extMem.write.writeReq(truncate(initCtrl << 2)); // This shift comes because of the interface expected by the sort tree.
     end
  endrule      

  rule doInitData(state == Init && initData < 1<<size);
     initData <= initData + 1;

     let data =   case (style) matches 
                      0: return 0;
		      1: return initData;
		      2: return maxBound - initData;
		      3: return lfsr.value();     
     endcase;

     lfsr.next;

     extMem.write.write(pack(replicate(data)));
  endrule      

  rule start(state == Init && initData == 1 << size && 
             initCtrl == 1<<size && !extMem.writesPending());

     state <= Waiting;
     controller.doSort(size);
  endrule

  rule returnPass;
    let msg <- controller.msgs.get;
    passes <= passes + 1;
  endrule  

endmodule