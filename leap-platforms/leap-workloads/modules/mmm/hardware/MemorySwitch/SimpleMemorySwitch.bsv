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

// Global imports
import GetPut::*;
import FIFO::*;
import Vector::*;

// Local imports 
`include "awb/provides/librl_bsv.bsh"
`include "awb/provides/soft_connections.bsh"
`include "awb/provides/soft_services.bsh"
`include "awb/provides/soft_services_lib.bsh"
`include "awb/provides/soft_services_deps.bsh"
`include "awb/provides/mmm_common.bsh"


module [CONNECTED_MODULE] mkSimpleMemorySwitch (MemorySwitch);

  CONNECTION_SEND#(Bit#(1)) sync <- mkConnectionSend("Sync");

  FIFO#(ComplexWord)             plbMasterComplexWordInfifo <- mkFIFO();
  FIFO#(ComplexWord)             plbMasterComplexWordOutfifo <- mkFIFO();

  Reg#(PPCMessage)               ppcMessageVal  <- mkReg(1);

  FIFO#(MemorySwitchCommand)     memorySwitchCommandInfifo <- mkFIFO();
  Vector#(FunctionalUnitNumber,
          FIFO#(ComplexWord))    functionalUnitComplexWordOutfifos <- replicateM(mkFIFO());
  Vector#(FunctionalUnitNumber,
          FIFO#(ComplexWord))    functionalUnitComplexWordInfifos <- replicateM(mkFIFO());
  
  Reg#(Bit#(LogBlockElements))   elementCounter <- mkReg(0);
  
  rule routeStore(memorySwitchCommandInfifo.first() matches tagged StoreFromFU .fu);
    // Since the block size is always a power of two, Overflow on the counter means 
    // we deque the command as we have finished
    if(elementCounter == 0)
      begin
        debug(memorySwitchDebug, $display("memorySwitch: processing StoreFromFU"));
      end

    if(elementCounter + 1 == 0)
      begin
        debug(memorySwitchDebug, $display("memorySwitch: finished processing StoreFromFU"));
        //What is this doing?
        sync.send(1);
        ppcMessageVal <= ppcMessageVal+1;
        memorySwitchCommandInfifo.deq();
      end
    elementCounter <= elementCounter + 1;
    plbMasterComplexWordOutfifo.enq(
    functionalUnitComplexWordInfifos[fu].first());
    functionalUnitComplexWordInfifos[fu].deq();
  endrule

  rule routeLoad(memorySwitchCommandInfifo.first() matches tagged LoadToFUs .fus);
    // Since the block size is always a power of two, Overflow on the counter means
    // we deque the command as we have finished
    if(elementCounter == 0)
      begin
        debug(memorySwitchDebug, $display("memorySwitch: processing LoadToFUs"));
      end

    if(elementCounter + 1 == 0)
      begin
        debug(memorySwitchDebug, $display("memorySwitch: finished processing LoadToFUs"));
        memorySwitchCommandInfifo.deq();
      end
    elementCounter <= elementCounter + 1;
    for (Integer x = 0; x < valueof(FunctionalUnitNumber); x = x + 1)
      if (fus[x] == 1)
        functionalUnitComplexWordOutfifos[x].enq(plbMasterComplexWordInfifo.first());
    //$display("SWITCH: %h", plbMasterComplexWordInfifo.first());
    plbMasterComplexWordInfifo.deq();
  endrule


  interface Put plbMasterComplexWordInput = fifoToPut(plbMasterComplexWordInfifo) ;
  interface Get plbMasterComplexWordOutput = fifoToGet(plbMasterComplexWordOutfifo);
  interface Put memorySwitchCommandInput = fifoToPut(memorySwitchCommandInfifo);
  interface functionalUnitComplexWordOutputs = map(fifoToGet,functionalUnitComplexWordOutfifos); 
  interface functionalUnitComplexWordInputs = map(fifoToPut,functionalUnitComplexWordInfifos); 

endmodule
