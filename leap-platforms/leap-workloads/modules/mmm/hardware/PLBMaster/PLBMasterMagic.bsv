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

Author: Kermin Fleming
*/

// Global Imports
import GetPut::*;
import FIFO::*;
import FIFOLevel::*;
import FIFOF::*;
import RegFile::*;
import DefaultValue::*;

// Project Imports


`include "awb/provides/mmm_common.bsh"

`include "awb/provides/librl_bsv.bsh"
`include "awb/provides/soft_connections.bsh"
`include "awb/provides/soft_services.bsh"
`include "awb/provides/soft_services_lib.bsh"
`include "awb/provides/soft_services_deps.bsh"
`include "awb/provides/mem_services.bsh"
`include "awb/provides/common_services.bsh"
`include "awb/provides/scratchpad_memory_common.bsh"
`include "awb/dict/VDEV_SCRATCH.bsh"
`include "awb/provides/dynamic_parameters_service.bsh"
`include "awb/dict/PARAMS_MMM_MEMORY_UNIT.bsh"
 

typedef enum
{
  Test,
  Sleep,
  Finish
} TestState
    deriving(Bits, Eq);

typedef enum
{
  A,
  B,
  C,
  Scratch
} MatrixOrder
    deriving(Bits, Eq);


module [CONNECTED_MODULE] mkPLBMasterMagic (PLBMaster);

  // some debug
  STDIO#(Bit#(32))     stdio <- mkStdIO();

  PARAMETER_NODE paramNode <- mkDynamicParameterNode();	
  Param#(8) maxOps <- mkDynamicParameter(`PARAMS_MMM_MEMORY_UNIT_OPS_INFLIGHT,paramNode);

  // state for the actual magic memory hardware
  FIFO#(ComplexWord)       wordInfifo <- mkFIFO();
  FIFOF#(ComplexWord)       wordOutfifo <- mkSizedFIFOF(64);
  FIFOF#(ComplexWord)       wordTokens  <- mkSizedFIFOF(64);
  FIFO#(PLBMasterCommand)  plbMasterCommandInfifo <- mkFIFO(); 

  SCRATCHPAD_CONFIG sconf = defaultValue;
  sconf.requestMerging = True;

  MEMORY_IFC#(Bit#(20), ComplexWord) matrixA       <- mkScratchpad(`VDEV_SCRATCH_MATRIXA, sconf);
  MEMORY_IFC#(Bit#(20), ComplexWord) matrixB       <- mkScratchpad(`VDEV_SCRATCH_MATRIXB, sconf);
  MEMORY_IFC#(Bit#(20), ComplexWord) matrixC       <- mkScratchpad(`VDEV_SCRATCH_MATRIXC, sconf);
  MEMORY_IFC#(Bit#(20), ComplexWord) matrixScratch <- mkScratchpad(`VDEV_SCRATCH_SCRATCH, sconf);

  FIFOCountIfc#(MatrixOrder, 64) matrixOrder <- mkFIFOCount();

  Reg#(Bit#(LogBlockElements)) elementCounter <- mkReg(0);
  Reg#(Bit#(LogBlockSize))     rowCounter <- mkReg(0);  // 0 -> blocksize
  Reg#(Bit#(LogRowSize))       rowOffset <- mkReg(0);   
  Reg#(BlockAddr)             addressOffset <- mkReg(0);       
  

  Reg#(Bit#(32))                   totalTicks <- mkReg(0);
  Reg#(Bit#(32))                   readsA <- mkReg(0);
  Reg#(Bit#(32))                   readsB <- mkReg(0);
  Reg#(Bit#(32))                   readsC <- mkReg(0);
  Reg#(Bit#(32))                   readsS <- mkReg(0);
  Reg#(Bit#(32))                   respsA <- mkReg(0);
  Reg#(Bit#(32))                   respsB <- mkReg(0);
  Reg#(Bit#(32))                   respsC <- mkReg(0);
  Reg#(Bit#(32))                   respsS <- mkReg(0);
  Reg#(Bit#(32))                   writesC <- mkReg(0);
  Reg#(Bit#(32))                   writesS <- mkReg(0);

  let statusMsg <-  getGlobalStringUID("readsA:%d:readsB:%d:readsC:%d:readsS:%d:writesC:%d:writesS:%d\n");
  let statusMsg2 <-  getGlobalStringUID("respsA:%d:respsB:%d:respsC:%d:respsS:%d\n");


  rule tick(True);
    totalTicks <= totalTicks +1;
    if(totalTicks[29:0] == 0)
    begin
        stdio.printf(statusMsg, list6(readsA, readsB, readsC, readsS, writesC, writesS));
    end
    else if(totalTicks[29:0] == 128)
    begin
        stdio.printf(statusMsg2, list4(respsA, respsB, respsC, respsS));
    end
  endrule

  rule rowSize(plbMasterCommandInfifo.first() matches tagged RowSize .rs);   
    debug(plbMasterDebug, $display("PLBMaster: processing RowSize command %d", rs));
    rowOffset <= rs;
    plbMasterCommandInfifo.deq();
  endrule
                          
  rule loadPage(plbMasterCommandInfifo.first() matches tagged LoadPage .ba &&& matrixOrder.count < unpack(truncate(maxOps)));
    $display("LoadPage %h element %d", ba, elementCounter);
    elementCounter <= elementCounter + 1;
    if(elementCounter == 0)
      begin
        debug(plbMasterDebug, $display("PLBMaster: processing LoadPage command"));
      end
    if(elementCounter + 1 == 0)
       begin
         debug(plbMasterDebug, $display("PLBMaster: finished LoadPage command"));
	 addressOffset <= 0;
	 rowCounter    <= 0;
         plbMasterCommandInfifo.deq();
       end  
    else if(rowCounter + 1 == 0)
      begin //When we get to the end of a row, we need to reset by
	    //shifting the Address Offset to 1 row higher =
	rowCounter <= 0;
        addressOffset <= addressOffset + 1 - unpack(fromInteger(1*valueof(BlockSize))) + (1 << rowOffset);
      end
    else
      begin
        addressOffset <= addressOffset + 1;
        rowCounter <= rowCounter + 1;
      end
                             
     BlockAddr addr = ba + addressOffset;
     // Now that we're done with calculating the address,
     // we can case out our memory space
     //case (addr[23:22]) 
     //  2'b00:  begin $display("PLB: reading matA[%h] => %h"   ,addr[21:2], matrixA.sub(addr[21:2])); end
     //  2'b01:  begin $display("PLB: reading matB[%h] => %h"   ,addr[21:2], matrixB.sub(addr[21:2])); end
     //  2'b10:  begin $display("PLB: reading matC[%h] => %h"   ,addr[21:2], matrixC.sub(addr[21:2])); end
     //  2'b11:  begin $display("PLB: reading scratch[%h] => %h",addr[21:2], matrixScratch.sub(addr[21:2])); end
     //endcase
     wordTokens.enq(?);
     case (addr[21:20]) 
       2'b00:  begin 
                   matrixA.readReq(addr[19:0]);  
                   matrixOrder.enq(A);
		   readsA <= readsA + 1;
               end

       2'b01:  begin
                   matrixB.readReq(addr[19:0]);
                   matrixOrder.enq(B);
		   readsB <= readsB + 1;
               end
       2'b10:  begin
                   matrixC.readReq(addr[19:0]);
                   matrixOrder.enq(C);
		   readsC <= readsC + 1;
               end
       2'b11:  begin
                   matrixScratch.readReq(addr[19:0]);
                   matrixOrder.enq(Scratch);
		   readsS <= readsS + 1;
               end
     endcase

  endrule
  
  rule wordA(matrixOrder.first == A);
      let data <- matrixA.readRsp;
      wordOutfifo.enq(data);
      matrixOrder.deq;
      respsA <= respsA + 1;
  endrule 

  rule wordB(matrixOrder.first == B);
      let data <- matrixB.readRsp;
      wordOutfifo.enq(data);
      matrixOrder.deq;
      respsB <= respsB + 1;
  endrule 

  rule wordC(matrixOrder.first == C);
      let data <- matrixC.readRsp;
      wordOutfifo.enq(data);
      matrixOrder.deq;
      respsC <= respsC + 1;
  endrule 

  rule wordScratch(matrixOrder.first == Scratch);
      let data <- matrixScratch.readRsp;
      wordOutfifo.enq(data);
      matrixOrder.deq;
      respsS <= respsS + 1;
  endrule 

  rule storePage(plbMasterCommandInfifo.first() matches tagged StorePage .ba);
    elementCounter <= elementCounter + 1;
    if(elementCounter == 0)
      begin
        debug(plbMasterDebug, $display("PLBMaster: processing StorePage command"));
      end
    if(elementCounter + 1 == 0)
      begin
        debug(plbMasterDebug, $display("PLBMaster: finished StorePage command"));
        addressOffset <= 0;
	rowCounter    <= 0;
        plbMasterCommandInfifo.deq();
      end  
    else if(rowCounter + 1 == 0)
      begin 
        addressOffset <= addressOffset + 1 - unpack(fromInteger(valueof(BlockSize))) + (1 << rowOffset);
        rowCounter <= 0;
      end
    else
      begin
        addressOffset <= addressOffset + 1;
	rowCounter <= rowCounter + 1;
      end
                            
    BlockAddr addr = ba + addressOffset;
    // Now that we're done with calculating the address,
    // we can case out our memory space
    case (addr[21:20])
      2'b00:  begin
		debug(plbMasterDebug,$display("PLB: writing to matA %h",addr[19:0]));
		matrixA.write(addr[19:0],wordInfifo.first());
	      end
      2'b01:  begin
		debug(plbMasterDebug,$display("PLB: writing to matB %h",addr[19:0]));
		matrixB.write(addr[19:0],wordInfifo.first());
	      end
      2'b10:  begin
		debug(plbMasterDebug,$display("PLB: writing to matC %h",addr[19:0]));
		matrixC.write(addr[19:0],wordInfifo.first());	
		writesC <= writesC + 1;	
	      end
      2'b11:  begin
		debug(plbMasterDebug,$display("PLB: writing to scratch %h",addr[19:0]));
		matrixScratch.write(addr[19:0],wordInfifo.first());
		writesS <= writesS + 1;	
	      end
    endcase
    wordInfifo.deq();
  endrule

  rule debugRule (True);
    case (plbMasterCommandInfifo.first()) matches
        tagged LoadPage .i: noAction;
        tagged StorePage .i: noAction;
        tagged RowSize .sz: noAction;
        default:
          $display("PLBMaster: illegal command: %h", plbMasterCommandInfifo.first());
    endcase

  endrule

  interface Put wordInput = interface Put;
    method Action put(x);
      wordInfifo.enq(x);
      //$display("PLB: got val %h", x);
    endmethod
  endinterface;

  interface Get wordOutput = interface Get;
	method get();
	  actionvalue
            //$display("PLB: sending val %h", wordOutfifo.first());                        
	    wordOutfifo.deq();
	    wordTokens.deq();
	    return wordOutfifo.first();
	  endactionvalue
	endmethod
      endinterface;
  interface Put plbMasterCommandInput = fifoToPut(plbMasterCommandInfifo);

endmodule
