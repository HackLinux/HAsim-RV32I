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

Author: Michael Pellauer, Nirav Dave
*/

import RegFile::*;
import BRAM::*;
import BRAMInitiatorWires::*;
import GetPut::*;
import FIFO::*;
import FIFOF::*;

import Types::*;
import Interfaces::*;
import DebugFlags::*;

import BRAMInitiator::*;


interface PPC;
  interface BRAMInitiatorWires#(Bit#(14))  bramInitiatorWires;
endinterface


(* synthesize *)
module mkPPCModel(PPC);
  
  RegFile#(Bit#(16), Bit#(64)) prog <- mkRegFileFullLoad("program.hex");

  Reg#(Bit#(16)) counter <- mkReg(0);
  
  // Data is held in 2 addr blocks. as
  // Addr n : 1---------22222  <- 1 valid bit (otherwise all zero) 
  //                              2 top bits of payload
  //    n+1 : 333333333333333  <- 3 rest of payload                     
 
  //State

  BRAMInitiator#(Bit#(14)) bramInit <- mkBRAMInitiator;
  let bram = bramInit.bram;
  
  //BRAM#(Bit#(16), Bit#(32)) bram <- mkBRAM_Full();
  
  FIFO#(PPCMessage)  ppcMesgQ <- mkFIFO();
  FIFOF#(Instruction) ppcInstQ <- mkFIFOF();
  
  let minWritePtr  =   0;
  let maxWritePtr  =  31;
  let minReadPtr   =  32;
  let maxReadPtr   =  63;
  
  Reg#(Bit#(14))  readPtr <- mkReg(minReadPtr); 

  Reg#(Bit#(14)) writePtr <- mkReg(minWritePtr);
  
  Reg#(FeederState) state <- mkReg(FI_InStartCheckRead);
  
  Reg#(Bit#(32)) partialRead  <- mkReg(0);
  Reg#(Bit#(32)) partialWrite <- mkReg(0);

  let debugF = debug(ppcDebug);
  
  ///////////////////////////////////////////////////////////  
  // In goes to FPGA, Out goes back to PPC
  ///////////////////////////////////////////////////////////

  rule inStartCheckRead( state == FI_InStartCheckRead);
    debugF($display("PPC: StartCheckRead %h", readPtr));
    bram.read_req(readPtr);
    state <= FI_InStartRead;
  endrule   
  
  rule inStartRead( state == FI_InStartRead);
    let v <- bram.read_resp();
    Bool valid = (v[31] == 1);
    state <= (valid) ? FI_InStartTake : ((ppcInstQ.notEmpty) ? FI_OutStartCheckWrite : FI_InStartCheckRead);  
    partialRead <= v; // record extra bits in case we need it
    debugF($display("PPC: StartRead %h", v));
    if (valid)
      begin
	$display("PPC: read: [%d] = %h",readPtr,v);
	bram.read_req(readPtr+1); 
      end
  endrule

  rule inStartTake( state == FI_InStartTake);
    debugF($display("PPC: StartTake"));
    let val <- bram.read_resp();
    $display("PPC: read: [%d] = %h",readPtr+1,val);
    ppcMesgQ.enq(unpack(truncate({partialRead,val}))); // get bottom (n < 63) bits
    bram.write(readPtr, 0);
    readPtr <= (readPtr + 2 > maxReadPtr) ? minReadPtr : (readPtr + 2);
    state <= ((ppcInstQ.notEmpty) ? FI_OutStartCheckWrite : FI_InStartCheckRead);
  endrule    
    
  rule inStartCheckWrite( state == FI_OutStartCheckWrite);
    debugF($display("PPC: StartCheckWrite %h", writePtr));
    bram.read_req(writePtr);
    state <= FI_OutStartWrite;
  endrule
    
  rule inStartWrite(state == FI_OutStartWrite);
    debugF($display("PPC: StartWrite"));
    let v <- bram.read_resp();
    Bool valid = (v[31] == 0);
    state <= (valid) ? FI_OutStartPush : FI_InStartCheckRead;  
    let pInst = pack(ppcInstQ.first);
    if (valid) begin
      
      partialWrite <= truncate (pInst >> 32);
      Bit#(32) fstwrite = {1'b1,truncate (pInst)};
//      $display("PPC:fstinst [%d] = %h",writePtr+1, fstwrite);
      bram.write(writePtr+1, fstwrite); 
      ppcInstQ.deq();
    end
  endrule  

  rule inStartPush( state == FI_OutStartPush);
    debugF($display("PPC: StartPush"));

    Bit#(32) sndwrite = {1'b1,truncate(partialWrite)};
//    $display("PPC:sndinst [%d] = %h", writePtr, sndwrite);
    bram.write(writePtr, sndwrite); // write top n - 32 bits + valid
    writePtr <= (writePtr + 2 > maxWritePtr) ? minWritePtr : (writePtr + 2);
    state <= FI_InStartCheckRead;
  endrule 

  rule nextInst2(counter == maxBound);
    $display("Uhoh, ran out of instructions");

  endrule


  rule nextInst(counter < maxBound);
    let inst = prog.sub(counter);
    if (inst != 64'hAAAA_AAAA_AAAA_AAAA)
      begin
	ppcInstQ.enq(unpack(truncate(inst)));
	counter <= counter + 1;
      end
  endrule

  rule nextMesq(True);
    ppcMesgQ.deq();
    $display("PPC: MESG TO PPC: %h", ppcMesgQ.first);
  endrule
  
  
  //Interface  



  interface bramInitiatorWires = bramInit.bramInitiatorWires;
  
  
endmodule







/*
module mkPPCModel#(BRAMTargetWires bram) ();

  Reg#(Bit#(10)) counter <- mkReg(0);
  Reg#(Bit#(9)) iBufHead <- mkReg(0);
  Reg#(Bit#(9)) iBufTail <- mkReg(0);
  
  RegFile#(Bit#(10), Bit#(64)) prog <- mkRegFileLoad("program.hex", 0, 1023);
    
  Reg#(Bit#(32)) next <- mkRegU;
  Reg#(Bool) allIssued <- mkReg(False);
  Reg#(Bit#(8)) holdTime <- mkReg(0);
    
  Stmt issue =
  seq
    //Hold soft reset.
    bram.bramIN(32'b00000000000000000010000011100000, 32'hffffffff, 4'b0100);
    while (holdTime < `HOLD_TIME)
      holdTime <= holdTime + 1;
    //No longer hold soft res,et.
    bram.bramIN(32'b00000000000000000010000011100000, 0, 4'b0100);
    while (!allIssued) seq
      //Write the first half on an Inst
      action
	match {.i1, .i2} = splitInst(unpack(truncate(prog.sub(counter))));
	bram.bramIN(0, i1, 1);
	next <= i2;
	counter <= counter + 1;
	if (((counter + 1) == 0) || (prog.sub(counter) == 64'haaaaaaaa)) allIssued <= True;
	
      endaction
      //Advance IBufHead
      action
        bram.bramIN(0, zeroExtend(iBufHead), 4'b0100);
        iBufHead <= iBufHead + 1;
      endaction
      //Write the second half of an Inst
      bram.bramIN(0, next, 1);
      //Advance IBufHead
      action
        bram.bramIN(0, zeroExtend(iBufHead), 1);
        iBufHead <= iBufHead + 1;
      endaction
      //Perhaps we should read from the PPC at some point
    endseq
  endseq;
   
  FSM issue_fsm <- mkFSM(issue);
  
  rule beginToIssuE (issue_fsm.done);
  
    issue_fsm.start();
  endrule
  
endmodule
*/