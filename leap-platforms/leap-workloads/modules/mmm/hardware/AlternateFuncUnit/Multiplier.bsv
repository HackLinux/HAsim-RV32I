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

Author: Muralidaran Vijayaraghavan
*/

import Interfaces::*;

import Types::*;
import FuncUnit::*;
import FIFO::*;
import FIFOF::*;
import GetPut::*; 
import PLBMaster::*;
import BRAMFeeder::*;
import Connectable::*;
import PLBMasterWires::*;
import BRAMInitiatorWires::*;

(* synthesize *)
module mkMultiplierTop(Multiplier);
    FuncUnit fu <- mkfunctionalunit();

    PLBMaster plb     <- mkPLBMaster();
    Feeder feeder     <- mkBRAMFeeder();

    Reg#(Bit#(32)) counter <- mkReg(1);

    FIFOF#(Data) dataInFIFO  <- mkFIFOF();
    FIFOF#(Data) dataOutFIFO <- mkFIFOF();
    FIFOF#(Instruction) instFIFO <- mkFIFOF();

    let inst = instFIFO.first();
    let fuOp = !(inst.op == SetRowSize || inst.op == LoadAddr || inst.op == StoreAddr);

    rule processInst(True);
        let instruction <- feeder.ppcInstructionOutput.get();
        $display("FU: got instruction %x %d", instruction.addr, instruction.op);
        instFIFO.enq(instruction);
    endrule

    rule processFU_PutData(True);
        let data <- plb.wordOutput.get();
        fu.putData.put(data);
    endrule

    rule processFU_GetData(True);
        let data <- fu.getData.get();
        plb.wordInput.put(data);
        $display("FU: FU to PLB");
    endrule

    rule processFUOp(fuOp);
        fu.putInst.put(inst.op);
        instFIFO.deq();
        $display("FU: got FU OP %d", inst.op);
    endrule

    rule processSetRowSize(inst.op == SetRowSize);
        plb.plbMasterCommandInput.put(tagged RowSize truncate(inst.addr));
        instFIFO.deq();
        $display("FU: got Set Row Size %d", inst.addr);
    endrule

    rule processLoadAddr(inst.op == LoadAddr);
        plb.plbMasterCommandInput.put(tagged LoadPage inst.addr);
        instFIFO.deq();
        $display("FU: got Load Addr %x", inst.addr);
    endrule

    rule processStoreAddr(inst.op == StoreAddr);
        plb.plbMasterCommandInput.put(tagged StorePage inst.addr);
        instFIFO.deq();
        $display("FU: got Store Addr %x", inst.addr);
    endrule

    rule processZeroReturn(True);
        let zero <- fu.getZero.get();
    endrule

    rule processLoadReturn(True);
        let load <- fu.getLoad.get();
    endrule

    rule processLoadMulReturn(True);
        let loadMul <- fu.getLoadMul.get();
    endrule

    rule processMulReturn(True);
        let mul <- fu.getMul.get();
    endrule

    rule processStoreReturn(True);
        let store <- fu.getStore.get();
        counter <= counter + 1;
        feeder.ppcMessageInput.put(counter);
    endrule

    interface plbMasterWires     = plb.plbMasterWires;
    interface bramInitiatorWires = feeder.bramInitiatorWires;
endmodule
