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

import FIFO::*;
import GetPut::*;


`include "Common.bsv"
import BRAMTargetWires::*;

interface FeederAndBRAM;

  method Action enq(Bit#(32) data);
  method Action deq();
  method Bit#(32) first();
  interface BRAMTargetWires bramTargetWires;
endinterface

(* synthesize *)
module mkPPCFeeder#() (Feeder);

  FeederAndBRAM prim_feeder <- mkPrimFeeder();
  Reg#(Maybe#(Bit#(32))) last <- mkReg(Invalid);
  
  rule preBuffer (!isValid(last));
  
    //Unmarshall Bit 32 into Instruction.
    last <= tagged Valid prim_feeder.first();
    prim_feeder.deq();
  
  endrule
  
  interface Get ppcInstructionOutput;
  
    method ActionValue#(Instruction) get() if (isValid(last));

      let part2 = prim_feeder.first();
      prim_feeder.deq();
      last <= tagged Invalid;
      return fuseInst(validValue(last), part2);

    endmethod

  endinterface
  
  interface Put ppcMessageInput;

    method Action put(PPCMessage msg);

      prim_feeder.enq(msg);

    endmethod

  endinterface

  interface BRAMTargetWires bramTargetWires = prim_feeder.bramTargetWires;
endmodule

//Import the reference HW into BSV as a Primitive FIFO

import "BVI" refhw =
  module mkPrimFeeder#(Clock ocmClock, Reset ocmReset) (FeederAndBRAM);
    default_clock clk(Clk);
    default_reset rst(Rst);
    input_clock(BRAM_Clk_A) = ocmClock;
    input_reset(BRAM_Rst_A) = ocmReset;
    method enq(FIFO_TO_PPC_Enq_data) enable(FIFO_TO_PPC_Enq_EN) ready(FIFO_TO_PPC_Enq_RDY);
    method deq() enable(FIFO_FROM_PPC_Deq_EN) ready(FIFO_FROM_PPC_Deq_RDY);
    method (* reg *)FIFO_FROM_PPC_First first ready(FIFO_FROM_PPC_First_RDY);
    
    interface BRAMTargetWires bramTargetWires;
      method bramIN(BRAM_Addr_A, BRAM_Dout_A, BRAM_WEN_A) enable (BRAM_EN_A) clocked_by(ocmClock);
      method (* reg *)BRAM_Din_A bramOUT() clocked_by(ocmClock);
    endinterface
    
    schedule deq CF enq;
    schedule enq CF (deq, first);
    schedule first CF first;
    schedule first SB deq;
    schedule (bramTargetWires.bramIN, bramTargetWires.bramOUT) CF (bramTargetWires.bramIN, bramTargetWires.bramOUT);
    schedule (bramTargetWires.bramIN, bramTargetWires.bramOUT) CF (enq, deq, first);
  endmodule
