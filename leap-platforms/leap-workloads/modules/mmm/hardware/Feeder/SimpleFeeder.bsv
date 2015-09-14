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
import GetPut::*;

`include "Common.bsv"


module mkSimpleFeeder (Feeder);

  Reg#(Bit#(16)) counter <- mkReg(0);
  
  RegFile#(Bit#(16), Bit#(64)) prog <- mkRegFileFullLoad("program.hex");
  
  Reg#(Bit#(64)) approxCC <- mkReg(0);
  
  rule countCC (True);
   
    approxCC <= approxCC + 3;
  
  endrule
  
  interface Put ppcMessageInput;
  
    method Action put(PPCMessage msg);
      
      $display("PPC [APPROXIMATE PPC CC: %0d]: Received Message: 0x%h", approxCC, msg);
      
    endmethod
  
  endinterface
  
  
  interface Get ppcInstructionOutput;

    method ActionValue#(Instruction) get() if (prog.sub(counter) != 64'hAAAAAAAAAAAAAAAA);
      debug(feederDebug, $display("SimpleFeeder: feeding instruction[%d]: %h",
                                  counter,
                                  prog.sub(counter)));
 
      counter <= (counter == maxBound) ? counter : counter + 1;
      let b = prog.sub(counter);
      return unpack(truncate(b));
    endmethod

  endinterface

//  interface BRAMTargetWires bramTargetWires   = ?;
  interface BRAMInitatorWires bramInitiatorWires =  ?;

endmodule
