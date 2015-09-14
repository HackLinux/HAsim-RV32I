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


import CommonTypes::*;

import RegFile::*;

`define FILENAME "test.out"
`define NUM_ENTRIES 16

//Test that the C code output is packing correctly.
//Assumes the C output is in `FILENAME

module mkPackTest ();


  RegFile#(Bit#(8), Bit#(64)) rf <- mkRegFileLoad(`FILENAME, 0, 255);
  
  Reg#(Bit#(8)) count <- mkReg(0);
  
  rule dispContents (True);
  
    count <= count + 1;
    let b = rf.sub(count);
    Instruction ins = unpack(truncate(b));
    displayInst(ins);
    
    if (count == `NUM_ENTRIES)
      $finish(0);
  
  endrule
  
endmodule
