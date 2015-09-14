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

`define BRAM_EN_NAME  "enBRAM"
`define BRAM_ADDR_NAME "addrBRAM"
`define BRAM_DOUT_NAME "doutBRAM"
`define BRAM_WEN_NAME "wenBRAM"
`define BRAM_DIN_NAME "dinBRAM"

//Note: Now assumes MCD are used for Clock, reset

interface BRAMTargetWires;
  (* always_ready, prefix="", enable=`BRAM_EN_NAME *)
  method Action bramIN((* port=`BRAM_ADDR_NAME *) Bit#(32) addr, 
                       (* port=`BRAM_DOUT_NAME *) Bit#(32) data, 
                       (* port=`BRAM_WEN_NAME *) Bit#(4) wen);
  (* always_ready, prefix="", result=`BRAM_DIN_NAME *)
  method Bit#(32) bramOUT();
endinterface
