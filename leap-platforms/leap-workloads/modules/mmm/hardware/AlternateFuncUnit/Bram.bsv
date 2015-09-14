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

interface Bram#(type addr_t, type data_t);
    method Action readReq(addr_t addr);
    method data_t readResp();
    method Action write(addr_t addr, data_t data);
    method Action zero();
endinterface

import "BVI" bram_fu =
module mkBram(Bram#(addr_t, data_t))
    provisos(Bounded#(addr_t), Bits#(addr_t, addr_sz), Bits#(data_t, data_sz), Literal#(addr_t));

    no_reset;

    parameter addr_width = valueOf(addr_sz);
    parameter data_width = valueOf(data_sz);
    parameter hi = valueOf(TSub#(TExp#(addr_sz),1));
    parameter lo = 0;

    method readReq(read_addr) enable(read_en);
    method read_data readResp();
    method write(write_addr, write_data) enable(write_en);
    method zero() enable(RST);

    schedule readReq C readReq;
    schedule write C write;
    schedule readResp CF (readReq, readResp, write);
    schedule readReq CF write;
endmodule

module mkBramInstance(Bram#(Bit#(4), Bit#(5)));
    Bram#(Bit#(4), Bit#(5)) bram <- mkBram();
    return bram;
endmodule

module mkBramTest(Empty);
    Bram#(Bit#(4), Bit#(5)) bram <- mkBram();
    Reg#(Bit#(5)) val <- mkReg(11);

    rule all(True);
        bram.readReq(4);
        bram.write(9, val);
        $display("both");
    endrule
endmodule
