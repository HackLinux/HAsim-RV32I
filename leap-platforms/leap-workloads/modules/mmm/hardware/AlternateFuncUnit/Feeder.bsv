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

Author: Nirav Dave, Michael Pellauer
*/

import FunctionalUnit::*;
import Types::*;
import FIFO::*;
import FIFOF::*;
import GetPut::*;

typedef Bit#(32) Addr;

typedef enum {InstFull, InstFIFO, DataInFull, DataInFIFO, DataOutEmpty, DataOutFIFO, Store} AddrType deriving (Bits, Eq);

(* always_ready *)
interface Feeder;
    method Action request(Bit#(4) isWrite, Addr address, Data data);
    method Data response();
endinterface

Integer fifoSize = 16;

(* synthesize *)
module mkfeeder(Feeder);
    FunctionalUnit fu <- mkfunctionalunit();

    FIFOF#(Op)      instFIFO <- mkSizedFIFOF(fifoSize);
    FIFOF#(Data)  dataInFIFO <- mkSizedFIFOF(fifoSize);
    FIFOF#(Data) dataOutFIFO <- mkSizedFIFOF(fifoSize);

    Reg#(Data) responseReg <- mkReg(0);
    Reg#(Data) storeReturn <- mkReg(0);

    RWire#(Bit#(4)) isWriteWire <- mkRWire();
    RWire#(Addr)       addrWire <- mkRWire();
    RWire#(Data)       dataWire <- mkRWire();

    function AddrType getAddr(Addr addr) = unpack(truncate(addr));

    let addr = getAddr(fromMaybe(?, addrWire.wget()));
    let addrValid = isValid(addrWire.wget());

    let dataVal = fromMaybe(?, dataWire.wget());

    rule processInst(True);
        instFIFO.deq();
        fu.putInst.put(instFIFO.first());
    endrule

    rule processDataIn(True);
        dataInFIFO.deq();
        fu.putData.put(dataInFIFO.first());
    endrule

    rule processDataOut(True);
        let data <- fu.getData.get();
        dataOutFIFO.enq(data);
    endrule

    rule processStore(True);
        let store <- fu.getStore.get();
        storeReturn <= storeReturn + 1;
    endrule

    rule processInstFull(addr == InstFull && addrValid);
        responseReg <= zeroExtend(pack(instFIFO.notFull()));
    endrule

    rule processInstFIFO(addr == InstFIFO && addrValid);
        instFIFO.enq(unpack(truncate(dataVal)));
    endrule

    rule processDataInFull(addr == DataInFull && addrValid);
        responseReg <= zeroExtend(pack(dataInFIFO.notFull()));
    endrule

    rule processDataInFIFO(addr == DataInFIFO && addrValid);
        dataInFIFO.enq(dataVal);
    endrule

    rule processDataOutEmpty(addr == DataOutEmpty && addrValid);
        responseReg <= zeroExtend(pack(dataOutFIFO.notEmpty()));
    endrule

    rule processDataOutFIFO(addr == DataOutFIFO && addrValid);
        dataOutFIFO.deq();
        responseReg <= dataOutFIFO.first();
    endrule

    rule processStoreInst(addr == Store && addrValid);
        responseReg <= storeReturn;
    endrule

    method Action request(Bit#(4) isWrite, Addr address, Data data);
        isWriteWire.wset(isWrite);
        addrWire.wset(address);
        dataWire.wset(data);
    endmethod

    method Data response();
        return responseReg;
    endmethod
endmodule
