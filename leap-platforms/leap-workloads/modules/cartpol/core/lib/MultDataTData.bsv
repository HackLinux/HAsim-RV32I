//----------------------------------------------------------------------//
// The MIT License 
// 
// Copyright (c) 2009 MIT
// 
// Permission is hereby granted, free of charge, to any person 
// obtaining a copy of this software and associated documentation 
// files (the "Software"), to deal in the Software without 
// restriction, including without limitation the rights to use,
// copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
// HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
//
// Author: Asif Khan
//
//----------------------------------------------------------------------//

import FIFO::*;
import FixedPointNew::*;
import Vector::*;
import Counter::*;
import RWire::*;
import Types::*;

//typedef 8 IntWidthData;
//typedef 42 FracWidthTheta;
typedef TAdd#(IntWidthData, FracWidthTheta) TWidth;
typedef FixedPoint#(IntWidthData, FracWidthTheta) DTData;
typedef Bit#(TWidth) DTDataBit;
typedef TDiv#(IntWidthData, 2) HalfITWidth;
typedef TDiv#(FracWidthTheta, 2) HalfFTWidth;
typedef TAdd#(HalfITWidth, HalfFTWidth) HalfTWidth;
typedef Bit#(HalfTWidth) HalfDTDataBit;
typedef TMul#(IntWidthData, 2) DoubleITWidth;
typedef TMul#(FracWidthTheta, 2) DoubleFTWidth;
typedef FixedPoint#(DoubleITWidth, DoubleFTWidth) DoubleDTData;
typedef Bit#(TAdd#(DoubleITWidth, DoubleFTWidth)) DoubleDTDataBit;
typedef 5 PipeDepthDTD;
typedef TAdd#(PipeDepthDTD, 1) PipeDepthDTD1;
typedef TAdd#(PipeDepthDTD, 2) PipeDepthDTD2;
typedef TAdd#(PipeDepthDTD2, 2) QuotaT;
typedef TAdd#(TLog#(QuotaT), 1) LogQuotaT;

interface MultiplierDTData;
    method Action put(DTData x, DTData y);
    method ActionValue#(DTData) get();
endinterface

(*synthesize*)
module mkMultDataTData (MultiplierDTData);
    Vector#(4,MultRawT)                  mult <- replicateM(mkMultRawT);
    FIFO#(Tuple2#(DTData,DTData))         in <- mkFIFO;
    FIFO#(DTData)                        out <- mkSizedFIFO(valueOf(QuotaT));
    Reg#(DoubleDTDataBit)                 t1 <- mkRegU();
    Reg#(DoubleDTDataBit)                 t2 <- mkRegU();
    Counter#(LogQuotaT)                 token <- mkCounter(fromInteger(valueOf(QuotaT)));
    Reg#(Vector#(PipeDepthDTD2,Bool)) vldchk <- mkReg(replicate(False)); 
    RWire#(Bit#(0))                    sfin1 <- mkRWire;

    rule shift_vldchk;
        vldchk <= shiftInAt0(vldchk,isValid(sfin1.wget));
    endrule

    rule putMult(token.value > 0);
        token.down;
        in.deq;
        match {.x,.y} = in.first;
        HalfDTDataBit x1 = truncateLSB(pack(x));
        HalfDTDataBit x2 = truncate(pack(x));
        HalfDTDataBit y1 = truncateLSB(pack(y));
        HalfDTDataBit y2 = truncate(pack(y));
        mult[0].put(x1,y1);
        mult[1].put(x1,y2);
        mult[2].put(x2,y1);
        mult[3].put(x2,y2);
        sfin1.wset(?);
    endrule

    rule getMultRes0(vldchk[valueOf(PipeDepthDTD)]);
        let mult0 = mult[0].get;
        let mult1 = mult[1].get;
        let mult2 = mult[2].get;
        let mult3 = mult[3].get;
        //$display("RES %b %b %b %b", mult0, mult1, mult2, mult3);
        t1 <= (zeroExtend(mult0)<<valueOf(TWidth)) + (zeroExtend(mult1)<<valueOf(HalfTWidth));
        t2 <= (zeroExtend(mult2)<<valueOf(HalfTWidth)) + zeroExtend(mult3);
    endrule

    rule getMultRes1(vldchk[valueOf(PipeDepthDTD1)]);
        DoubleDTDataBit resBit = t1 + t2;
        DoubleDTData resFix = unpack(resBit);
        out.enq(fxptTruncate(resFix));
    endrule

    method Action put(DTData x, DTData y);
        in.enq(tuple2(x,y));
    endmethod

    method ActionValue#(DTData) get;
        token.up;
        out.deq;
        return out.first;
    endmethod
endmodule

interface MultRawT;
    method Action put(HalfDTDataBit _x, HalfDTDataBit _y);
    method DTDataBit get();
endinterface

module mkMultRawT (MultRawT);
    Reg#(HalfDTDataBit) x <- mkRegU();
    Reg#(HalfDTDataBit) y <- mkRegU();

    Vector#(PipeDepthDTD, Reg#(DTDataBit)) pipedOutput <- replicateM(mkRegU);

    rule propagate;
        //$display("%b %b",x, y);
        pipedOutput[0] <= pack(unsignedMul(unpack(x),unpack(y)));
        for(Integer i = 1; i < valueOf(PipeDepthDTD); i=i+1)
            pipedOutput[i] <= pipedOutput[i-1];
    endrule

    method Action put(HalfDTDataBit _x, HalfDTDataBit _y);
        x <= _x;
        y <= _y;
    endmethod

    method DTDataBit get();
        return pipedOutput[valueOf(PipeDepthDTD)-1];
    endmethod
endmodule

