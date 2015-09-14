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

//typedef 14 IntegralWidth;
//typedef 36 FractionWidth;
typedef TAdd#(IntegralWidth, FractionWidth) LWidth;
typedef FixedPoint#(IntegralWidth, FractionWidth) LData;
typedef Bit#(LWidth) LDataBit;
typedef TDiv#(IntegralWidth, 2) HalfILWidth;
typedef TDiv#(FractionWidth, 2) HalfFLWidth;
typedef TAdd#(HalfILWidth, HalfFLWidth) HalfLWidth;
typedef Bit#(HalfLWidth) HalfLDataBit;
typedef TMul#(IntegralWidth, 2) DoubleILWidth;
typedef TMul#(FractionWidth, 2) DoubleFLWidth;
typedef FixedPoint#(DoubleILWidth, DoubleFLWidth) DoubleLData;
typedef Bit#(TAdd#(DoubleILWidth, DoubleFLWidth)) DoubleLDataBit;
typedef 5 PipeDepthLD;
typedef TAdd#(PipeDepthLD, 1) PipeDepthLD1;
typedef TAdd#(PipeDepthLD, 2) PipeDepthLD2;
typedef TAdd#(PipeDepthLD2, 2) QuotaL;
typedef TAdd#(TLog#(QuotaL), 1) LogQuotaL;

interface MultiplierLData;
    method Action put(LData x, LData y);
    method ActionValue#(LData) get();
endinterface

(*synthesize*)
module mkMultLData (MultiplierLData);
    Vector#(4,MultRawL)                 mult <- replicateM(mkMultRawL);
    FIFO#(Tuple2#(LData,LData))          in <- mkFIFO;
    FIFO#(LData)                        out <- mkSizedFIFO(valueOf(QuotaL));
    Reg#(DoubleLDataBit)                 t1 <- mkRegU();
    Reg#(DoubleLDataBit)                 t2 <- mkRegU();
    Counter#(LogQuotaL)                token <- mkCounter(fromInteger(valueOf(QuotaL)));
    Reg#(Vector#(PipeDepthLD2,Bool)) vldchk <- mkReg(replicate(False)); 
    RWire#(Bit#(0))                   sfin1 <- mkRWire;

    rule shift_vldchk;
        vldchk <= shiftInAt0(vldchk,isValid(sfin1.wget));
    endrule

    rule putMult(token.value > 0);
        token.down;
        in.deq;
        match {.x,.y} = in.first;
        HalfLDataBit x1 = truncateLSB(pack(x));
        HalfLDataBit x2 = truncate(pack(x));
        HalfLDataBit y1 = truncateLSB(pack(y));
        HalfLDataBit y2 = truncate(pack(y));
        mult[0].put(x1,y1);
        mult[1].put(x1,y2);
        mult[2].put(x2,y1);
        mult[3].put(x2,y2);
        sfin1.wset(?);
    endrule

    rule getMultRes0(vldchk[valueOf(PipeDepthLD)]);
        let mult0 = mult[0].get;
        let mult1 = mult[1].get;
        let mult2 = mult[2].get;
        let mult3 = mult[3].get;
        //$display("RES %b %b %b %b", mult0, mult1, mult2, mult3);
        t1 <= (zeroExtend(mult0)<<valueOf(LWidth)) + (zeroExtend(mult1)<<valueOf(HalfLWidth));
        t2 <= (zeroExtend(mult2)<<valueOf(HalfLWidth)) + zeroExtend(mult3);
    endrule

    rule getMultRes1(vldchk[valueOf(PipeDepthLD1)]);
        DoubleLDataBit resBit = t1 + t2;
        DoubleLData resFix = unpack(resBit);
        out.enq(fxptTruncate(resFix));
    endrule

    method Action put(LData x, LData y);
        in.enq(tuple2(x,y));
    endmethod

    method ActionValue#(LData) get;
        token.up;
        out.deq;
        return out.first;
    endmethod
endmodule

interface MultRawL;
    method Action put(HalfLDataBit _x, HalfLDataBit _y);
    method LDataBit get();
endinterface

module mkMultRawL (MultRawL);
    Reg#(HalfLDataBit) x <- mkRegU();
    Reg#(HalfLDataBit) y <- mkRegU();

    Vector#(PipeDepthLD, Reg#(LDataBit)) pipedOutput <- replicateM(mkRegU);

    rule propagate;
        //$display("%b %b",x, y);
        pipedOutput[0] <= pack(unsignedMul(unpack(x),unpack(y)));
        for(Integer i = 1; i < valueOf(PipeDepthLD); i=i+1)
            pipedOutput[i] <= pipedOutput[i-1];
    endrule

    method Action put(HalfLDataBit _x, HalfLDataBit _y);
        x <= _x;
        y <= _y;
    endmethod

    method LDataBit get();
        return pipedOutput[valueOf(PipeDepthLD)-1];
    endmethod
endmodule

