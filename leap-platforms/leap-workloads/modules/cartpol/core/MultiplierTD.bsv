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
// Author: Abhinav Agarwal, Asif Khan, Muralidaran Vijayaraghavan
//
//----------------------------------------------------------------------//

import Types::*;
import FIFOF::*;
import FixedPointNew::*;
import Vector::*;
import Counter::*;

interface MultiplierTD;
    method Action put(TData x, TData y);
    method ActionValue#(TData) get();
endinterface

typedef 4 PipeDepth;
typedef TAdd#(PipeDepth, 1) PipeDepth1;
typedef TAdd#(TLog#(PipeDepth), 1) LogPipeDepth;
typedef TMul#(IntWidthTheta, 2) DoubleIWidth;
typedef TMul#(FracWidthTheta, 2) DoubleFWidth;
typedef FixedPoint#(DoubleIWidth, DoubleFWidth) DoubleTData;

module mkMultiplierTD (MultiplierTD);
    FIFOF#(Tuple2#(TData,TData)) in <- mkFIFOF;
    FIFOF#(TData)              out <- mkFIFOF;

    Counter#(LogPipeDepth)  count <- mkCounter(0);

    MultRaw                  mult <- mkMultRawTD;

    Bit#(LogPipeDepth) maxCount = 2;

    rule sendValid(in.notEmpty && count.value < maxCount);
        match {.x,.y} = in.first;
        in.deq;
        mult.put(True, x,y);
        count.up;
    endrule

    rule sendInvalid(!in.notEmpty || count.value == maxCount);
        mult.put(False, 0, 0);
    endrule

    match {.valid, .val} = mult.get;
    rule receive(valid);
        out.enq(val);
    endrule

    method Action put(TData x, TData y);
        in.enq(tuple2(x,y));
    endmethod

    method ActionValue#(TData) get;
        count.down;
        out.deq;
        return out.first;
    endmethod
endmodule

interface MultRawTD;
    method Action put(Bool _valid, TData _x, TData _y);
    method Tuple2#(Bool, TData) get();
endinterface

module mkMultRawTD (MultRawTD);

    Reg#(TData) x <- mkRegU();
    Reg#(TData) y <- mkRegU();

    Vector#(PipeDepth, Reg#(DoubleTData)) pipedOutput <- replicateM(mkRegU);
    Vector#(PipeDepth1, Reg#(Bool))            valid <- replicateM(mkRegU);

    rule propagate;
        pipedOutput[0] <= fxptUMult(x,y);
        for(Integer i = 1; i < valueOf(PipeDepth); i=i+1)
        begin
            pipedOutput[i] <= pipedOutput[i-1];
            valid[i] <= valid[i-1];
        end
        valid[valueOf(PipeDepth)] <= valid[valueOf(PipeDepth)-1];
    endrule

    method Action put(Bool _valid, TData _x, TData _y);
        x <= _x;
        y <= _y;
        valid[0] <= _valid;
    endmethod

    method Tuple2#(Bool, TData) get();
        return tuple2(valid[valueOf(PipeDepth)], fxptTruncate(pipedOutput[valueOf(PipeDepth)-1]));
    endmethod
endmodule

