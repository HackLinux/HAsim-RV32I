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
// Author: Abhinav Agarwal, Nirav Dave, Muralidaran Vijayaraghavan
//
//----------------------------------------------------------------------//

`include "awb/provides/librl_bsv.bsh"
`include "awb/provides/soft_connections.bsh"
`include "awb/provides/soft_services.bsh"
`include "awb/provides/soft_services_lib.bsh"
`include "awb/provides/soft_services_deps.bsh"
`include "awb/provides/cartpol_common.bsh"
`include "awb/provides/cartpol_cordic.bsh"


import FIFOF::*;
import Vector::*;
import Connectable::*;

interface ThetaGenerate;
    method Bool notEmpty();
    method Action put(ThetaInit _thetaInit);
    method ActionValue#(RayInit) get();
endinterface

interface RayGenerate;
    method Action put(RayInit _rayInit);
    method ActionValue#(Pos) get();
endinterface

typedef enum {Req, Resp} State deriving (Bits, Eq);

interface ComputeCore;
    method Bool notEmpty();
    method Action put(ThetaInit _thetaInit);
    method ActionValue#(Pos) get();
endinterface

module mkComputeCore#(Multiplier#(PipeDepth, Data, TData, Data) mult0)(ComputeCore);
    FIFOF#(RayInit)        fifo <- mkFIFOF;
    ThetaGenerate thetaGenerate <- mkThetaGenerate(fifo, mult0);
    RayGenerate     rayGenerate <- mkRayGenerate(fifo);

    method notEmpty = thetaGenerate.notEmpty;
    method put = thetaGenerate.put;
    method get = rayGenerate.get;
endmodule

module mkThetaGenerate#(FIFOF#(RayInit) out, Multiplier#(PipeDepth, Data, TData, Data) mult0)(ThetaGenerate);
    Reg#(Index)         counter <- mkReg(0);

    FIFOF#(ThetaInit)        in <- mkSizedFIFOF(1);

    Reg#(State)           state <- mkReg(Req);
    Reg#(Bit#(2))    countState <- mkReg(0);

    let                   mult1 <- mkMultTData;

    Reg#(Data) xReg <- mkRegU;
    Reg#(Data) yReg <- mkRegU;

    Reg#(TData) dxReg <- mkRegU;
    Reg#(TData) dyReg <- mkRegU;

    Reg#(TData) cosDTReg <- mkRegU;
    Reg#(TData) sinDTReg <- mkRegU;

    let maxN   = in.first.maxN;
    let r      = in.first.r;
    let rCos   = in.first.rCos;
    let dr     = in.first.dr;
    let cosDTheta = in.first.cosDTheta;
    let sinDTheta = in.first.sinDTheta;
    let invDeltaX = in.first.invDeltaX;
    let invDeltaY = in.first.invDeltaY;

    rule computeReq(state == Req && in.notEmpty);
        let a = (countState == 0 || countState == 2)? xReg : yReg;
        let da = (countState == 0 || countState == 2)? dxReg : dyReg;
        let b = (countState == 0 || countState == 3)? cosDTReg : sinDTReg;

        mult0.put(a, b);
        mult1.put(da, b);

        countState <= countState + 1;
        if(countState == 3)
            state <= Resp;
    endrule

    rule computeResp(state == Resp);
        let val0 <- mult0.get;
        let val1 <- mult1.get;

        cosDTReg <= cosDTheta;
        sinDTReg <= sinDTheta;

        case (countState) matches
            0:
            begin
                xReg <= val0;
                dxReg <= val1;
            end
            1:
            begin
                xReg <= xReg - val0;
                dxReg <= dxReg - val1;
            end
            2:
            begin
                yReg <= val0;
                dyReg <= val1;
            end
            3:
            begin
                yReg <= yReg + val0;
                dyReg <= dyReg + val1;
            end
        endcase

        countState <= countState + 1;
        if(countState == 3)
        begin
            out.enq(RayInit{maxN: maxN, x: xReg-rCos, y: yReg + val0, dx: dxReg,
                            dy: dyReg + val1, invDeltaX: invDeltaX, invDeltaY: invDeltaY});
            state <= Req;

            if(counter != maxN)
                counter <= counter + 1;
            else
            begin
                in.deq;
                counter <= 0;
            end
        end
    endrule

    method notEmpty = in.notEmpty;

    method Action put(ThetaInit _thetaInit) if(!in.notEmpty);
        in.enq(_thetaInit);
        xReg <= _thetaInit.r;
        yReg <= FixedPoint{i:0, f:0};
        dxReg <= _thetaInit.dr;
        dyReg <= FixedPoint{i:0, f:0};
        cosDTReg <= FixedPoint{i:1, f:0};
        sinDTReg <= FixedPoint{i:0, f:0};
    endmethod

    method ActionValue#(RayInit) get();
        out.deq;
        return out.first;
    endmethod
endmodule

module mkRayGenerate#(FIFOF#(RayInit) in)(RayGenerate);
    Reg#(Index) counter <- mkReg(0);

    FIFOF#(Pos)      out <- mkFIFOF;

    Vector#(2, Multiplier#(PipeDepth, Data, LData, IntData)) mult <- replicateM(mkMultFinal);

    let maxN  = in.first.maxN;
    let xInit = in.first.x;
    let yInit = in.first.y;
    let dx = FixedPoint{i: zeroExtend(in.first.dx.i), f: truncateLSB(in.first.dx.f)};
    let dy = FixedPoint{i: zeroExtend(in.first.dy.i), f: truncateLSB(in.first.dy.f)};
    let invDeltaX = in.first.invDeltaX;
    let invDeltaY = in.first.invDeltaY;

    Reg#(Data) xReg <- mkRegU;
    Reg#(Data) yReg <- mkRegU;

    rule compute1;
        let x = (counter == 0)? xInit : xReg;
        let y = (counter == 0)? yInit : yReg;

        xReg <= x + dx;
        yReg <= y + dy;

        mult[0].put(x, invDeltaX);
        mult[1].put(y, invDeltaY);

        if(counter == maxN)
        begin
            in.deq;
            counter <= 0;
        end
        else
            counter <= counter + 1;
    endrule

    rule compute2;
        let x <- mult[0].get;
        let y <- mult[1].get;
        out.enq(Pos{x: truncate(x.i), y: truncate(y.i)});
    endrule

    method Action put(RayInit _rayInit);
        in.enq(_rayInit);
    endmethod

    method ActionValue#(Pos) get();
        out.deq;
        return out.first;
    endmethod
endmodule
