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

import FIFOF::*;
import ComputeTypes::*;
import FixedPointNew::*;
import ComputeCore::*;
import CORDICTrigonometry::*;
import CORDICDivision::*;
import Vector::*;
import InvN::*;
import MultiplierNew::*;
import MultiplierInstances::*;
import Clocks::*;
import Connectable::*;
import FIFOUtility::*;
import GetPut::*;
import Float::*;
import FloatToFixedPoint::*;

interface Compute;
    method Action setParam(Float _r, Float _theta, Index _n);
    method ActionValue#(Pos) getPos();
endinterface

(*synthesize*)
module mkCompute#(Clock slowClock, Reset slowReset)(Compute);
    FIFOF#(TopInit) in <- mkSizedFIFOF(1);

    InvN      inverseN <- mkInvN;

    CosAndSin#(IntWidthTheta, FracWidthTheta) cordic <- mkCORDICCosAndSin_Circ(cordicCosSinIters, cordicCosSinStages, clocked_by slowClock, reset_by slowReset);

    let cosAndSinFastToSlow <- mkSyncFIFOFromCC(2,slowClock); 
    let cosAndSinSlowToFast <- mkSyncFIFOToCC(2,slowClock,slowReset); 

    mkConnection(cordic.getCosSinPair ,syncFifoToPut(cosAndSinSlowToFast).put);
    mkConnection(syncFifoToGet(cosAndSinFastToSlow).get, cordic.putAngle);

    Division#(LData) divider <- mkCORDICDivision_Circ(cordicDivIters, cordicDivStages, clocked_by slowClock, reset_by slowReset);

    let divFastToSlow <- mkSyncFIFOFromCC(2,slowClock); 
    let divSlowToFast <- mkSyncFIFOToCC(2,slowClock,slowReset); 

    mkConnection(divider.getQuotient ,syncFifoToPut(divSlowToFast).put);

    rule dividerConnect;
        match {.y, .x} = divFastToSlow.first;
        divider.putYX(y,x);
        divFastToSlow.deq;
    endrule

    let                                         mult <- mkMultDataTData;
    ComputeCore                                 core <- mkComputeCore(mult);

    Reg#(Bit#(1)) invReq <- mkReg(0);
    Reg#(Bit#(2)) multReq <- mkReg(0);
    Reg#(Bit#(2)) multResp <- mkReg(0);
    Reg#(Bit#(2)) cordicReq <- mkReg(0);
    Reg#(Bit#(1)) cordicResp <- mkReg(0);
    Reg#(Bit#(2)) dividerReq <- mkReg(0);
    Reg#(Bit#(1)) dividerResp <- mkReg(0);

    Reg#(TData) dr <- mkRegU;
    Reg#(TData) sin <- mkRegU;
    Reg#(Data) rCos <- mkRegU;
    Reg#(LData) invDeltaX <- mkRegU;

    let maxN = in.first.maxN;
    let r = in.first.r;
    let theta = in.first.theta;

    (* mutually_exclusive = "multReq1, core_thetaGenerate_computeReq" *)
    (* mutually_exclusive = "multReq2, core_thetaGenerate_computeReq" *)
    (* mutually_exclusive = "multReq3, core_thetaGenerate_computeReq" *)
    (* mutually_exclusive = "dividerReq1, core_thetaGenerate_computeResp" *)
    (* mutually_exclusive = "dividerReq2, core_thetaGenerate_computeResp" *)
    (* mutually_exclusive = "cordicReq2, core_thetaGenerate_computeResp" *)

    rule invRule(invReq == 0);
        inverseN.put(maxN);
        invReq <= 1;
    endrule

    rule multReq1(multReq == 0 && invReq == 1);
        let _dr <- inverseN.get;
        dr <= _dr;
        mult.put(unpack(pack(_dr)), unpack(pack(theta)));
        multReq <= 1;
    endrule

    rule cordicReq1(cordicReq == 0);
        cosAndSinFastToSlow.enq(theta);
        cordicReq <= 1;
    endrule

    rule cordicReq2(cordicReq == 1 && multResp == 0);
        let dTheta <- mult.get;
        cosAndSinFastToSlow.enq(unpack(pack(dTheta)));
        cordicReq <= 2;
        multResp <= 1;
    endrule

    rule multReq2(multReq == 1 && cordicResp == 0);
        let trig = cosAndSinSlowToFast.first;
        cosAndSinSlowToFast.deq;
        mult.put(r, trig.cos);
        sin <= trig.sin;
        multReq <= 2;
        cordicResp <= 1;
    endrule

    rule multReq3(multReq == 2);
        mult.put(r+1, sin);
        multReq <= 3;
    endrule

    rule dividerReq1(dividerReq == 0 && multResp == 1);
        let _rCos <- mult.get;
        rCos <= _rCos;
        let invX = r+1-_rCos;
        divFastToSlow.enq(tuple2(FixedPoint{i: zeroExtend(maxN), f: 0}, FixedPoint{i: zeroExtend(invX.i), f: truncateLSB(invX.f)}));
        dividerReq <= 1;
        multResp <= 2;
    endrule

    rule dividerReq2(dividerReq == 1 && multResp == 2);
        let _r1Sin <- mult.get;
        divFastToSlow.enq(tuple2(FixedPoint{i: zeroExtend(maxN), f: 0}, FixedPoint{i: zeroExtend(_r1Sin.i), f: truncateLSB(_r1Sin.f)}));
        dividerReq <= 2;
        multResp <= 3;
    endrule

    rule dividerResp1(dividerResp == 0);
        let _invDeltaX = divSlowToFast.first;
        divSlowToFast.deq;
        invDeltaX <= _invDeltaX;
        dividerResp <= 1;
    endrule

    rule dividerResp2(dividerReq == 2 && dividerResp == 1 && cordicReq == 2 && cordicResp == 1 && multReq == 3 && multResp == 3 && invReq == 1);
        let invDeltaY = divSlowToFast.first;
        divSlowToFast.deq;
        let trig = cosAndSinSlowToFast.first;
        cosAndSinSlowToFast.deq;
        core.put(ThetaInit{maxN: maxN, r: r, rCos: rCos, cosDTheta: trig.cos, sinDTheta: trig.sin,
                           dr: dr, invDeltaX: invDeltaX, invDeltaY: invDeltaY});

        //$display("%d = n", maxN);
        //fxptWrite(5, r); $display(" = r");
        //fxptWrite(5, theta); $display(" = theta");
        //fxptWrite(5, rCos); $display(" = rCos");
        //fxptWrite(5, trig.cos); $display(" = cosDTheta");
        //fxptWrite(5, trig.sin); $display(" = sinDTheta");
        //fxptWrite(5, dr); $display(" = dr");
        //fxptWrite(5, invDeltaX); $display(" = invDeltaX");
        //fxptWrite(5, invDeltaY); $display(" = invDeltaY");

        invReq <= 0;
        multReq <= 0;
        multResp <= 0;
        cordicReq <= 0;
        cordicResp <= 0;
        dividerReq <= 0;
        dividerResp <= 0;

        in.deq;
    endrule

    method Action setParam(Float _r, Float _theta, Index _n) if(!core.notEmpty);
        Data rVal = validValue(floatToFixedPoint(_r));
        TData thetaVal = validValue(floatToFixedPoint(_theta));
        in.enq(TopInit{maxN: _n-1, r: rVal, theta: thetaVal});
    endmethod

    method getPos = core.get;
endmodule
