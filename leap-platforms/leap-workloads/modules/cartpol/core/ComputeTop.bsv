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
import Clocks::*;
import Connectable::*;
import GetPut::*;
import Float::*;
import FloatToFixedPoint::*;

interface ComputeTop;
   method Action setParam(Float _rad, Float _ang, Index _n);
   method ActionValue#(Pos) getPos();
endinterface

typedef enum {
  Init1,
  Init2,
  Init3,
  Init4,
  Init5,
  Init6,
  Init7,
  Init8,
  RunReq,
  RunResp
} Stage deriving (Bits, Eq);

module [CONNECTED_MODULE] mkComputeTop (ComputeTop);

   let slowClock <- exposeCurrentClock;
   let slowReset <- exposeCurrentReset;

   FIFOF#(Index)         maxN <- mkFIFOF;
   FIFOF#(Data)           rad <- mkFIFOF;
   FIFOF#(TData)          ang <- mkFIFOF;
   Reg#(TData)             dR <- mkRegU;
   Reg#(Data)            rCos <- mkRegU;
   Reg#(Data)             rp1 <- mkRegU;
   Reg#(LData)      invDeltaX <- mkRegU;
   Reg#(Data)    invDeltaYDen <- mkRegU;
   Reg#(LData)     invDeltaYF <- mkRegU;
   
   Reg#(TData)     deltaTheta <- mkRegU;
   Reg#(TData)      nextTheta <- mkRegU;
   Reg#(Stage)          stage <- mkReg(Init1);
   Reg#(Index)          count <- mkReg(0);
   Reg#(TData)           sinA <- mkRegU;

   //InverseN            invGen <- mkInverseN;
  
   Multiplier#(PipeDepth, Data, TData, Data)   multTD <- mkMultDataTData;
   //Multiplier#(PipeDepth, Data, Data, Data) multD <- mkMultData;

   RayGenerate     rayGenerate <- mkRayGenerate (multTD);

   let cosAndSinFastToSlow <- mkSyncFIFOFromCC(2,slowClock); 
   let cosAndSinSlowToFast <- mkSyncFIFOToCC(2,slowClock,slowReset); 

   CosAndSin#(IntWidthTheta, FracWidthTheta) 
   cordic <- mkCORDICCosAndSin_Circ(cordicCosSinIters, cordicCosSinStages, 
                                    clocked_by slowClock, reset_by slowReset);

   mkConnection(cordic.getCosSinPair ,toPut(cosAndSinSlowToFast).put);
   mkConnection(toGet(cosAndSinFastToSlow).get, cordic.putAngle);
   
   let divFastToSlow <- mkSyncFIFOFromCC(2,slowClock); 
   let divSlowToFast <- mkSyncFIFOToCC(2,slowClock,slowReset); 

   Division#(LData) divider <- mkCORDICDivision_Circ(cordicDivIters, cordicDivStages, 
                                                    clocked_by slowClock, reset_by slowReset);

   mkConnection(divider.getQuotient , toPut(divSlowToFast).put);

   (* mutually_exclusive = "reqDTheta,       rayGenerate_compute1" *)
//   (* mutually_exclusive = "respInv,         rayGenerate_compute1" *)
   (* mutually_exclusive = "reqInvDeltaX,    rayGenerate_compute1" *)
   (* mutually_exclusive = "respDTheta,      rayGenerate_compute1" *)
   (* mutually_exclusive = "respDTheta,      rayGenerate_compute2" *)
   (* mutually_exclusive = "reqInvDeltaX,    rayGenerate_compute2" *)
   (* mutually_exclusive = "respInvDeltaX,   rayGenerate_compute2" *)

   rule dividerConnect;
      match {.y, .x} = divFastToSlow.first;
      divider.putYX(y,x);
      divFastToSlow.deq;
   endrule

   rule reqInv (stage == Init1);
//       invGen.put(maxN.first);
//       stage <= Init2;
//    endrule

//    rule respInv (stage == Init2);
//      let invMaxNTD <- invGen.get;
      let invMaxNTD = getInverse(maxN.first);
      dR <= invMaxNTD;
      multTD.put(unpack(pack(invMaxNTD)), ang.first);
      ang.deq;
      
      cosAndSinFastToSlow.enq(ang.first);
      stage <= Init3;
   endrule

   rule reqDTheta(stage == Init3);
      let trigTD = cosAndSinSlowToFast.first; 
      cosAndSinSlowToFast.deq; 
      sinA <= trigTD.sin;
      multTD.put(rad.first, trigTD.cos);
      stage <= Init4;
   endrule
   
   rule respDTheta (stage == Init4);
      let  dThetaTemp <- multTD.get;
      TData dThetaCast = unpack(pack(dThetaTemp));
      deltaTheta      <= dThetaCast;
      nextTheta       <= dThetaCast;// + dThetaTemp;
      rp1             <= rad.first + 1;
      stage           <= Init5;
   endrule

   rule reqInvDeltaX (stage == Init5);
      let rCosTemp <- multTD.get;
      rCos <= rCosTemp;
      Data invDeltaXDen = rp1 - rCosTemp;
      divFastToSlow.enq(tuple2(FixedPoint{i:zeroExtend(unpack(maxN.first)),f:0}, 
                               FixedPoint{i:zeroExtend(invDeltaXDen.i),f:truncateLSB(invDeltaXDen.f)}));

      multTD.put(rp1, sinA);

      stage <= Init6;
   endrule
   
   rule respInvDeltaX  (stage == Init6);
      let invDeltaXTemp = divSlowToFast.first;
      divSlowToFast.deq;
      invDeltaX <= invDeltaXTemp;
      let invDeltaYDenTemp <- multTD.get;
      invDeltaYDen <= invDeltaYDenTemp;
      stage <= Init7;
   endrule
   
   rule reqInvDeltaY (stage == Init7);
      divFastToSlow.enq(tuple2(FixedPoint{i:zeroExtend(unpack(maxN.first)),f:0}, 
                               FixedPoint{i:zeroExtend(invDeltaYDen.i),f:truncateLSB(invDeltaYDen.f)}));

      stage <= Init8;
   endrule
   
   rule respInvDeltaY (stage == Init8);
      let invDeltaY = divSlowToFast.first;
      divSlowToFast.deq;
      invDeltaYF <= invDeltaY;
      $display("ComputeTop initiating Core with values: MaxN = ", maxN.first);
      fxptWrite(6, rad.first); $display(" = radd");
      fxptWrite(6, rCos); $display(" = RCos");
      fxptWrite(6, dR); $display(" = dR");
      fxptWrite(6, invDeltaX); $display(" = invDX");
      fxptWrite(6, invDeltaY); $display(" = invDY");
      
      rayGenerate.put(RayInit{maxN:maxN.first, r:rad.first, rCos:rCos, dR:dR,
                              trig:ThetaTrig{cosA:FixedPoint{i:1,f:0},sinA:FixedPoint{i:0,f:0}},
                              invDeltaX: invDeltaX, invDeltaY: invDeltaY});
      count <= 1;
      stage <= RunReq;
   endrule
   
   rule respNextTheta (stage == RunReq);
      cosAndSinFastToSlow.enq(nextTheta);
      //fxptWrite(8,nextTheta); $display(" = newTheta Requested");
      nextTheta <= nextTheta + deltaTheta;
     
      stage <= RunResp; 
   endrule
   
   rule sendNextTheta (stage == RunResp);
      let trigTD = cosAndSinSlowToFast.first; 
      cosAndSinSlowToFast.deq; 
      Data cosD = FixedPoint{i:zeroExtend(trigTD.cos.i), f:truncateLSB(trigTD.cos.f)};
      Data sinD = FixedPoint{i:zeroExtend(trigTD.sin.i), f:truncateLSB(trigTD.sin.f)};
      //fxptWrite(8,cosD); $display(" = new CosTheta, count %d, N %d", count, maxN.first);
      rayGenerate.put(RayInit{maxN:maxN.first, r:rad.first, rCos:rCos, dR:dR,
                              trig:ThetaTrig{cosA:trigTD.cos,sinA:trigTD.sin}, 
                              invDeltaX: invDeltaX, invDeltaY: invDeltaYF});
      count <= count + 1;
      
      if (count == maxN.first) begin
         maxN.deq;
         rad.deq;
         stage <= Init1;
      end
      else
         stage <= RunReq;
   endrule
   
   method Action setParam(Float _rad, Float _ang, Index _n);
      Data  fixed_rad = fromMaybe(?,floatToFixedPoint(_rad));
      TData fixed_ang = fromMaybe(?,floatToFixedPoint(_ang));
      rad.enq(fixed_rad);
      ang.enq(fixed_ang);
      maxN.enq(_n - 1);
   endmethod
   
   method getPos = rayGenerate.get;
endmodule
