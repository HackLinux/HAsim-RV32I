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
import Types::*;
import Connectable::*;

interface RayGenerate;
   method Action put(RayInit _rayInit);
   method ActionValue#(Pos) get();
endinterface

typedef enum {Req, Resp} State deriving (Bits, Eq);

module mkRayGenerate#(Multiplier#(PipeDepth, Data, TData, Data) mult0) (RayGenerate);
   //Reg#(Index) counter1 <- mkReg(0);
   Reg#(Index) counter2 <- mkReg(0);
   Reg#(Index) counter3 <- mkReg(0);

   FIFOF#(RayInit)   in <- mkFIFOF;
   FIFOF#(Data)   rCosF <- mkFIFOF;
   FIFOF#(LData)  invDX <- mkFIFOF;
   FIFOF#(LData)  invDY <- mkFIFOF;
   //FIFOF#(Index)  maxN1 <- mkFIFOF;
   FIFOF#(Index)  maxN2 <- mkFIFOF;
   FIFOF#(Index)  maxN3 <- mkFIFOF;
   FIFOF#(Pos)      out <- mkFIFOF;

   Multiplier#(PipeDepth, Data, TData, Data) mult1 <- mkMultDataTData;
   Vector#(2, Multiplier#(PipeDepth, Data, LData, LData)) mult <- replicateM(mkMultLData);

   let rInit = in.first.r;
   let dR = in.first.dR;
   let trig = in.first.trig;
   let rCos = rCosF.first;
   let invDeltaX = invDX.first;
   let invDeltaY = invDY.first;

   Reg#(Data) rReg <- mkRegU;

   rule compute1;
      //let deltaR <- mult[0].get;
      Data dRTrunc = FixedPoint{i:zeroExtend(dR.i), f:truncateLSB(dR.f)};
      let r = (counter2 == 0)? rInit: rReg;
      rReg <= r  + dRTrunc;
      mult0.put(r, trig.cosA);
      mult1.put(r, trig.sinA);
      if(counter2 == maxN2.first)
         begin
            in.deq;
            maxN2.deq;
            counter2 <= 0;
         end
      else
          counter2 <= counter2 + 1;
   endrule
   
   rule compute2;// (stage == Comp2);
       let x <- mult0.get;
       let y <- mult1.get;
       mult[0].put(x - rCos, invDeltaX);
       mult[1].put(y, invDeltaY);
       if(counter3 == maxN3.first)
       begin
           maxN3.deq;
           rCosF.deq;
           invDX.deq;
           invDY.deq;
           counter3 <= 0;
       end
       else
          counter3 <= counter3 + 1;
   endrule

   rule compute3;
       let x <- mult[0].get;
       let y <- mult[1].get;
       //fxptWrite(8,x);$display(" = x Bspec");
       //fxptWrite(8,y);$display(" = y Bspec");
       out.enq(Pos{x: truncate(pack(fxptGetInt(x))), y: truncate(pack(fxptGetInt(y)))});
   endrule

   method Action put(RayInit _rayInit);
       in.enq(_rayInit);
       //maxN1.enq(_rayInit.maxN);
       maxN2.enq(_rayInit.maxN);
       maxN3.enq(_rayInit.maxN);
       rCosF.enq(_rayInit.rCos);
       invDX.enq(_rayInit.invDeltaX);
       invDY.enq(_rayInit.invDeltaY);
   endmethod

   method ActionValue#(Pos) get();
       out.deq;
       return out.first;
   endmethod
endmodule
