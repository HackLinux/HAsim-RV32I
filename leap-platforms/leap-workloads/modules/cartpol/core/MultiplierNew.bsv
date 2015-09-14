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
// Author: Alfred Man Cheuk Ng (mcn02@csail.mit.edu)
//
//----------------------------------------------------------------------//

`include "awb/provides/librl_bsv.bsh"
`include "awb/provides/soft_connections.bsh"
`include "awb/provides/soft_services.bsh"
`include "awb/provides/soft_services_lib.bsh"
`include "awb/provides/soft_services_deps.bsh"
`include "awb/provides/cartpol_common.bsh"
`include "awb/provides/cartpol_cordic.bsh"

import Counter::*;
import FIFO::*;
import RWire::*;
import Vector::*;

interface Multiplier#(numeric type pipe_depth, 
                      type in1_t,
                      type in2_t,
                      type out_t);
    method Action put(in1_t x, in2_t y);
    method ActionValue#(out_t) get();
endinterface

(* always_enabled *)
module mkMultRaw (Multiplier#(pipe_depth,
                              FixedPoint#(ii1,if1),
                              FixedPoint#(ii2,if2),
                              FixedPoint#(oi,of)))
   provisos (Add#(xxA,2,pipe_depth)
             ,Add#(pipe_depth_m_1,1,pipe_depth)
             ,Add#(ii1,ii2,oi)   // ri = ai + bi
             ,Add#(if1,if2,of)   // rf = af + bf
             ,Add#(ii1,if1,ib1)
             ,Add#(ii2,if2,ib2)
             ,Add#(ib1,ib2,ob)
             ,Add#(oi,of,ob)
            ) ;
   
   Reg#(FixedPoint#(ii1,if1)) x_reg <- mkRegU;
   Reg#(FixedPoint#(ii2,if2)) y_reg <- mkRegU;
   
   Vector#(pipe_depth_m_1,Reg#(FixedPoint#(oi,of))) pipe_regs <- replicateM(mkRegU);
   
   rule shiftRegs(True);
      pipe_regs[0] <= fxptUMult(x_reg,y_reg);
      for (Integer i = 0; i < (valueOf(pipe_depth_m_1)-1); i = i + 1)
         pipe_regs[i+1] <= pipe_regs[i];
   endrule
   
   method Action put(FixedPoint#(ii1,if1) x, FixedPoint#(ii2,if2) y);
      x_reg <= x;
      y_reg <= y;   
   endmethod
   
   method ActionValue#(FixedPoint#(oi,of)) get();
      return pipe_regs[valueOf(pipe_depth_m_1)-1];
   endmethod
   
endmodule

module mkMultiplier(Multiplier#(pipe_depth,
                                FixedPoint#(ii1,if1),
                                FixedPoint#(ii2,if2),
                                FixedPoint#(oi,of)))
   provisos (Add#(xxA,2,pipe_depth), // pipe_depth >= 2
             Add#(pipe_depth_m_1,1,pipe_depth),
             Add#(ii1,ii2,ri),   // ri = ai + bi
             Add#(if1,if2,rf),   // rf = af + bf
             Add#(ii1,if1,ib1),
             Add#(ii2,if2,ib2),
             Add#(ib1,ib2,rb),
             Add#(ri,rf,rb),
             Add#(xxB,oi,ri),
             Add#(xxC,of,rf),
//             Add#(oi,of,oo),
//             Add#(1,xxD,oo),
             Add#(pipe_depth,2,quota_max),
             Add#(quota_max,1,quota_sz),
             Log#(quota_sz,quota_bit_sz)
             ) ;
   
   Multiplier#(pipe_depth, 
               FixedPoint#(ii1,if1),
               FixedPoint#(ii2,if2), 
               FixedPoint#(ri,rf)) mult_raw <- mkMultRaw;
   
   RWire#(Bit#(0))                sfin1  <- mkRWire;
   FIFO#(FixedPoint#(oi,of))      out_q  <- mkSizedFIFO(valueOf(quota_max));
   Counter#(quota_bit_sz)         token  <- mkCounter(fromInteger(valueOf(quota_max)));
   Reg#(Vector#(pipe_depth,Bool)) vldchk <- mkReg(replicate(False)); 
                   
   rule shift_vldchk(True);
      vldchk <= shiftInAt0(vldchk,isValid(sfin1.wget));
   endrule

   rule getMultRes(vldchk[valueOf(pipe_depth)-1]);
      let mult_res <- mult_raw.get;
      out_q.enq(fxptTruncate(mult_res));
   endrule
   
   method Action put(FixedPoint#(ii1,if1) x, FixedPoint#(ii2,if2) y)
      if (token.value > 0);
      token.down;
      mult_raw.put(x,y);
      sfin1.wset(?);
   endmethod
   
   method ActionValue#(FixedPoint#(oi,of)) get();
      token.up;
      out_q.deq;
      return out_q.first;
   endmethod
   
endmodule
