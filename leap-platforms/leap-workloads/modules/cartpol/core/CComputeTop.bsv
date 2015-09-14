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

import ComputeTop::*;
import FIFO::*;
import FixedPointNew::*;
import Float::*;
import FloatToFixedPoint::*;
import Types::*;

Bool display_debug_info = False;

import "BDPI" cGetPos = 
       function Pos cGetPos(Bool    reset_val,
                            Float   rad,
                            Float   ang,
                            Index   n,
                            Index   r_idx,
                            Index c_idx);

(* synthesize *)
module mkCComputeTop(ComputeTop);
   
   FIFO#(Pos)  pos_q     <- mkFIFO;
   Reg#(Bool)  can_read  <- mkReg(True);
   Reg#(Index) n_reg     <- mkRegU;
   Reg#(Index) n_row_reg <- mkRegU;
   Reg#(Index) n_col_reg <- mkRegU;
   
   rule countDownFSM(!can_read);
      let enq_data = cGetPos(False,?,?,?,n_row_reg,n_col_reg);
      pos_q.enq(enq_data);
      if (n_row_reg == n_reg)
         if (n_col_reg == n_reg)
            can_read <= True;
         else
            begin
               n_col_reg <= n_col_reg + 1;
               n_row_reg <= 0;
            end
      else
         n_row_reg <= n_row_reg + 1;
      if (display_debug_info)
         begin
            $display("CComputeTop computePos: row_idx %d, col_idx %d, x %d, y %d",n_row_reg, n_col_reg, enq_data.x, enq_data.y);
         end
   endrule
   
   method Action setParam(Float rad, Float ang, Index n)
      if (can_read);
      //Data angD = FixedPoint{i:zeroExtend(ang.i), f:truncateLSB(ang.f)};
      let enq_data = cGetPos(True, rad, ang, n, 0, 0);
      Data  fixed_rad = fromMaybe(?,floatToFixedPoint(rad));
      TData fixed_ang = fromMaybe(?,floatToFixedPoint(ang));
      pos_q.enq(enq_data);
      can_read <= False;
      n_row_reg <= 1;
      n_col_reg <= 0;
      n_reg <= n-1;  
      if (display_debug_info)
         begin
            $write("CComputeTop fires setParam: rad ");
            fxptWrite(9,fixed_rad);
            $write(", ang ");
            fxptWrite(9,fixed_ang);
            $display(", n %d",n);
            $display("CComputeTop computePos: row_idx 0, col_idx 0, x %d, y %d",enq_data.x, enq_data.y);
         end
   endmethod
       
   method ActionValue#(Pos) getPos();
      pos_q.deq;
      return pos_q.first;
   endmethod
      
endmodule
