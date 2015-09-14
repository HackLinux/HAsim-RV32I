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

import FIFO::*;
import FixedPointNew::*;
import Float::*;
import GetPut::*;
import Real::*;

// convert a float to fixed point, if the output overflow, output invalid
function Maybe#(FixedPoint#(i,f)) floatToFixedPoint(Float float)
   provisos (Add#(i,f,fp_sz),
             Log#(fp_sz,shf_sz),
             Add#(xxA,fp_sz,128),
             Add#(xxB,shf_sz,11));
   let sign_bit = float.s;
   let exponent = float.e;
   Bit#(53) tmp_mantissa = {1,float.m};
   Bit#(128) long_mantissa = {tmp_mantissa,0};
   Bit#(fp_sz) mantissa = truncateLSB(long_mantissa); 
   Integer int_i = valueOf(i);
   Integer int_f = valueOf(f);
   if (exponent > fromInteger(1021 + int_i)) // input value too big to be contained by the fixedpoint!
      return tagged Invalid;
   else
      if (exponent < fromInteger(1023 - int_f)) // zero
         return tagged Valid 0;
      else
         begin
            Bit#(shf_sz) right_shift_amount = truncate(fromInteger(1022 + int_i) - exponent);
            let shifted_mantissa = mantissa >> right_shift_amount;
            if (sign_bit)
               shifted_mantissa = negate(shifted_mantissa);
            return tagged Valid unpack(shifted_mantissa);
         end
endfunction

interface FloatToFixedPoint#(numeric type i,numeric type f);
   interface Put#(Float) in;
   interface Get#(Maybe#(FixedPoint#(i,f))) out;
endinterface

module mkFloatToFixedPoint(FloatToFixedPoint#(i,f))
   provisos (Add#(i,f,fp_sz),
             Log#(fp_sz,shf_sz),
             Add#(xxA,fp_sz,128),
             Add#(xxB,shf_sz,11));
   
   FIFO#(Maybe#(FixedPoint#(i,f))) out_q <- mkFIFO;
   
   interface Put in;
      method Action put(Float x);
         out_q.enq(floatToFixedPoint(x));
      endmethod
   endinterface
   
   interface out = fifoToGet(out_q);
   
endmodule

module mkFloatToFixedPointInstance(FloatToFixedPoint#(14,40));
   let convert <- mkFloatToFixedPoint;
   return convert;
endmodule

module mkFloatToFixedPointTest(Empty);

   Reg#(Float) p <- mkReg(fromReal(-5.337));
   
   rule check;
      p <= p + (fromReal(0.42455) - (fromReal(0.3756) * fromReal(0.447)));
      Maybe#(FixedPoint#(4,48)) fp = floatToFixedPoint(p);
      if (isValid(fp))
         begin
            $write("Valid conversion: ");
            fxptWrite(9,fromMaybe(?,fp));
            $display(" bit representation %h",p);
            $display(" p > 0.5? %d",p>fromReal(0.5));
            $display(" p < 1.7? %d",p<fromReal(1.7));
         end
      else
         $display("Invalid conversion! Overflow! %h",p);
   endrule
   
endmodule
