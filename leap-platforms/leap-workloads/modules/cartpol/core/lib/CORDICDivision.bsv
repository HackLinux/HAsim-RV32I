//----------------------------------------------------------------------//
// The MIT License 
// 
// Copyright (c) 2007 Alfred Man Cheuk Ng, mcn02@mit.edu 
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
//----------------------------------------------------------------------//

import CORDIC::*;
import FixedPointNew::*;
import GetPut::*;
import Real::*;

Bool display_debug_info = False;

////////////////////////////////////////////////////////////////////////////////////////
// Begin of Interfaces

interface Division#(type data_t);
   method Action putYX(data_t y, data_t x); // return y/x
   method ActionValue#(data_t) getQuotient();
endinterface

/////////////////////////////////////////////////////////////////////////////////////////
// Begin of Functions

function CORDICMode getDelta(CORDICData#(FixedPoint#(i,f),bypass_t) data);
   return (data.y == 0 || data.bypass) ? NOP : ((data.y < 0) ? ADD : SUB);
//   return (data.y != 0) ? ((data.y > 0) ? SUB : ADD) : NOP;     
endfunction

function FixedPoint#(i,f) getEpsilon(FixedPoint#(i,f) max_val, Bit#(sz) stage);   
   return (max_val >> stage); 
endfunction

/////////////////////////////////////////////////////////////////////////////////////////
// Begin of Modules

// divison based on cordic, all x, y values are allowed
module  mkCORDICDivision_Pipe#(Integer numStages, 
                                       Integer steps)
   (Division#(FixedPoint#(i,f)))
   provisos (Add#(xxA,2,i), // i >= 2
             Add#(i,i,ix2),
             Bits#(FixedPoint#(i,f),fsz),
             Bits#(FixedPoint#(ix2,f),fxpt_sz),
             Log#(fxpt_sz,sz),
             Add#(sz,xxB,32)
             ); 
   
   FixedPoint#(i,f)   max_val  = unpack(1 << fromInteger(valueOf(fsz)-2));
   FixedPoint#(ix2,f) max_val2 = fxptSignExtend(max_val);
   Bit#(sz)           init_shf = fromInteger(valueOf(i)-2);  
//   CORDIC#(FixedPoint#(ix2,f),Bool) cordic;
   CORDIC#(FixedPoint#(ix2,f),Bit#(0)) cordic;
   cordic <- mkCORDIC_Pipe(numStages, 
                           steps,
                           NOP,
                           getDelta,
                           getEpsilon(max_val2),
                           init_shf
                           );
   
   method Action putYX(FixedPoint#(i,f) y, FixedPoint#(i,f) x);
//       Bool is_x_neg = x < 0;
//       Bool is_y_neg = y < 0;
//       let in_x = is_x_neg ? negate(x) : x;
//       let in_y = is_y_neg ? negate(y) : y;
//       cordic.in.put(CORDICData{bypass: False, x: fxptSignExtend(in_x), y: fxptSignExtend(in_y), z:0, u:(is_x_neg != is_y_neg)});   
      cordic.in.put(CORDICData{bypass: False, x: fxptZeroExtend(x), y: fxptZeroExtend(y), z:0, u: ?}); 
      if (display_debug_info)
         begin
            $write("init_shf %d max_val ", init_shf);
            fxptWrite(8,max_val2);
            $display("");
         end
   endmethod
   
   method ActionValue#(FixedPoint#(i,f)) getQuotient();
      let result <- cordic.out.get;
      return fxptTruncate(result.z);
//       let res = result.u ? negate(result.z) : result.z;
//       return fxptTruncate(res);
   endmethod
                                           
endmodule

// divison based on cordic, all x, y values are allowed
module  mkCORDICDivision_Circ#(Integer numStages, 
                                       Integer steps)
   (Division#(FixedPoint#(i,f)))
   provisos (Add#(xxA,2,i), // i >= 2
             Add#(i,i,ix2),
             Bits#(FixedPoint#(i,f),fsz),
             Bits#(FixedPoint#(ix2,f),fxpt_sz),
             Log#(fxpt_sz,sz),
             Add#(sz,xxB,32)
             ); 
   
   FixedPoint#(i,f)   max_val  = unpack(1 << fromInteger(valueOf(fsz)-2));
   FixedPoint#(ix2,f) max_val2 = fxptSignExtend(max_val);
   Bit#(sz)           init_shf = fromInteger(valueOf(i)-2);  
//   CORDIC#(FixedPoint#(ix2,f),Bool) cordic;
   CORDIC#(FixedPoint#(ix2,f),Bit#(0)) cordic;
   cordic <- mkCORDIC_Circ(numStages, 
                           steps,
                           NOP,
                           getDelta,
                           getEpsilon(max_val2),
                           init_shf
                           );
   
   method Action putYX(FixedPoint#(i,f) y, FixedPoint#(i,f) x);
//       Bool is_x_neg = x < 0;
//       Bool is_y_neg = y < 0;
//       let in_x = is_x_neg ? negate(x) : x;
//       let in_y = is_y_neg ? negate(y) : y;
//       cordic.in.put(CORDICData{bypass: False, x: fxptSignExtend(in_x), y: fxptSignExtend(in_y), z:0, u:(is_x_neg != is_y_neg)});
      cordic.in.put(CORDICData{bypass: False, x: fxptZeroExtend(x), y: fxptZeroExtend(y), z:0, u: ?}); 
      if (display_debug_info)
         begin
            $write("init_shf %d max_val ", init_shf);
            fxptWrite(8,max_val2);
            $display("");
         end
   endmethod
   
   method ActionValue#(FixedPoint#(i,f)) getQuotient();
      let result <- cordic.out.get;
      return fxptTruncate(result.z);
//       let res = result.u ? negate(result.z) : result.z;
//       return fxptTruncate(res);
   endmethod
                                           
endmodule
