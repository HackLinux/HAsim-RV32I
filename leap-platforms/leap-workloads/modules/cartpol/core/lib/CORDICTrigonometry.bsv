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
import Vector::*;


////////////////////////////////////////////////////////////////////////////////////////
// Begin of Data Types

// a pair of values representing the result of cos and sin to the same angle
typedef struct
{
  FixedPoint#(ai, af) cos;
  FixedPoint#(ai, af) sin;
 } CosSinPair#(type ai, type af) deriving (Bits, Eq);
 

/////////////////////////////////////////////////////////////////////////////////////////
// Begin of Interfaces

// aisz = no. of bits for the integer part of the input and output values, 
// afsz = no. of bits for the fractional part of the input and output values,
interface CosAndSin#(numeric type ai, numeric type af);
  method Action putAngle(FixedPoint#(ai,af) angle); // input angle
  method ActionValue#(CosSinPair#(ai,af)) getCosSinPair(); // return tuple2 (cos(x),sin(x))
endinterface

// aisz = no. of bits for the integer part of the input and output values, 
// afsz = no. of bits for the fractional part of the input and output values,
interface ATan#(numeric type ai, numeric type af);
  method Action putYX(FixedPoint#(ai,af) y, FixedPoint#(ai,af) x); // val of x, y coordinate
  method ActionValue#(FixedPoint#(ai,af)) getATan(); // return arcTan(y/x) 
endinterface

/////////////////////////////////////////////////////////////////////////////////////////
// Begin of Functions

function CORDICMode getCosAndSinDelta(CORDICData#(FixedPoint#(i,f),bypass_t) data);
   return data.bypass ? NOP: ((data.z < 0) ? SUB : ADD);     
endfunction

function CORDICMode getATanDelta(CORDICData#(FixedPoint#(i,f),bypass_t) data);
   return (data.y == 0 || data.bypass) ? NOP : ((data.y < 0) ? ADD : SUB);     
endfunction

function FixedPoint#(i,f) getTrigonometricEpsilon(Bit#(sz) stage);   
   Vector#(TExp#(sz),FixedPoint#(i,f)) atan_LUT = newVector;
   Real val = 1;
   for(Integer i = 0; i < valueOf(TExp#(sz)); i = i + 1)
      begin
         atan_LUT[i] = fromReal(atan2(val,1));
         val = val/2;
      end
   return atan_LUT[stage]; 
endfunction

/////////////////////////////////////////////////////////////////////////////////////////
// Begin of Modules

// CosAndSin, works only for 0<=angle<=pi/2
module  mkCORDICCosAndSin_Pipe#(Integer numStages,
                                        Integer steps)
   (CosAndSin#(i,f))
   provisos (Add#(xxA,2,i),
             Bits#(FixedPoint#(i,f),fp_sz),
             Log#(fp_sz,sz),
             Add#(sz,xxB,32)
             );

   CORDIC#(FixedPoint#(i,f),Bit#(0)) cordic;
   cordic <- mkCORDIC_Pipe(numStages, 
                           steps,
                           ADD,
                           getCosAndSinDelta,
                           getTrigonometricEpsilon,
                           0
                           );
   
   method Action putAngle(FixedPoint#(i,f) angle);
      cordic.in.put(CORDICData{bypass: False, 
                               x:0.6072529350088812561694,
                               y:0,
                               z:angle,
                               u:?});
   endmethod
   
   method ActionValue#(CosSinPair#(i,f)) getCosSinPair();
      CORDICData#(FixedPoint#(i,f),Bit#(0)) result <- cordic.out.get;
      return CosSinPair{cos: result.x, sin: result.y};
   endmethod

endmodule

module  mkCORDICCosAndSin_Circ#(Integer numStages,
                                        Integer steps)
   (CosAndSin#(i,f))
   provisos (Add#(xxA,2,i),
             Bits#(FixedPoint#(i,f),fp_sz),
             Log#(fp_sz,sz),
             Add#(sz,xxB,32)
             );

   CORDIC#(FixedPoint#(i,f),Bit#(0)) cordic;
   cordic <- mkCORDIC_Circ(numStages, 
                           steps,
                           ADD,
                           getCosAndSinDelta,
                           getTrigonometricEpsilon,
                           0
                           );
   
   method Action putAngle(FixedPoint#(i,f) angle);
      cordic.in.put(CORDICData{bypass: False, 
                               x:0.6072529350088812561694,
                               y:0,
                               z:angle,
                               u:?});
   endmethod
   
   method ActionValue#(CosSinPair#(i,f)) getCosSinPair();
      CORDICData#(FixedPoint#(i,f),Bit#(0)) result <- cordic.out.get;
      return CosSinPair{cos: result.x, sin: result.y};
   endmethod

endmodule

// Atan, works only for both x and y > 0
module  mkCORDICATan_Pipe#(Integer numStages,
                                   Integer steps)
   (ATan#(i,f))
   provisos (Add#(xxA,3,i), // at least 3 bits for -pi<=ans<=pi
             Bits#(FixedPoint#(i,f),fp_sz),
             Log#(fp_sz,sz),
             Add#(sz,xxB,32)
             );

   CORDIC#(FixedPoint#(i,f),Bit#(0)) cordic;
   cordic <- mkCORDIC_Pipe(numStages, 
                           steps,
                           ADD,
                           getATanDelta,
                           getTrigonometricEpsilon,
                           0
                           );

   method Action putYX(FixedPoint#(i,f) y, FixedPoint#(i,f) x);
      cordic.in.put(CORDICData{bypass:False,
                               x: x,
                               y: y,
                               z: 0,
                               u: ?});
  endmethod
     
  method ActionValue#(FixedPoint#(i,f)) getATan();
     CORDICData#(FixedPoint#(i,f),Bit#(0)) result <- cordic.out.get;
     return result.z;     
  endmethod

endmodule
