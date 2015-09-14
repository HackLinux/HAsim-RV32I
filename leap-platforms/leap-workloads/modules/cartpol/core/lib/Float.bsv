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

import Real :: *;

typedef struct {
   Bool     s; // sign True = negative
   Bit#(11) e; // exponent
   Bit#(52) m; // mantissa
} Float deriving(Bits,Eq); // double precision floating point

import "BDPI" cAdd = 
       function Float cAdd(Float x,
                           Float y);
          
import "BDPI" cSub = 
       function Float cSub(Float x,
                           Float y);

import "BDPI" cMul = 
       function Float cMul(Float x,
                           Float y);
          
import "BDPI" cIsSmaller = 
       function Bool cIsSmaller(Float x,
                                Float y);

import "BDPI" cIsLarger = 
       function Bool cIsLarger(Float x,
                               Float y);
          
          
// make Float an instance of RealLiteral type case
instance RealLiteral#(Float);
   
   function Float fromReal(Real n);
      let {s,m,e} = (decodeReal(n));
      Bit#(54) mantissa = s ? fromInteger(m) : negate(fromInteger(m)); 
      Bit#(11) exponent = (e==0) ? 0 : fromInteger(e+1075);
      return Float{s: !s, e: exponent, m: mantissa[51:0]};
   endfunction
   
endinstance
   
// make Float an instance of Literal type case
instance Literal#(Float);

   function Float fromInteger(Integer v);
      Bool sign = v < 0;
      Bit#(53) val = sign ? fromInteger(0-v) : fromInteger(v);
      Integer msb_pos = 0;
      Bool msb_found = False;
      for (Integer n = 53; n > 0; n = n - 1)
         if (!msb_found)
            begin 
               if (val[52] == 1)
                  begin
                     msb_pos = n-1;
                     msb_found = True;
                  end
               val = val << 1;
            end
      Bit#(11) exponent = (v==0) ? 0 : fromInteger(1023+msb_pos);
      Bit#(52) mantissa = truncateLSB(val);
      return Float{s: sign, e: exponent, m: mantissa};
   endfunction

   function Bool inLiteralRange(Float targer, Integer x);
      return True; // all integer should be able to cover by double precision floating point
   endfunction
   
endinstance

// make Float an instance of Arith type case
instance Arith#(Float);
   
   // Addition does not change the binary point
   function Float \+ (Float in1, Float in2 );
       return cAdd(in1,in2) ;
   endfunction

   // Similar subtraction does not as well
   function Float \- (Float in1, Float in2 );
       return cSub(in1,in2) ;
   endfunction
   
   // For multiplication, the computation is accomplished in full
   // precision, and the result truncated to fit
   function Float \* (Float in1, Float in2 );
      return cMul(in1,in2);
   endfunction

   // negate is defined in terms of the subtraction operator.
   function negate (Float in1);
      return Float{s: !in1.s, e: in1.e, m: in1.m};
   endfunction

   // quotient is not defined for FixedPoint
   function Float \/ (Float in1, Float in2);
      return error ("The operator " + quote("/") +
                    " is not defined for " + quote("FixedPoint") + ".");
   endfunction

   // remainder is not defined for FixedPoint
   function Float \% (Float in1, Float in2);
      return error ("The operator " + quote("%") +
                    " is not defined for " + quote("FixedPoint") + ".");
   endfunction

   function Float abs( Float x);
      // to prevent the instance from being recursive,
      // we don't use "-" ("negate") directly on "x"
      return Float{s: False, e: x.e, m: x.m};
   endfunction

   function Float signum( Float x);
      return Float{s: x.s, e: 1023, m: 0};
   endfunction

   // Rather than use the defaults for the following functions
   // (which would mention the full type in the error message)
   // we use special versions that omit the type parameter and
   // just say "FixedPoint".

   function \** (x,y);
      return error ("The operator " + quote("**") + 
                    " is not defined for " + quote("FixedPoint") + ".");
   endfunction

   function exp_e(x);
      return error ("The function " + quote("exp_e") + 
                    " is not defined for " + quote("FixedPoint") + ".");
   endfunction

   function log(x);
      return error ("The function " + quote("log") + 
                    " is not defined for " + quote("FixedPoint") + ".");
   endfunction

   function logb(b,x);
      return error ("The function " + quote("logb") + 
                    " is not defined for " + quote("FixedPoint") + ".");
   endfunction

   function log2(x);
      return error ("The function " + quote("log2") + 
                    " is not defined for " + quote("FixedPoint") + ".");
   endfunction

   function log10(x);
      return error ("The function " + quote("log10") + 
                    " is not defined for " + quote("FixedPoint") + ".");
   endfunction

endinstance
          
// make Float an instance of Ord type case
instance Ord#( Float )
   provisos (Eq#(Float)); 

   function Bool \< (Float in1, Float in2 ) ;
      return cIsSmaller(in1,in2);
   endfunction

   function Bool \<= (Float in1, Float in2 );
      return cIsSmaller(in1,in2) || (in1==in2);
   endfunction

   function Bool \> (Float in1, Float in2 );
      return cIsLarger(in1,in2);
   endfunction

   function Bool \>= (Float in1, Float in2 );
      return cIsLarger(in1,in2) || (in1==in2) ;
   endfunction

endinstance
