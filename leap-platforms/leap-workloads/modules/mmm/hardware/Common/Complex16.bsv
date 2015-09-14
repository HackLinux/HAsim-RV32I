/*
Copyright (c) 2007 MIT

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

Author: Kermin Fleming
*/

import FixedPoint::*;
import Vector::*;
import MultPrim::*;
import Complex::*;


typedef Complex#(Bit#(16)) Complex16;

/*
typedef struct{
  Bit#(16) i; // top bit is sign, one bit int
  Bit#(16) q;
}
 Complex16 deriving(Eq, Bits);*/

function Int#(n) toInt(Bit#(n) x)= unpack(x);
function Bit#(n) toBit(Int#(n) x)= pack(x);

/*
instance Literal#(Complex16);
  function Complex16 fromInteger(Integer x);
    return Complex16{
	     i: fromInteger(x),
	     q: 0     
	    };
  endfunction
endinstance

instance Bounded#(Complex16);
  function Complex16 minBound();
    Int#(16) mb = minBound;
    return Complex16{i: toBit(mb),q: toBit(mb) };
  endfunction

  function Complex16 maxBound();
    Int#(16) mb = maxBound;
    return Complex16{i: toBit(mb),q: toBit(mb)};
  endfunction
endinstance
  
  
  function Bit#(m) rzeroExtend(Bit#(n) a) provisos(Add#(n,k,m));
    return({a,0});
  endfunction
        
  function Bit#(m) rtruncate(Bit#(n) i) provisos(Add#(k,m,n));
    match {.top,.bot} = split(i);
    return (top);
  endfunction

 
//   function Bit#(n2) mult(Bit#(n) a, Bit#(n) b) provisos(Add#(n,k,n2));
//     Int#(n2) ap = signExtend(toInt(a));
//     Int#(n2) bp = signExtend(toInt(b));
//     Bit#(n2) cp = pack(ap*bp);

//     return cp;//[29:14];
//   endfunction
    
   (* noinline *)
   function FixedPoint#(2,14) mult_fp(FixedPoint#(2,14) a, FixedPoint#(2,14) b);
      return (a*b);
   endfunction
   
   function FixedPoint#(2,14) bits2fp(Bit#(16) x);
      return unpack(x);
   endfunction
						       
instance Arith#(Complex16);

  function Complex16 \+ (Complex16 x, Complex16 y);
     return Complex16{
              i: x.rel + y.rel, 
              q: x.img + y.img
             };
  endfunction     

  function Complex16 \- (Complex16 x, Complex16 y);
     return Complex16{
              i: x.rel - y.rel,
              q: x.img - y.img
             };
  endfunction

   // for synthesis
   function Complex16 \* (Complex16 x, Complex16 y);
   
      //using 4 muls and two adds
      //(a,b).(c,d) = (ac - bd,ad + bc);

      let ac = mult_fp(bits2fp(x.rel), bits2fp(y.rel));
      let bd = mult_fp(bits2fp(x.img), bits2fp(y.img));
      let ad = mult_fp(bits2fp(x.rel), bits2fp(y.img));
      let bc = mult_fp(bits2fp(x.img), bits2fp(y.rel));
        
      let i = ac - bd;
      let q = ad + bc;
   
      return Complex16{
	 i: pack(i),
	 q: pack(q)
	 };
      
   endfunction



//    // for simulation
//    function Complex16 \* (Complex16 x, Complex16 y);
   
//       Bit#(33) ii = mult(x.rel, y.rel);
//       Bit#(33) qq = mult(x.img, y.img);
   
//       Bit#(33) iq = mult(x.rel, y.img);
//       Bit#(33) qi = mult(x.img, y.rel);
      
//       Bit#(16) ni = (ii-qq)[29:14];//rtruncate(ii-qq);
    
//       Bit#(16) nq = (iq +qi)[29:14];
   
//       return Complex16{
// 	 i: ni,
// 	 q: nq };
      
//    endfunction
  
   function Complex16 negate (Complex16 x);
   return Complex16{
		    i: negate(x.rel),
		    q: negate(x.img)
	    };
  endfunction
  
  function Complex16 \% (Complex16 x,Complex16 y);
    return (error("no % for Complex16"));
  endfunction
  
  function Complex16 \/ (Complex16 x,Complex16 y);
    return (error("no / for Complex16"));
  endfunction
 
endinstance
*/

   interface MulFirst;
      method Vector#(4,Bit#(16)) doMult(Complex16 x, Complex16 y);
   endinterface
   
   module mkMulFirst(MulFirst);
      Mult mul1 <- mkMult();
      Mult mul2 <- mkMult();
      Mult mul3 <- mkMult();
      Mult mul4 <- mkMult();
      method Vector#(4,Bit#(16)) doMult(Complex16 x, Complex16 y);
	 Vector#(4, Bit#(16)) retval = newVector();
	 retval[0] = mul1.mult(x.rel, y.rel);
	 retval[1] = mul2.mult(x.img, y.img);
	 retval[2] = mul3.mult(x.rel, y.img);
	 retval[3] = mul4.mult(x.img, y.rel);
	 return retval;
      endmethod
   endmodule   
   
   function (Complex16) mul_second(Vector#(4,Bit#(16)) v);      
      return (Complex16{rel:pack(v[0]-v[1]),img:pack(v[2]+v[3])});	
   endfunction
   
   
