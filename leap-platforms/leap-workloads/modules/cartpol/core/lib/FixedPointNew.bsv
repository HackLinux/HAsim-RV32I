// Copyright 2005 Bluespec, Inc.  All rights reserved.

import Real :: *;

//@ \subsubsection{FixedPoint}
//@ \index{FixedPoint@\te{FixedPoint} (package)|textbf}
//@
//@ The \te{FixedPoint} library package defines a type for
//@ representing fixed-point numbers and corresponding functions to
//@ operate and manipulate variables of this type.  
//@    
//@ A fixed-point number represents signed real numbers which
//@ have a fixed number of binary digits (bits) before and after the binary
//@ point. The type constructor for Fixed point number takes two 
//@ numeric types as argument, the first (isize) defines the number of bits
//@ to the left of the binary point (the integer part), while the
//@ second (fsize)
//@ defines the number of bits to the right of the binary point, the
//@ fractional part.
//@    
//@  The following data structure defines this type, while some
//@ utility functions provide the reading of the integer and
//@ fractional parts.
//@ # 5
typedef struct {
                Bit#(isize) i;
                Bit#(fsize) f;
                }
FixedPoint#(numeric type isize, numeric type fsize )
deriving( Eq ) ;

/* aik added these functions */
function FixedPoint#(i,f)  _fromUInternal ( UInt#(b) x )
   provisos ( Add#(i,f,b) );
   return unpack ( pack (x) );
endfunction

function  UInt#(b) _toUInternal ( FixedPoint#(i,f) x )
   provisos ( Add#(i,f,b) );
   return unpack ( pack (x) );
endfunction
/* end of aik additions */

function FixedPoint#(i,f)  _fromInternal ( Int#(b) x )
   provisos ( Add#(i,f,b) );
   return unpack ( pack (x) );
endfunction

function  Int#(b) _toInternal ( FixedPoint#(i,f) x )
   provisos ( Add#(i,f,b) );
   return unpack ( pack (x) );
endfunction

instance Bits#( FixedPoint#(i,f), b ) 
   provisos ( Add#(i,f,b) );
   
   function Bit#(b) pack (FixedPoint#(i,f) x);
      return { pack(x.i),pack(x.f) };
   endfunction
   function FixedPoint#(i,f) unpack ( Bit#(b) x );
      match {.i,.f} = split (x);
      return FixedPoint { i:i, f:f } ;
   endfunction
endinstance


//@ \index{fxptGetInt@\te{fxptGetInt} (fixed-point function)|textbf}
//@ Utility functions are provided to extract the integer and
//@ fractional parts.
//@ # 3
function Int#(isize) fxptGetInt ( FixedPoint#(isize, fsize) a );
   return unpack (a.i);
endfunction

//@ \index{fxptGetFrac@\te{fxptGetFrac} (fixed-point function)|textbf}
//@ # 2
function UInt#(fsize) fxptGetFrac ( FixedPoint#(isize, fsize) a );
   return unpack (a.f);
endfunction


//@
//@ The range of values, $v$, representable with a signed
//@ fixed-point number of type \te{FixedPoint\#(isize, fsize)} is
//@ $+(2^{isize -1} - 2^{-fsize}) \leq v \leq -2^{isize - 1}$.
//@ This range is provided by the members of 
//@  \te{Bounded} type class to which \te{FixedPoint} belongs. The
//@ function {\tt epsilon} returns the smallest representable quantum
//@ by a specific type, $2^{-fsize}$.
//@ For example, a variable $v$ of type \te{FixedPoint\#(2,3)} type can
//@ represent numbers from 3.875 ($3\frac{7}{8}$) to $-4.0$ in
//@ intervals of $\frac{1}{8} = 0.125$. The type
//@ \te{FixedPoint\#(5,0)} is equivalent to \te{Int\#(5)}. 
//@ # 1 
instance Bounded#( FixedPoint#(i,f) )
   provisos( Add#(i,f,b) );
   
   function  FixedPoint#(i,f) minBound() ;
      Int#(b)  minb = minBound ;
      return _fromInternal (minb) ;
   endfunction

   function  FixedPoint#(i,f) maxBound() ;
      Int#(b)  maxb = maxBound ;
      return _fromInternal (maxb);
   endfunction
endinstance


instance RealLiteral#( FixedPoint# (i, f) );
   
function FixedPoint#(i,f) fromReal ( Real n )
   provisos ( 
             Add#(i,f,fpsize)
             ,Add#(54,fpsize,workingSize)
             );
   
   let {s,m,e} =  ( decodeReal (n) ) ;
   Int#(workingSize) joined = fromInteger(m); 
   Int#(12) be = fromInteger(e);
   
   Int#(fpsize) bres = ? ;
   Bool overflow = ? ;
   let ediff = e+valueOf(f) ;
   if (ediff <= 0)
      begin
         // Shifting to 0 results in 0, this is ok
         Int#(workingSize) aligned = joined >> (0- ediff) ;
         bres = truncate (aligned);
         // Possible overflow on truncate
         overflow = ( signExtend(bres) != aligned );
      end
   else
      begin
         // overflow if data is shifted off left end 
         // or shifted to 0
         Int#(workingSize) aligned = joined << ediff ;
         bres = truncate (aligned); 
         overflow =  ( signExtend(bres) != aligned ) || (bres == 0 && joined != 0 ) ;
      end
   
   String bmsg = s ? "maxBound" : "minBound" ;
   String msg = ("Converting from RealLiteral '" +
                 realToString (n) + 
                 "' to type 'FixedPoint#(" + integerToString(valueOf(i)) +
                 "," + integerToString(valueOf(f)) +
                 ")' exceeds the FixedPoint range, replacing with " + bmsg + "."
                 ) ;
   FixedPoint#(i,f) res1 = _fromInternal ( bres ) ;
   FixedPoint#(i,f) bound = s ? maxBound : minBound ;
   FixedPoint#(i,f) res = overflow ? warning (msg, bound) : res1 ;
   return res ;
   
endfunction
endinstance


//@ \index{epsilon@\te{epsilon} (fixed-point function)|textbf}
//@ # 1
function FixedPoint#(i,f) epsilon () ;
      Int#(b)  eps = 'b01  ;
      return _fromInternal (eps);
endfunction


//@ The type \te{FixedPoint} belongs to the \te{Arith} type class,
//@ hence the common infix operators (+, -, and *) and can be used to
//@ manipulate variables of type \te{FixedPoint}. 
//@ # 3
instance Arith#( FixedPoint#(i,f) )
   provisos( Add#(i,f,b) );

   // Addition does not change the binary point
   function \+ (FixedPoint#(i,f) in1, FixedPoint#(i,f) in2 );
       return _fromInternal ( _toInternal (in1) + _toInternal (in2) ) ;
   endfunction

   // Similar subtraction does not as well
   function \- (FixedPoint#(i,f) in1, FixedPoint#(i,f) in2 );
       return _fromInternal ( _toInternal (in1) - _toInternal (in2) ) ;
   endfunction
   
   // For multiplication, the computation is accomplished in full
   // precision, and the result truncated to fit
   function FixedPoint#(i,f) \* (FixedPoint#(i,f) in1, FixedPoint#(i,f) in2 )
      provisos(Add#(b,b,doublesize) ) ;
      // define some double sized operands for computations
      Int#(b) a1 = _toInternal(in1);
      Int#(b) a2 = _toInternal(in2);
      Int#(doublesize) m1, m2, prod ;
   
      m1 = signExtend( a1 ) ;
      m2 = signExtend( a2 ) ;
      prod = m1 * m2 ;
      // truncate the fractional bit
      prod = prod >> fromInteger(valueOf(f)) ;
      // Now truncate the integer part before placing it into the structure.
      Int#(b) finalv = truncate( prod ) ;
      return _fromInternal (finalv);
   endfunction

   // negate is defined in terms of the subtraction operator.
   function negate (FixedPoint#(i,f) in1 );
      return _fromInternal ( 0 - _toInternal(in1) );
   endfunction

   // quotient is not defined for FixedPoint
   function FixedPoint#(i,f) \/ (FixedPoint#(i,f) in1, FixedPoint#(i,f) in2);
      return error ("The operator " + quote("/") +
                    " is not defined for " + quote("FixedPoint") + ".");
   endfunction

   // remainder is not defined for FixedPoint
   function FixedPoint#(i,f) \% (FixedPoint#(i,f) in1, FixedPoint#(i,f) in2);
      return error ("The operator " + quote("%") +
                    " is not defined for " + quote("FixedPoint") + ".");
   endfunction

   function FixedPoint#(i,f) abs( FixedPoint#(i,f) x);
      // to prevent the instance from being recursive,
      // we don't use "-" ("negate") directly on "x"
      FixedPoint#(i,f) neg_x = _fromInternal( 0 - _toInternal(x) ) ;
      Int#(b) t = _toInternal(x);
      return ( (t < 0) ? neg_x : x );
   endfunction

   function FixedPoint#(i,f) signum( FixedPoint#(i,f) x);
      Int#(i) t = unpack(x.i);
      Int#(i) r = signum(t);
      return FixedPoint { i: pack(r), f:0 } ;
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


//@ The type \te{FixedPoint} belongs to the \te{Literal} type class,
//@ which allows conversion from (compile-time) type \te{Integer} to type
//@ \te{FixedPoint}.  Note that only the integer part is assigned.
//@ # 3
instance Literal#( FixedPoint#(i,f) ) 
   provisos( Add#(i,f,b) );
   
   // A way to convert Integer constants to fixed points
   function FixedPoint#(i,f) fromInteger( Integer n) ;

      // Convert to explicit types to all for bounds checking 
      Int#(i) conv = fromInteger(n) ;
      return FixedPoint {i:pack(conv),
                         f:0 };
   endfunction
   function inLiteralRange(f,n);
      Int#(i) int_part = ?;
      return(inLiteralRange(int_part,n));
   endfunction
endinstance


 
 
//@ \index{fromInt@\te{fromInt} (fixed-point function)|textbf}
//@ \index{fromUInt@\te{fromUInt} (fixed-point function)|textbf}
//@ To convert run-time \te{Int} and \te{UInt} values to type
//@ \te{FixedPoint}, the following conversion functions are
//@ provided. Both of these functions invoke the necessary extension
//@ of the source operand.
//@ # 5
function FixedPoint#(ir,fr) fromInt( Int#(ia) inta )
   provisos ( Add#(ia, xxA, ir )          // ir >= ia
             ) ;
   
   Int#(ir)  temp = signExtend( inta ) ;
   return FixedPoint { i:pack(temp), f:0 } ;
endfunction


//@ # 5
function FixedPoint#(ir,fr) fromUInt( UInt#(ia) uinta )
   provisos ( Add#(ia,  1, ia1),          // ia1 = ia + 1 
              Add#(ia1,xxB, ir )         // ir >= ia1
             );
   Bit#(ia1) t1 = {1'b0, pack(uinta) };
   Bit#(ir)  temp = zeroExtend( t1 ) ;
   return FixedPoint { i:temp, f:0 } ;
endfunction


//@ Non-integer compile time constants may be specified by a rational
//@ number which is a ratio of two integers.  For example, one-third
//@ may be specified by {\tt fromRational(1,3);}, while {$\pi$} can be
//@ specified as {\tt fromRational( 31415926, 10000000); }.  
//@ #  2
function FixedPoint#(i,f) fromRational( Integer numerator, Integer denominator) 
   provisos ( //Add#(1, xxA, i )          // i >= 1
             Add#(i,f,b) );
   let zmsg = error( "FixedPoint::fromRational " +
                    "denominator cannot be zero." );

   let rmsg = error( "FixedPoint::fromRational " +
                     "The rational number: " +
                     fromInteger( numerator ) + " / " +
                     fromInteger (denominator) +
                    " cannot be represented in a FixedPoint#(" +
                    fromInteger( valueOf(i) ) + "," +
                    fromInteger( valueOf(f) ) + ")."
                    );
   
   Integer temp = numerator * (2 ** valueOf(f)) ;
   temp = (denominator != 0) ?  div( temp, denominator ) : zmsg ;

   Integer max = 2 ** (valueOf(i) + valueOf(f) - 1) ;
   Bool rangeOk = (temp < max) && (temp >= negate( max ) ) ;
   
   Int#(b) res = rangeOk ? fromInteger( temp ) : rmsg ;
   return _fromInternal(res);
 
endfunction


//@ \index{fxptMult@\te{fxptMult} (fixed-point function)|textbf}
//@ At times, a full precision multiplication may be required, where
//@ the result is sum of the field sizes of the operands.  Note that
//@ the operand do not have to be the same type (sizes), as is
//@ required for the infix multiplication (*) operator.
//@ # 5
function FixedPoint#(ri,rf)  fxptMult( FixedPoint#(ai,af) a,
                                       FixedPoint#(bi,bf) b  )
   provisos (Add#(ai,bi,ri)   // ri = ai + bi
             ,Add#(af,bf,rf)   // rf = af + bf
             ,Add#(ai,af,ab)
             ,Add#(bi,bf,bb)
             ,Add#(ab,bb,rb)
             ,Add#(ri,rf,rb)
            ) ;

   Int#(ab) ap = _toInternal(a);
   Int#(bb) bp = _toInternal(b);
   
   Int#(rb) prod = signedMul(ap, bp); // signed integer multiplication

   return _fromInternal(prod);
      
endfunction

/* aik added this function */
function FixedPoint#(ri,rf)  fxptUMult( FixedPoint#(ai,af) a,
                                        FixedPoint#(bi,bf) b  )
   provisos (Add#(ai,bi,ri)   // ri = ai + bi
             ,Add#(af,bf,rf)   // rf = af + bf
             ,Add#(ai,af,ab)
             ,Add#(bi,bf,bb)
             ,Add#(ab,bb,rb)
             ,Add#(ri,rf,rb)
            ) ;

   UInt#(ab) ap = _toUInternal(a);
   UInt#(bb) bp = _toUInternal(b);
   
   UInt#(rb) prod = unsignedMul(ap, bp); // signed integer multiplication

   return _fromUInternal(prod);
      
endfunction
/* end of aik additions */


//@ \index{fxptTruncate@\te{fxptTruncate} (fixed-point function)|textbf}
//@ {\tt fxptTruncate} is a general truncate function which converts
//@ variables to 
//@ \te{FixedPoint\#(ai,af)} to type \te{FixedPoint\#(ri,rf)}, where
//@ $ai \geq ri$ and $af \geq rf$.  This function truncates bit as
//@ appropriate from the most significant integer bits and the least
//@ significant fractional bits.  
//# 5
function FixedPoint#(ri,rf) fxptTruncate( FixedPoint#(ai,af) a )
   provisos( Add#(xxA,ri,ai),    // ai >= ri
             Add#(xxB,rf,af)    // af >= rf
            ) ;
   
   FixedPoint#(ri,rf) res = FixedPoint {i: truncate (a.i),
                                        f: truncateLSB (a.f) } ;
   return res;
   
endfunction


//@ \index{fxptSignExtend@\te{fxptSignExtend} (fixed-point function)|textbf}
//@ \index{fxptZeroExtend@\te{fxptZeroExtend} (fixed-point function)|textbf}
//@ {\tt fxptSignExtend} is a general sign extend function which
//@ converts variables of type
//@ \te{FixedPoint\#(ai,af)} to type \te{FixedPoint\#(ri,rf)}, where
//@ $ai \leq ri$ and $af \leq rf$.  The integer part is sign extended,
//@ while additional 0 bits are added to least significant end of the
//@ fractional part.
//# 5
function FixedPoint#(ri,rf) fxptSignExtend( FixedPoint#(ai,af) a )
   provisos( Add#(xxA,ai,ri),      // ri >= ai
             Add#(fdiff,af,rf)    // rf >= af
            )  ;
   return FixedPoint {i:signExtend(a.i),
                      f:{a.f,0} } ;
endfunction


//@ A general zero extend function is also provided.
//@ # 5
function FixedPoint#(ri,rf) fxptZeroExtend( FixedPoint#(ai,af) a )
   provisos( Add#(xxA,ai,ri),    // ri >= ai
             Add#(xxB,af,rf)    // rf >= af
            ) ;
   return FixedPoint {i: zeroExtend (a.i),
                      f: {a.f,0} };
endfunction


//@ In addition to equality and inequality comparisons,
//@ \te{FixedPoint} variables can be compared by the relational
//@ operators provided by the \te{Ord} type class. i.e., <, >, <=, and
//@ >=. 
//@ # 2
instance Ord#( FixedPoint#(i,f) ) 
   provisos( Add#(i,f,b) );

   function Bool \< (FixedPoint#(i,f) in1, FixedPoint#(i,f) in2 ) ;
      Int#(b) n1 = _toInternal(in1) ;
      Int#(b) n2 = _toInternal(in2) ;
      return n1 < n2 ;
   endfunction

   function Bool \<= (FixedPoint#(i,f) in1, FixedPoint#(i,f) in2 );
      Int#(b) n1 = _toInternal(in1) ;
      Int#(b) n2 = _toInternal(in2) ;
      return n1 <= n2;
   endfunction

   function Bool \> (FixedPoint#(i,f) in1, FixedPoint#(i,f) in2 );
      Int#(b) n1 = _toInternal(in1) ;
      Int#(b) n2 = _toInternal(in2) ;
      return n1 > n2 ;
   endfunction

   function Bool \>= (FixedPoint#(i,f) in1, FixedPoint#(i,f) in2 );
      Int#(b) n1 = _toInternal(in1) ;
      Int#(b) n2 = _toInternal(in2) ;
      return n1 >= n2 ;
   endfunction

endinstance


//@ Left and right shifts are provided for \te{FixedPoint} variables
//@ as part of the \te{Bitwise} type class.  Note that the shift right
//@ ({\verb^>>^}) function does an arithmetic shift, thus preserving the sign
//@ of the operand. Note that a right shift of 1 is equivalent to a
//@ division by 2, except when the operand is equal to $- {\tt
//@ epsilon}$.  The other 
//@ methods of \te{Bitwise} type class are not provided since they
//@ have no operational meaning on \te{FixedPoint} variables.
//@ # 2
instance Bitwise#( FixedPoint#(i,f) )
   provisos (Add#(i,f,b)
           );

   function FixedPoint#(i,f) \>> (FixedPoint#(i,f) in1, ix sftamt ) 
      provisos(PrimIndex#(ix, dx),PrimShiftIndex#(ix, a__));
      Int#(b) bitsin = _toInternal(in1) ;
      return _fromInternal ( bitsin >> sftamt );
   endfunction

   function FixedPoint#(i,f) \<< (FixedPoint#(i,f) in1, ix sftamt ) 
      provisos(PrimIndex#(ix, dx), PrimShiftIndex#(ix, a__));
      Int#(b) ini = _toInternal(in1);
      return _fromInternal ( ini << sftamt );
   endfunction

   // We do not provide the following since their operation are
   // meaningless w.r.t. FixedPoint variables
   function invert (in1);
      return error ("The invert operator is not defined for " +
                    quote ("FixedPoint") + ".");
   endfunction
   function \^~ (in1, in2);
      return error ("The operator " + quote("^~") +
                    " is not defined for " + quote("FixedPoint") + ".");
   endfunction
   function \~^ (in1, in2);
      return error ("The operator " + quote("~^") +
                    " is not defined for " + quote("FixedPoint") + ".");
   endfunction
   function \^ (in1, in2);
      return error ("The operator " + quote("^") +
                    " is not defined for " + quote("FixedPoint") + ".");
   endfunction
   function \| (in1, in2);
      return error ("The operator " + quote("|") +
                    " is not defined for " + quote("FixedPoint") + ".");
   endfunction
   function \& (in1, in2);
      return error ("The operator " + quote("&") +
                    " is not defined for " + quote("FixedPoint") + ".");
   endfunction
   
endinstance


//@ \index{fxptWrite@\te{fxptWrite} (fixed-point function)|textbf}
//@ Displaying \te{FixedPoint} values in a simple bit notation would
//@ result in a difficult to read pattern.  The following write
//@ utility function is provided to ease in their display.  Note
//@ that the use of this functions adds many multipliers and adders
//@ into the design which are only used for generating the 
//@ output and not the actual circuit.  
//@
//@ Displays a  \te{FixedPoint} value in a decimal format, where {\tt
//@ fwidth} give the number of digits to the right of the decimal
//@ point. {\tt fwidth} must be in the inclusive range of 0 to 10. The
//@ displayed result is truncated without rounding.
//@ # 2
function Action fxptWrite( Integer fwidth,
                           FixedPoint#(i,f) a ) 
   provisos( 
            Add#(i, f, b),
            Add#(33,f,ff)      // 33 extra bits for computations.  10^10
            );
   action
      Int#(i)           ipart = fxptGetInt( a ) ; 
      Bit#(f)           fpart = pack( fxptGetFrac( a ) ) ;

      // negate the fractional part if the integer part is less than 0
      if (( ipart < 0 ) && (fpart != 0))
         begin
            fpart = 0 - fpart ;
            ipart = unpack( pack(ipart) + 1 );
            if ( ipart == 0 )
               $write( "-0." ) ;
            else
               $write( "%0d.", ipart ) ;
         end
      else
         $write( "%0d.", ipart ) ;

      Integer dispWidth = fwidth ;
      if ( fwidth < 0 || fwidth > 10 )
         begin
            let msg = "FixedPoint::fxptWrite Function fwidth argument must satisfy: "
                            + "0 <=fwidth <= 10. " + fromInteger( fwidth ) +
                            " is out of range."  ;
            dispWidth = error( msg ) ; 
         end

      // Now for some machinery to generate the fractional part in a 
      // decimal notation
      Integer idx ;
      Integer position = 1 ;
      Bit#(ff) taken = 0 ;
      for ( idx = 0 ; idx < dispWidth; idx = idx + 1 )
         begin
            position = position * 10 ;
            Bit#(ff) tx = (zeroExtend(fpart) * fromInteger(position) )  >>  fromInteger( valueOf( f ) ) ;
            Bit#(ff) tx2 = tx - taken ;
            Bit#(ff)  digit = tx2 & 'h0f ; // get the least 4 bits
            taken = 10 * (taken +  digit ) ;
            $write( "%0d",  digit ) ;
         end

   endaction
endfunction


