//
// Copyright (c) 2014, Intel Corporation
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// Neither the name of the Intel Corporation nor the names of its contributors
// may be used to endorse or promote products derived from this software
// without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
import FIFO::*;
import FIFOF::*;
import SpecialFIFOs::*;
import Vector::*;

`include "awb/provides/matrix_multiply_common.bsh"
   
interface MULTIPLIER_IFC#(type t_DATA);
    method Action inputReq(t_DATA x, t_DATA y);
    method ActionValue#(t_DATA) getProductResp();
    method t_DATA peekProductResp();
endinterface


function Bit#(n) bitSignedMult(Bit#(n) x, Bit#(n) y);
    Int#(n) xx = unpack(x);
    Int#(n) yy = unpack(y);
    return truncate(pack(signedMul(xx, yy)));
endfunction

   
module mkComplexMultiplier(MULTIPLIER_IFC#(Complex#(Bit#(n))));
   
   FIFO#(Vector#(4,Bit#(n))) phase1ResultQ <- mkFIFO();
   FIFO#(Complex#(Bit#(n))) finalResultQ <- mkBypassFIFO();
   
   function Vector#(4,Bit#(n)) complexSignedMultPhase1(Complex#(Bit#(n)) x, Complex#(Bit#(n)) y);
       Vector#(4, Bit#(n)) m = newVector();
       m[0] = bitSignedMult(x.rel, y.rel);
       m[1] = bitSignedMult(x.img, y.img);
       m[2] = bitSignedMult(x.rel, y.img);
       m[3] = bitSignedMult(x.img, y.rel);
       return m;	
   endfunction

   rule complexSignedMultPhase2 (True);
       let m = phase1ResultQ.first();
       phase1ResultQ.deq();
       finalResultQ.enq(cmplx((m[0]-m[1]),(m[2]+m[3])));	
   endrule

   method Action inputReq(Complex#(Bit#(n)) x, Complex#(Bit#(n)) y);
       phase1ResultQ.enq(complexSignedMultPhase1(x, y));
   endmethod

   method ActionValue#(Complex#(Bit#(n))) getProductResp();
       let r = finalResultQ.first();
       finalResultQ.deq();
       return r;
   endmethod

   method Complex#(Bit#(n)) peekProductResp();
       return finalResultQ.first();
   endmethod

endmodule   
   
