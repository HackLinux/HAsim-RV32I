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
// Author: Muralidaran Vijayaraghavan
//
//----------------------------------------------------------------------//

import Types::*;
import RegFile::*;
import FixedPointNew::*;

(* synthesize *)
module mkLutInv();
    rule disp;
        File f <- $fopen("lutInvTop.txt", "w");
        $fdisplay(f, "0");
        for(Integer i = 1; i <= 1000; i = i + 1)
        begin
            TData d = fromRational(1, i);
            $fdisplay(f, "%x", pack(d));
        end
        $finish;
    endrule
endmodule

interface InverseN;
   method Action put(Index maxN);
   method ActionValue#(TData) get();
endinterface

(* synthesize *)
module mkInverseN(InverseN);

   Reg#(TData) data <- mkRegU;
   
   RegFile#(Index, TData) lutInverse <- mkRegFileLoad("lutInvTop.txt", 0, 1000);
   
   method Action put(Index maxN);
      data <= lutInverse.sub(maxN);
   endmethod
      
   method ActionValue#(TData) get();
      return data;
   endmethod
endmodule
