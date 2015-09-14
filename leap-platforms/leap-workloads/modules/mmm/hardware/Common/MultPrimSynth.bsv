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


interface Mult;
   (* always_ready, prefix="" *)
   method Bit#(16) mult(Bit#(16) x, Bit#(16) y);
endinterface

/*   
import "BVI" mult = module mkMult(Mult);
  method mult_fp mult(mult_fp_1, mult_fp_2);
endmodule
*/

module mkMult(Mult);

    method Mult(Bit#(16) x, Bit#(16) y);
   
       Bit#(33) ii = mult(x.rel, y.rel);
       Bit#(33) qq = mult(x.img, y.img);
   
       Bit#(33) iq = mult(x.rel, y.img);
       Bit#(33) qi = mult(x.img, y.rel);
      
       Bit#(16) ni = (ii-qq)[29:14];//rtruncate(ii-qq);
    
       Bit#(16) nq = (iq +qi)[29:14];
   
       return Complex16{
 	 i: ni,
 	 q: nq };
      
    endfunction
  

endmodule