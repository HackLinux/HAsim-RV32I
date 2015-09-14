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

///////////////////////////////////////////////////////////////////////////////////////////
//
// File: CORDIC.bsv
// Description: Provides cos, sin and arctan using CORDIC
// Author: Man C Ng, Email: mcn02@mit.edu
// Date: 26, June, 2006
// Last Modified: 10, Oct, 2006
//
//////////////////////////////////////////////////////////////////////////////////////////

import GetPut::*;
import Pipeline::*;

/////////////////////////////////////////////////////////////////////////////////////////
// Begin of Data Types

typedef struct
{ 
 Bool     bypass; // bypass data if exact result obtained already 
 data_t   x; 
 data_t   y;
 data_t   z;
 bypass_t u; // unchanged info through pipeline
 } CORDICData#(type data_t, type bypass_t) deriving (Bits, Eq);

typedef enum
{
 ADD = 0, // add
 SUB = 1, // subtract
 NOP = 2  // no op
 } CORDICMode deriving (Bits, Eq);

/////////////////////////////////////////////////////////////////////////////////////////
// Begin of Interfaces

interface CORDIC#(type data_t, type bypass_t);

   interface Put#(CORDICData#(data_t, bypass_t)) in;
   interface Get#(CORDICData#(data_t, bypass_t)) out;
      
endinterface

/////////////////////////////////////////////////////////////////////////////////////////
// Begin of Functions

function CORDICData#(data_t, bypass_t) executeStage(CORDICMode m, 
                                                    function CORDICMode getDelta(CORDICData#(data_t, bypass_t) data),
                                                    function data_t getEpsilon(Bit#(sz) stage),
                                                    Bit#(sz) init_left_shift,
                                                    Bit#(sz) in_stage,
                                                    CORDICData#(data_t, bypass_t) in_data)
   provisos (Bits#(bypass_t,bypass_sz),
             Bits#(data_t,data_sz),
             Log#(data_sz,sz),
             Arith#(data_t),
             Bitwise#(data_t)
             );
   
   data_t preproc_x  = in_data.x << init_left_shift;
   data_t preproc_y  = in_data.y << init_left_shift;
   data_t shifted_x  = preproc_x >> in_stage;
   data_t shifted_y  = preproc_y >> in_stage;
   CORDICMode delta  = getDelta(in_data);
   data_t epsilon    = getEpsilon(in_stage);
   Bool   is_add     = delta == ADD;
   Bool   is_sub     = delta == SUB;
   Bool   is_nop     = delta == NOP; 
   data_t tmp_add_x  = is_add ? shifted_y : negate(shifted_y); 
   data_t add_x      = case (m)
                          ADD: negate(tmp_add_x);
                          SUB: tmp_add_x;
                          NOP: 0;
                       endcase;               
   data_t add_y      = is_add ? shifted_x : negate(shifted_x);
   data_t add_z      = is_sub ? epsilon : negate(epsilon);
   data_t new_x      = is_nop ? in_data.x : in_data.x + add_x; 
   data_t new_y      = is_nop ? in_data.y : in_data.y + add_y;
   data_t new_z      = is_nop ? in_data.z : in_data.z + add_z;
   Bool   new_bypass = is_nop;
   
   return CORDICData{bypass:new_bypass, x:new_x, y:new_y, z:new_z, u:in_data.u};
                                             
endfunction

/////////////////////////////////////////////////////////////////////////////////////////
// Begin of Modules

module [m] mkCORDIC#(m#(Pipeline#(CORDICData#(data_t, bypass_t))) mkP)
              (CORDIC#(data_t,bypass_t)) 
provisos (IsModule#(m, a__));

   Pipeline#(CORDICData#(data_t, bypass_t)) p <- mkP(); 
   
   interface in  = p.in;
   interface out = p.out;
 
endmodule

// for making cordic pipeline
module  mkCORDIC_Pipe#(Integer numStages, 
                               Integer step,
                               CORDICMode m, 
                               function CORDICMode getDelta(CORDICData#(data_t, bypass_t) data),
                               function data_t getEpsilon(Bit#(sz) stage),
                               Bit#(sz) init_left_shift)
   (CORDIC#(data_t,bypass_t)) provisos (Bits#(bypass_t,bypass_sz),
                                        Bits#(data_t,data_sz),
                                        Log#(data_sz,sz),
                                        Add#(sz,k,32),
                                        Arith#(data_t),
                                        Bitwise#(data_t)
                                        );
   
   CORDIC#(data_t,bypass_t) cordic <- mkCORDIC(mkPipeline_Sync(numStages, step, executeStage(m,getDelta,getEpsilon,init_left_shift)));

   interface in  = cordic.in;
   interface out = cordic.out; 

endmodule                                             
   
// for making cir cordic 
module mkCORDIC_Circ#(Integer numStages, 
                               Integer step,
                               CORDICMode m, 
                               function CORDICMode getDelta(CORDICData#(data_t, bypass_t) data),
                               function data_t getEpsilon(Bit#(sz) stage),
                               Bit#(sz) init_left_shift)
   (CORDIC#(data_t,bypass_t)) provisos (Bits#(bypass_t,bypass_sz),
                                        Bits#(data_t,data_sz),
                                        Log#(data_sz,sz),
                                        Add#(sz,k,32),
                                        Arith#(data_t),
                                        Bitwise#(data_t)
                                        );
   
   CORDIC#(data_t,bypass_t) cordic <- mkCORDIC(mkPipeline_Circ(numStages, step, executeStage(m,getDelta,getEpsilon,init_left_shift)));

   interface in  = cordic.in;
   interface out = cordic.out; 

endmodule                                             
                                             
                                             










