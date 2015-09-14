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

import BRAMInitiatorWires::*;


interface BRAMModel#(type idx_type); 
  (* always_ready, always_enabled, prefix="" *) 
  method Action bramCLKA(Bit#(1) clk);

  (* always_ready, always_enabled, prefix="" *) 
  method Action bramRSTA(Bit#(1) rst);

  (* always_ready, always_enabled, prefix="" *) 
  method Action bramAddrA(idx_type idx);

  (* always_ready, always_enabled, prefix="" *) 
  method Action bramDoutA(Bit#(32) value);

  (* always_ready, always_enabled, prefix="" *) 
  method Bit#(32) bramDinA();

  (* always_ready, always_enabled, prefix="" *) 
  method Action bramWENA(Bit#(4) wen);

  (* always_ready, always_enabled, prefix="" *) 
  method Action bramENA(Bit#(1) en);

  (* always_ready, always_enabled, prefix="" *) 
  method Action bramCLKB(Bit#(1) clk);

  (* always_ready, always_enabled, prefix="" *) 
  method Action bramRSTB(Bit#(1) rst);

  (* always_ready, always_enabled, prefix="" *) 
  method Action bramAddrB(idx_type idx);

  (* always_ready, always_enabled, prefix="" *) 
  method Action bramDoutB(Bit#(32) value);

  (* always_ready, always_enabled, prefix="" *) 
  method Bit#(32) bramDinB();

  (* always_ready, always_enabled, prefix="" *) 
  method Action bramWENB(Bit#(4) wen);

  (* always_ready, always_enabled, prefix="" *) 
  method Action bramENB(Bit#(1) en);

endinterface

import "BVI" BRAMModel = module mkBRAMModelWires#(Clock clk1, Clock clk2, Reset rst1, Reset rst2)
  //interface:
              (BRAMModel#(idx_type)) 
  provisos
          (Bits#(idx_type, idx),
	   Literal#(idx_type));

  default_clock clk(CLK);

  input_clock clk1(clk1) = clk1 ; // put clock
  input_clock clk2(clk2) = clk2 ; // get clock

  input_reset rst1(rst_n1) = rst1;
  input_reset rst2(rst_n2) = rst2;

  parameter addr_width = valueof(idx);
  parameter addr_exp   = valueof(TExp#(idx));

  method bramCLKA(BRAM_CLKA) enable(DUMMYA0) clocked_by(clk1);
  method bramRSTA(BRAM_RSTA) enable(DUMMYA1) clocked_by(clk1);
  method bramAddrA(BRAM_AddrA) enable(DUMMYA2) clocked_by(clk1);
  method bramDoutA(BRAM_DoutA) enable(DUMMYA3) clocked_by(clk1);
  method BRAM_DinA bramDinA() clocked_by(clk1);
  method bramWENA(BRAM_WENA) enable(DUMMYA4) clocked_by(clk1);
  method bramENA(BRAM_ENA) enable(DUMMYA5) clocked_by(clk1);

  method bramCLKB(BRAM_CLKB) enable(DUMMYB0) clocked_by(clk2);
  method bramRSTB(BRAM_RSTB) enable(DUMMYB1) clocked_by(clk2);
  method bramAddrB(BRAM_AddrB) enable(DUMMYB2) clocked_by(clk2);
  method bramDoutB(BRAM_DoutB) enable(DUMMYB3) clocked_by(clk2) ;
  method BRAM_DinB bramDinB() clocked_by(clk2);
  method bramWENB(BRAM_WENB) enable(DUMMYB4) clocked_by(clk2);
  method bramENB(BRAM_ENB) enable(DUMMYB5) clocked_by(clk2);


  // All the BRAM methods are CF
  schedule bramCLKA CF (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB);
  schedule bramRSTA CF  (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB);
  schedule bramAddrA CF (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB); 
  schedule bramDoutA CF (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB); 
  schedule bramDinA CF (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB); 
  schedule bramWENA CF  (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB);
  schedule bramENA CF  (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB);


  schedule bramCLKB CF (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB);
  schedule bramRSTB CF  (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB);
  schedule bramAddrB CF (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB); 
  schedule bramDoutB CF (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB); 
  schedule bramDinB CF (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB); 
  schedule bramWENB CF  (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB);
  schedule bramENB CF  (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB);

endmodule

module mkBRAMModel#(Clock clk1, Reset rst1, Clock clk2, Reset rst2, BRAMInitiatorWires#(idx_type) bramInitiatorA,BRAMInitiatorWires#(idx_type) bramInitiatorB) ()
  provisos
          (Bits#(idx_type, idx),
	   Literal#(idx_type));

  BRAMModel#(idx_type) bram <- mkBRAMModelWires(clk1, clk2, rst1, rst2); 
  
  rule tie_wiresA;
    bram.bramCLKA(bramInitiatorA.bramCLK);
    bram.bramRSTA(bramInitiatorA.bramRST);
    bram.bramAddrA(bramInitiatorA.bramAddr);
    bram.bramDoutA(bramInitiatorA.bramDout);
    bramInitiatorA.bramDin(bram.bramDinA);
    bram.bramWENA(bramInitiatorA.bramWEN);
    bram.bramENA(bramInitiatorA.bramEN);
  endrule
  
  rule tie_wiresB;
    bram.bramCLKB(bramInitiatorB.bramCLK);
    bram.bramRSTB(bramInitiatorB.bramRST);
    bram.bramAddrB(bramInitiatorB.bramAddr);
    bram.bramDoutB(bramInitiatorB.bramDout);
    bramInitiatorB.bramDin(bram.bramDinB);
    bram.bramWENB(bramInitiatorB.bramWEN);
    bram.bramENB(bramInitiatorB.bramEN);
  endrule


endmodule


//BRAMModel8

import "BVI" BRAMModel8 = module mkBRAMModel8Wires#(Clock clk1, Clock clk2, Reset rst1, Reset rst2)
  //interface:
              (BRAMModel#(Bit#(3)));

  default_clock clk(CLK);

  input_clock clk1(clk1) = clk1 ; // put clock
  input_clock clk2(clk2) = clk2 ; // get clock

  input_reset rst1(rst_n1) = rst1;
  input_reset rst2(rst_n2) = rst2;

  method bramCLKA(BRAM_CLKA) enable(DUMMYA0) clocked_by(clk1);
  method bramRSTA(BRAM_RSTA) enable(DUMMYA1) clocked_by(clk1);
  method bramAddrA(BRAM_AddrA) enable(DUMMYA2) clocked_by(clk1);
  method bramDoutA(BRAM_DoutA) enable(DUMMYA3) clocked_by(clk1);
  method BRAM_DinA bramDinA() clocked_by(clk1);
  method bramWENA(BRAM_WENA) enable(DUMMYA4) clocked_by(clk1);
  method bramENA(BRAM_ENA) enable(DUMMYA5) clocked_by(clk1);

  method bramCLKB(BRAM_CLKB) enable(DUMMYB0) clocked_by(clk2);
  method bramRSTB(BRAM_RSTB) enable(DUMMYB1) clocked_by(clk2);
  method bramAddrB(BRAM_AddrB) enable(DUMMYB2) clocked_by(clk2);
  method bramDoutB(BRAM_DoutB) enable(DUMMYB3) clocked_by(clk2) ;
  method BRAM_DinB bramDinB() clocked_by(clk2);
  method bramWENB(BRAM_WENB) enable(DUMMYB4) clocked_by(clk2);
  method bramENB(BRAM_ENB) enable(DUMMYB5) clocked_by(clk2);

  // All the BRAM methods are CF
  schedule bramCLKA CF (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB);
  schedule bramRSTA CF  (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB);
  schedule bramAddrA CF (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB); 
  schedule bramDoutA CF (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB); 
  schedule bramDinA CF (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB); 
  schedule bramWENA CF  (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB);
  schedule bramENA CF  (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB);


  schedule bramCLKB CF (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB);
  schedule bramRSTB CF  (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB);
  schedule bramAddrB CF (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB); 
  schedule bramDoutB CF (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB); 
  schedule bramDinB CF (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB); 
  schedule bramWENB CF  (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB);
  schedule bramENB CF  (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB);

endmodule

module mkBRAMModel8#(Clock clk1, Reset rst1, Clock clk2, Reset rst2, BRAMInitiatorWires#(Bit#(3)) bramInitiatorA,BRAMInitiatorWires#(Bit#(3)) bramInitiatorB) ();

  BRAMModel#(Bit#(3)) bram <- mkBRAMModel8Wires(clk1, clk2, rst1, rst2); 
  
  rule tie_wiresA;
    bram.bramCLKA(bramInitiatorA.bramCLK);
    bram.bramRSTA(bramInitiatorA.bramRST);
    bram.bramAddrA(bramInitiatorA.bramAddr);
    bram.bramDoutA(bramInitiatorA.bramDout);
    bramInitiatorA.bramDin(bram.bramDinA);
    bram.bramWENA(bramInitiatorA.bramWEN);
    bram.bramENA(bramInitiatorA.bramEN);
  endrule
  
  rule tie_wiresB;
    bram.bramCLKB(bramInitiatorB.bramCLK);
    bram.bramRSTB(bramInitiatorB.bramRST);
    bram.bramAddrB(bramInitiatorB.bramAddr);
    bram.bramDoutB(bramInitiatorB.bramDout);
    bramInitiatorB.bramDin(bram.bramDinB);
    bram.bramWENB(bramInitiatorB.bramWEN);
    bram.bramENB(bramInitiatorB.bramEN);
  endrule

endmodule



//BRAMModel128

import "BVI" BRAMModel128 = module mkBRAMModel128Wires#(Clock clk1, Clock clk2, Reset rst1, Reset rst2)
  //interface:
              (BRAMModel#(Bit#(7)));

  default_clock clk(CLK);

  input_clock clk1(clk1) = clk1 ; // put clock
  input_clock clk2(clk2) = clk2 ; // get clock

  input_reset rst1(rst_n1) = rst1;
  input_reset rst2(rst_n2) = rst2;

  method bramCLKA(BRAM_CLKA) enable(DUMMYA0) clocked_by(clk1);
  method bramRSTA(BRAM_RSTA) enable(DUMMYA1) clocked_by(clk1);
  method bramAddrA(BRAM_AddrA) enable(DUMMYA2) clocked_by(clk1);
  method bramDoutA(BRAM_DoutA) enable(DUMMYA3) clocked_by(clk1);
  method BRAM_DinA bramDinA() clocked_by(clk1);
  method bramWENA(BRAM_WENA) enable(DUMMYA4) clocked_by(clk1);
  method bramENA(BRAM_ENA) enable(DUMMYA5) clocked_by(clk1);

  method bramCLKB(BRAM_CLKB) enable(DUMMYB0) clocked_by(clk2);
  method bramRSTB(BRAM_RSTB) enable(DUMMYB1) clocked_by(clk2);
  method bramAddrB(BRAM_AddrB) enable(DUMMYB2) clocked_by(clk2);
  method bramDoutB(BRAM_DoutB) enable(DUMMYB3) clocked_by(clk2) ;
  method BRAM_DinB bramDinB() clocked_by(clk2);
  method bramWENB(BRAM_WENB) enable(DUMMYB4) clocked_by(clk2);
  method bramENB(BRAM_ENB) enable(DUMMYB5) clocked_by(clk2);

  // All the BRAM methods are CF
  schedule bramCLKA CF (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB);
  schedule bramRSTA CF  (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB);
  schedule bramAddrA CF (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB); 
  schedule bramDoutA CF (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB); 
  schedule bramDinA CF (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB); 
  schedule bramWENA CF  (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB);
  schedule bramENA CF  (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB);


  schedule bramCLKB CF (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB);
  schedule bramRSTB CF  (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB);
  schedule bramAddrB CF (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB); 
  schedule bramDoutB CF (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB); 
  schedule bramDinB CF (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB); 
  schedule bramWENB CF  (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB);
  schedule bramENB CF  (bramCLKA, bramRSTA, bramAddrA, bramDoutA, bramDinA, bramWENA, bramENA, bramCLKB, bramRSTB, bramAddrB, bramDoutB, bramDinB, bramWENB, bramENB);

endmodule

module mkBRAMModel128#(Clock clk1, Reset rst1, Clock clk2, Reset rst2, BRAMInitiatorWires#(Bit#(7)) bramInitiatorA,BRAMInitiatorWires#(Bit#(7)) bramInitiatorB) ();

  BRAMModel#(Bit#(7)) bram <- mkBRAMModel128Wires(clk1, clk2, rst1, rst2); 
  
  rule tie_wiresA;
    bram.bramCLKA(bramInitiatorA.bramCLK);
    bram.bramRSTA(bramInitiatorA.bramRST);
    bram.bramAddrA(bramInitiatorA.bramAddr);
    bram.bramDoutA(bramInitiatorA.bramDout);
    bramInitiatorA.bramDin(bram.bramDinA);
    bram.bramWENA(bramInitiatorA.bramWEN);
    bram.bramENA(bramInitiatorA.bramEN);
  endrule
  
  rule tie_wiresB;
    bram.bramCLKB(bramInitiatorB.bramCLK);
    bram.bramRSTB(bramInitiatorB.bramRST);
    bram.bramAddrB(bramInitiatorB.bramAddr);
    bram.bramDoutB(bramInitiatorB.bramDout);
    bramInitiatorB.bramDin(bram.bramDinB);
    bram.bramWENB(bramInitiatorB.bramWEN);
    bram.bramENB(bramInitiatorB.bramEN);
  endrule

endmodule

