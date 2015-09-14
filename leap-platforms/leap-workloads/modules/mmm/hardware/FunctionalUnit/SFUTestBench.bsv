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

Author: Myron King
*/

import FIFO::*;
import Vector::*;
import StmtFSM::*;
import RegFile::*;
import GetPut::*;
import Connectable::*;

//Local imports
`include "Common.bsv"

import Complex16::*;
import SimpleFunctionalUnit::*;
import SimpleFUNetwork::*;
import FunctionalUnit::*;

module mkSFUTestBench(Empty);
   FunctionalUnit#(8) fu <- mkFunctionalUnit_STRIPPED8();  
   Reg#(int)      count  <- mkReg(0);
   Reg#(Complex16) v     <- mkReg(0);
   Reg#(Complex16) v2    <- mkReg(0);
   
//    rule inputData(count < 8192);
//       count <= count + 1;
//       $display("Count %d",count);
//       if (count == 0)
// 	 fu.functionalUnitCommandInput.put(tagged Load A);
//       if (count == 4096)
// 	 fu.functionalUnitCommandInput.put(tagged Load B);
//       if (count == 8191)
// 	 fu.functionalUnitCommandInput.put(tagged Op Multiply);
//       if (count < 4096)
// 	 begin
// 	    v <= fromInteger(8*1024);//v + 1;
// 	    fu.switchInput.put(v);
// 	    $display("pushing into A %d", count);     
// 	 end
//       else if (count < 8192)
// 	 begin
// 	    fu.switchInput.put(v2);
// 	    v2 <= fromInteger(8*1024);//v2 + 1;
// 	    $display("pushing into B %d", count);    
// 	 end    
//    endrule
   
   rule inputData(count < 1024);
      count <= count + 1;
      $display("Count %d",count);
      if (count == 0)
   	 fu.functionalUnitCommandInput.put(tagged ForwardDest A);
      if (count == 512)
   	 fu.functionalUnitCommandInput.put(tagged ForwardDest B);
      if (count == 1023)
   	 fu.functionalUnitCommandInput.put(tagged Op Multiply);
      
      if (count < 512)
   	 begin
   	    v <= v + fromInteger(8);
   	    fu.link.a_in.put(v);
   	    $display("pushing into A %d", count);     
   	 end
      else 
   	 begin
   	    fu.link.b_in.put(v2);
   	    v2 <= v2 + fromInteger(8);
   	    $display("pushing into B %d", count);    
   	 end
   endrule

   
//    rule outputData(count < (3*4096)+1);
//       count <= count + 1;
//       if (count == 8192)
// 	 fu.functionalUnitCommandInput.put(tagged Store C);
//       else
// 	 begin
//             let o <- fu.switchOutput.get();
//       	    $display("output: %b", o);     
// 	 end
//    endrule
   
   rule outputData(count < (3*512)+1);
      count <= count + 1;
      if (count == 1024)
         fu.functionalUnitCommandInput.put(tagged ForwardSrc C);
      else
         begin
            let o <- fu.link.c_out.get();
	    $display("output: %b", o);     
         end
   endrule
   
   rule inputDataA(count == (3*512)+1);
      $finish;
   endrule   
endmodule


// TODO change this to test mkFunctionalUnit, not mkSimpleFunctionalUnit
module mkSFUNetworkTestBench(Empty);
   
   Vector#(FunctionalUnitNumber, FunctionalUnit#(8)) fus <- replicateM(mkFunctionalUnit_STRIPPED8());
   let fu_links = map(getLink, fus);
   FUNetwork     funet <- mkSimpleNetwork(fu_links);

   Reg#(int)  cmd_count  <- mkReg(0);
   Reg#(int)      count  <- mkReg(0);
   Reg#(Complex16) v     <- mkReg(0);
   Reg#(Complex16) v2    <- mkReg(0);

   let fu1 = (fus[0]);
   let fu2 = (fus[1]);

   rule issue_commands(True);
      cmd_count <= cmd_count+1;
      if (cmd_count == 0)
	 fu1.functionalUnitCommandInput.put(tagged Load A);
      else if (cmd_count == 1)
	 fu1.functionalUnitCommandInput.put(tagged Load B);
      else if (cmd_count == 2)
	 fu1.functionalUnitCommandInput.put(tagged Op Multiply);
      else if (cmd_count == 3)
	 begin
            // foreward data from fu1's A to fu2's A
            fu1.functionalUnitCommandInput.put(tagged ForwardSrc A);
            fu2.functionalUnitCommandInput.put(tagged ForwardDest A);
	    funet.fuNetworkCommandInput.put(tagged FUNetworkCommand {fuSrc:0, regSrc:A, fuDests:1<<1, regDest:A});
	 end
      else if (cmd_count == 4)
	 begin
            // foreward data from fu1's B to fu2's B
            fu1.functionalUnitCommandInput.put(tagged ForwardSrc B);
            fu2.functionalUnitCommandInput.put(tagged ForwardDest B);
	    funet.fuNetworkCommandInput.put(tagged FUNetworkCommand {fuSrc:0, regSrc:B, fuDests:1<<1, regDest:B});
	 end
      else if (cmd_count == 5)
	 fu2.functionalUnitCommandInput.put(tagged Op Multiply);
      else if (cmd_count == 6)
	 begin
            fu1.functionalUnitCommandInput.put(tagged Store C);
            fu2.functionalUnitCommandInput.put(tagged Store C);
	 end 
   endrule

   rule inputData(count < 8192);
      count <= count + 1;    
      //$display("Count %d",count);
      if (count < 4096)
	 begin
	    v <= v+fromInteger(8);
	    fu1.switchInput.put(v);
	    //$display("pushing into A %d", count);     
	 end
      else if (count < 8192)
	 begin
	    fu1.switchInput.put(v2);
	    v2 <= v2+fromInteger(8);
	    //$display("pushing into B %d", count);    
	 end    
   endrule

   rule outputData(count < 3*4096);
      count <= count + 1;
      let o1 <- fu1.switchOutput.get();
      let o2 <- fu2.switchOutput.get();
      if(o1 != o2)
	 $display("error o1: %b o2: %b", o1, o2);
      else
	 $display("great success o1: %b o2: %b", o1, o2);	
   endrule
   
   rule inputDataA(count == 3*4096);
      $finish;
   endrule 
endmodule


// TODO: this module should test both isolated as well as network interractions
// TODO: especially an accumulate inst.
module mkFURegressionTestBench(Empty);   
   
   FunctionalUnit#(1)  sfu <- mkSimpleFunctionalUnit();
   FunctionalUnit#(8)  fu <- mkFunctionalUnit_STRIPPED8();
   Reg#(int) count_fu <- mkReg(0);
   Reg#(int) count_sfu <- mkReg(0);
   Reg#(int) cmd_count_fu <- mkReg(0);
   Reg#(int) cmd_count_sfu <- mkReg(0);
   Reg#(int) output_count <- mkReg(0);
   Reg#(Complex16) v_fu <- mkReg(0);
   Reg#(Complex16) v_sfu <- mkReg(0);
   
   rule inputData_fub(True);
      count_fu <= count_fu + 1;    
      v_fu <= v_fu+fromInteger(8);
      fu.switchInput.put(v_fu);
   endrule   

   rule inputData_sfu(True);
      count_sfu <= count_sfu + 1;    
      v_sfu <= v_sfu+fromInteger(8);
      sfu.switchInput.put(v_sfu);
   endrule      
   
   
   FunctionalUnitCommand instrs_small[6] = {
      (tagged Op Zero),	    
      (tagged Load B),	    
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Store C)
      };
   
   // 14 insts per pattern, 12 repetitions
   FunctionalUnitCommand instrs[14*12] = {
      (tagged Op Zero),	    
      (tagged Load B),	    
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Store C),	    
      (tagged Op Zero),	          
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Store C),
      (tagged Op Zero),	          
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Store C),
      (tagged Op Zero),	          
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Store C),
      (tagged Op Zero),	          
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Store C),
      (tagged Op Zero),	          
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Store C),
      (tagged Op Zero),	          
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Store C),
      (tagged Op Zero),	          
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Store C),
      (tagged Op Zero),	          
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Store C),
      (tagged Op Zero),	          
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Store C),
      (tagged Op Zero),	          
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Store C),
      (tagged Op Zero),	          
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Load B),
      (tagged Load A),
      (tagged Op MultiplyAddAccumulate),
      (tagged Store C)
      };
   
   rule issue_commands_fu(True);
      cmd_count_fu <= cmd_count_fu+1;
      for(int i = 0; i < 6; i=i+1)
	 begin
	    if(cmd_count_fu==i)
	       fu.functionalUnitCommandInput.put(instrs_small[i]);	    
	 end
   endrule

   rule issue_commands_sfu(True);
      cmd_count_sfu <= cmd_count_sfu+1;
      for(int i = 0; i < 6; i=i+1)
	 begin
	    if(cmd_count_sfu==i)
	       sfu.functionalUnitCommandInput.put(instrs_small[i]);	    
	 end
   endrule

   rule outputData(True);
      //$display("outputData, count=%h", count_fu);
      output_count <= output_count + 1;
      let o2 <- fu.switchOutput.get();
      let o4 <- sfu.switchOutput.get();
      if (o4 != o2)
	 $display("error gold = %h val = %h", o4, o2);
   endrule
   
   rule inputDataA(output_count == 1*4096);
      $finish;
   endrule 
   
endmodule


// code to test the regQ
// TODO: need to test simultaneous reading/writing to 
//       make sure we don't clobber data --mdk
module mkRegQTestBench(Empty);

   Reg#(int) count <- mkReg(0);
   Reg#(Complex16) v <- mkReg(0);
   Reg#(int) stage <- mkReg(15);
   RegFile#(Addr, ComplexWord) rfile <- mkRegFileFull();
   Reg#(Bool) written_col <- mkReg(False);
   RegQ#(4) regQ <- mkRegQ_STRIPPED();   
   
   function Addr cvt_int(int x);
      return truncate(unpack(pack(x)));
   endfunction   
   
   rule set_write_mode_row(stage==0);
      regQ.startWrite(WRowMajor);
      $display("set_write_mode_row");
      stage <= 1;
   endrule
   
   rule write_data_row_rfile(stage==1);
      rfile.upd(cvt_int(count), v);
      $display("write_data_row_rfile %d", count);
      if(count==4096-1)
	 begin
	    count <= 0;
	    stage <= 2;
	    v <= fromInteger(0);
	 end
      else
	 begin
	    count <= count + 1;    
	    v <= v+fromInteger(1);
	 end
   endrule

   rule write_data_row_regq(stage==2);
      Vector#(4, Complex16) vals = newVector();
      vals[0] = v + fromInteger(0);
      vals[1] = v + fromInteger(1);
      vals[2] = v + fromInteger(2);
      vals[3] = v + fromInteger(3);
      regQ.write(vals);
      $display("write_data_row_regq %d", count);
      if(count==1024-1)
   	 begin
   	    count <= 0;
   	    stage <= 3;
   	    v <= fromInteger(0);
   	 end
	 else
      begin
   	 count <= count + 1;    
   	 v <= v+fromInteger(4);
      end
   endrule
   
   rule set_read_mode_rowmajor(stage==3);
      regQ.startRead(RRowMajor);
      $display("set_read_mode_rowmajor");
      stage <= 4;
   endrule
   
   rule read_data_rowmajor(stage==4);
      Vector#(4, Complex16) blah <- regQ.read();
      let refval0 = rfile.sub(cvt_int((count*4)+0));
      let refval1 = rfile.sub(cvt_int((count*4)+1));
      let refval2 = rfile.sub(cvt_int((count*4)+2));
      let refval3 = rfile.sub(cvt_int((count*4)+3));      
      $display("read_data_rowmajor %d", count);
      if(blah[0]!=refval0)
	 $display("error [0] %d %d", refval0, blah[0]);
      if(blah[1]!=refval1)
	 $display("error [1] %d %d", refval1, blah[1]);
      if(blah[2]!=refval2)
	 $display("error [2] %d %d", refval2, blah[2]);
      if(blah[3]!=refval3)
	 $display("error [3] %d %d", refval3, blah[3]);
      if(count==(64*16)-1)
	 begin
	    count <= 0;
	    stage <= 5;
	 end
      else
	 count <= count+1;
   endrule
   

   rule set_read_mode_mmula(stage==5);
      regQ.startRead(RMatrixMultC);
      $display("set_read_mode_mmula");
      stage <= 6;
   endrule
   
   rule read_data_mmula(stage==6);
      Vector#(4, Complex16) blah <- regQ.read();
      int row = (count*4)/4096;
      int col = (count*4)%64;
      int idx = (row*64)+col;
      let refval0 = rfile.sub(cvt_int(idx+0));
      let refval1 = rfile.sub(cvt_int(idx+1));
      let refval2 = rfile.sub(cvt_int(idx+2));
      let refval3 = rfile.sub(cvt_int(idx+3));      
      $display("read_data_mmula %d", count);
      if(blah[0]!=refval0)
	 $display("error [0] %d %d", refval0, blah[0]);
      if(blah[1]!=refval1)
	 $display("error [1] %d %d", refval1, blah[1]);
      if(blah[2]!=refval2)
	 $display("error [2] %d %d", refval2, blah[2]);
      if(blah[3]!=refval3)
	 $display("error [3] %d %d", refval3, blah[3]);
      if(count==(64*64*16)-1)
	 begin
	    count <= 0;
	    stage <= 7;
	 end
      else
	 count <= count+1;
   endrule
   
   rule set_read_mode_mmulb(stage==7);
      regQ.startRead(RMatrixMultB);
      $display("set_read_mode_mmulb");
      stage <= 8;
   endrule
   
   // MMULB should be read off in COL major order
   rule read_data_mmulb(stage==8);
      Vector#(4, Complex16) blah <- regQ.read();
      int row = ((count*4)%4096)/64;
      int col = ((count*4)%64);
      int idx = (row*64)+(col);
      let refval0 = rfile.sub(cvt_int(idx+0));
      let refval1 = rfile.sub(cvt_int(idx+1));
      let refval2 = rfile.sub(cvt_int(idx+2));
      let refval3 = rfile.sub(cvt_int(idx+3));      
      $display("read_data_mmulb %d", count);
      if(blah[0]!=refval0)
	 $display("error [0] %d %d", refval0, blah[0]);
      if(blah[1]!=refval1)
	 $display("error [1] %d %d", refval1, blah[1]);
      if(blah[2]!=refval2)
	 $display("error [2] %d %d", refval2, blah[2]);
      if(blah[3]!=refval3)
	 $display("error [3] %d %d", refval3, blah[3]);
      if(count==(64*64*16)-1)
	 begin
	    count <= 0;
	    stage <= 9;
	 end
      else
	 count <= count+1;
   endrule
   
   rule set_read_mode_colmajor(stage==9);
      regQ.startRead(RColMajor);
      $display("set_read_mode_colmajor");
      stage <= 10;
   endrule
   
   rule read_data_col(stage==10);
      Vector#(4, Complex16) blah <- regQ.read();
      int row = count%64;
      int col = count/64;
      int idx = (row*64)+(col);
      let refval = rfile.sub(cvt_int(idx));
      $display("read_data_col %d", count);
      if(blah[0] != refval)
	 $display("error [0] %d %d", refval, blah[0]);
      if(count==4096-1)
	 begin
	    count <= 0;
	    stage <= 11;
       	 end
      else
	 count <= count + 1;    
   endrule   

   rule set_read_mode_rowscalar(stage==11);
      regQ.startRead(RRowMajorScalar);
      $display("set_read_mode_rowscalar");
      stage <= 12;
   endrule
   
   rule read_data_rowscalar(stage==12);
      Vector#(4, Complex16) blah <- regQ.read();
      let refval = rfile.sub(cvt_int(count));
      $display("read_data_rowscalar %d", count);
      if(blah[0] != refval)
	 $display("error [0] %d %d", refval, blah[0]);
      if(count==4096-1)
	 begin
	    if( written_col )
	       stage <= 15;
	    else
	       begin
		  count <= 0;
		  stage <= 13;
	       end
	 end
      else
	 count <= count + 1;    
   endrule
   
   
   rule set_write_mode_col(stage==13);
      regQ.startWrite(WColMajor);
      $display("set_write_mode_col");
      stage <= 14;
   endrule
   
   rule write_data_col(stage==14);
      v <= v+fromInteger(8);
      regQ.write(replicate(v));
      int row = count%64;
      int col = count/64;
      int idx = (row*64)+(col);
      rfile.upd(cvt_int(idx), v);
      $display("write_data_col %d", count);
      if(count==4096-1)
	 begin
	    count <= 0;
	    stage <= 3;
	    written_col <= True;
	 end
      else
	 count <= count + 1;    
   endrule
   

   rule set_write_and_read(stage==15);
      regQ.startRead(RMatrixMultC);
      regQ.startWrite(WMatrixMultC);
      $display("set_write_and_read");
      stage <= 16;
      v <= 0;
   endrule
   
   function val_t add( val_t x, val_t y) provisos (Arith#(val_t));
      return x+y;
   endfunction
   
   rule read_and_write_rfile(stage==16);
      $display("read_and_write_rfile count = %h", count);
      // each row has 64 elements and we write 
      // each row 64 times before moving on
      int row = count/(64*64);
      // column is just the low bits
      Bit#(6) col = truncate(pack(count));
      // the row is incremented every (64*64) cycles
      Bit#(6) row_iter = truncate(pack((count%(64*64)/64)));
      int idx = (row*64)+ unpack(zeroExtend(col));
      if(row_iter==0)
	 begin
	    $display("row_iter==0, row = %h, col = %h, idx = %h", row, col, idx);
	    rfile.upd(cvt_int(idx), 0);
	 end
      else
	 begin
	    let old_v = rfile.sub(cvt_int(idx));
	    rfile.upd(cvt_int(idx), old_v+v);
	    $display("row = %h, col = %h, idx = %h, old_val = %h", row, col, idx, old_v);
	 end
      
      if(count==(64*64*64)-1)
	 begin
	    count <=0;
	    stage <= 17;
	    v <= 0;
	 end
      else
	 begin
	    count <= count+1;
	    v <= v+fromInteger(1);
	 end
   endrule
   
   rule read_and_write_regq(stage==17);
      $display("read_and_write_regq count = %h", count);      
      // each row has 64 elements and we write 
      // each row 64 times before moving on
      int row = count/(64*64);
      
      // the row is incremented every (64*64) cycles
      Bit#(6) row_iter = truncate(pack((count%(64*64)/64)));
      
      if(row_iter==0)
	 begin
	    $display("row_iter==0, row = %h", row);
	    let old_v <- regQ.read();
	    regQ.write(replicate(0));
	 end
      else
	 begin
	    let old_v <- regQ.read();
	    
	    Vector#(4, Complex16) vals = newVector();
	    vals[0] = v + fromInteger(0);
	    vals[1] = v + fromInteger(1);
	    vals[2] = v + fromInteger(2);
	    vals[3] = v + fromInteger(3);
	    
	    regQ.write(zipWith(add, old_v, vals));
	    $display("row = %h, old_val = %h, %h, %h, %h", row, old_v[0], old_v[1], old_v[2], old_v[3]);
	 end
	    
      if(count==(64*64*64)-4)
	 begin
	    count <=0;
	    stage <= 18;
	 end
      else
	 begin
	    count <= count+4;
	    v <= v+fromInteger(4);
	 end
   endrule
   
   
   rule set_read_mode_rowscalara(stage==18);
      regQ.startRead(RRowMajorScalar);
      $display("set_read_mode_rowscalar");
      stage <= 19;   
   endrule
   
   rule read_data_rowscalara(stage==19);
      Vector#(4, Complex16) blah <- regQ.read();
      let refval = rfile.sub(cvt_int(count));
      $display("read_data_rowscalar %d", count);
      if(blah[0] != refval)
	 $display("error [0] %d %d", refval, blah[0]);
      if(count==4096-1)
	 begin
	    stage <= 20;
	    count <= 0;
	 end
      else
	 count <= count + 1;    
   endrule
   
   rule inputDataA(stage==20);
      $finish;
   endrule 
   
endmodule