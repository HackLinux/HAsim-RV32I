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

`include "awb/provides/fpga_components.bsh"
`include "asim/provides/librl_bsv.bsh"
`include "awb/provides/soft_connections.bsh"
`include "awb/provides/soft_services.bsh"
`include "awb/provides/soft_services_lib.bsh"
`include "awb/provides/soft_services_deps.bsh"
`include "awb/provides/mmm_common.bsh"

typedef 1  NumLanes;
typedef 12 LogBlockSize;
typedef TAdd#(1, LogBlockSize) AddrSize; // Add a bit to handle overflow
typedef Bit#(AddrSize)	       Addr;

Addr nn = fromInteger(valueof(BlockSize));
Addr logN = fromInteger(log2(valueof(BlockSize)));

module mkFunctionalUnit (FunctionalUnit);

    FIFO#(FunctionalUnitCommand) instQ <- mkFIFO();

    //outputs
    FIFO#(ComplexWord) toMemQ          <- mkFIFO();
    FIFO#(ComplexWord) a2networkQ      <- mkFIFO();
    FIFO#(ComplexWord) b2networkQ      <- mkFIFO();
    FIFO#(ComplexWord) c2networkQ      <- mkFIFO();
  
    //inputs
    FIFO#(ComplexWord) fromMemQ        <- mkFIFO();
    FIFO#(ComplexWord) network2aQ      <- mkFIFO();
    FIFO#(ComplexWord) network2bQ      <- mkFIFO();
    FIFO#(ComplexWord) network2cQ      <- mkFIFO();

    Reg#(Addr) i <- mkReg(0);
    Reg#(Addr) j <- mkReg(0);  
    Reg#(Addr) k <- mkReg(0);  

    RegFile#(Addr, ComplexWord) a_cache <- mkRegFileFull();
    RegFile#(Addr, ComplexWord) b_cache <- mkRegFileFull();
    RegFile#(Addr, ComplexWord) c_cache <- mkRegFileFull();
    
    Reg#(Bool) isNetOpA <- mkRegU();
    Reg#(Bool) isNetOpB <- mkRegU();	 
    Reg#(Bool) isNetOpC <- mkRegU(); 
  
    Reg#(FunctionalUnitOp) cmd <- mkRegU();
  
  
    Reg#(Bool) idle <- mkReg(True);
  
    let nnp = case(cmd)
	       Multiply,
	       MultiplyAddAccumulate,
	       MultiplySubAccumulate: return (nn);
	       default: return(1);
	     endcase;
    
    Stmt compute = 
      seq 
        $display("SimpFuncUnit: start compute.");
        for(i <= 0; i < nn; i <= i + 1)
	seq for(j <= 0; j < nn; j <= j + 1)
	  seq for(k <= 0; k < nnp; k <= k + 1)
	    seq action
		  let a_val  = a_cache.sub((i<<logN) + k);
		  let b_val  = b_cache.sub((k<<logN) + j);
		  let m_val  = a_val*b_val;
		  let c_val  = (cmd == Multiply || cmd == Zero) && (k == 0) ? 0 : c_cache.sub((i<<logN) + j);
		  let nc_val = case (cmd)
				 Multiply,
				 MultiplyAddAccumulate: return (c_val + m_val);
				 MultiplySubAccumulate: return (c_val - m_val);
				 Zero:	                return (0);
				 AddAccumulate: return (c_val + a_val);
				 SubAccumulate: return (c_val - a_val);
			       endcase;
                  //$display("SimpFuncUnit: compute(%d) a[%h]=%h b[%h]=%h c[%h]=%h", k, (i<<logN) + k, a_val, (k<<logN) + j, b_val, (i<<logN) + j, c_val);
                  //$display("SimpFuncUnit: compute c[%d][%d] (%h) = %h", i,j,c_cache.sub((i<<logN) + j), nc_val);
		  // $display("SimpFuncUnit: c_val = %h, nc_val = %h, a_val = %h, b_val = %h", c_val, nc_val, a_val, b_val);
		   c_cache.upd((i<<logN) + j, nc_val);
		endaction
	    endseq // end k for
	  endseq // end j for
	endseq // end i for
	$display("SimpFuncUnit: done compute");
      endseq; // end fsm

    FSM compute_fsm <- mkFSM(compute);
  
    Stmt transferFromA = 
      seq
        $display("SimpFuncUnit: start transferFromA");
	for(i <= 0; i < nn; i <= i + 1)
	  seq
	    for(j <= 0; j < nn; j <= j + 1)
	      seq
		action
		  let data = a_cache.sub((i<<logN) + j);
		  let fifo = (isNetOpA)? a2networkQ : toMemQ;
		  fifo.enq(data);
		endaction
	      endseq // end j for
	  endseq // end i for
    	  $display("SimpFuncUnit: done transferFromA");
      endseq; // end fsm

    FSM transferFromA_fsm <- mkFSM(transferFromA);

    Stmt transferFromB = 
      seq
        $display("SimpFuncUnit: start transferFromB");
	for(i <= 0; i < nn; i <= i + 1)
	  seq
	    for(j <= 0; j < nn; j <= j + 1)
	      seq
		action
		  let data = b_cache.sub((i<<logN) + j);
		  let fifo = (isNetOpB)? b2networkQ : toMemQ;
		  fifo.enq(data);
		endaction
	      endseq // end j for
	  endseq // end i for
    	  $display("SimpFuncUnit: done transferFromB");
      endseq; // end fsm

    FSM transferFromB_fsm <- mkFSM(transferFromB);

    Stmt transferFromC = 
      seq
        $display("SimpFuncUnit: start transferFromC");
	for(i <= 0; i < nn; i <= i + 1)
	  seq
	    for(j <= 0; j < nn; j <= j + 1)
	      seq
		action
		  let data = c_cache.sub((i<<logN) + j);
		  let fifo = (isNetOpC)? c2networkQ : toMemQ;
		  //$display("SimpFuncUnit: xfer C[%d][%d] => Mem %h", i,j,data);
		  fifo.enq(data);		  
		endaction
	      endseq // end j for
	  endseq // end i for
    	  $display("SimpFuncUnit: done transferFromC");
      endseq; // end fsm

    FSM transferFromC_fsm <- mkFSM(transferFromC);
  
    Stmt transferToA = 
      seq
        $display("SimpFuncUnit: start transferToA");
	for(i <= 0; i < nn; i <= i + 1)
	  seq
	    for(j <= 0; j < nn; j <= j + 1)
	      seq
		action
		  let fifo = (isNetOpA)? network2aQ : fromMemQ;
		  a_cache.upd((i<<logN) + j,fifo.first());
		  //$display("SimpFuncUnit: xfer Mem %h => C[%d][%d]", fifo.first(), i,j);
		  fifo.deq();
		endaction
	      endseq // end j for
	  endseq // end i for
    	  $display("SimpFuncUnit: done transferToA");
      endseq; // end fsm

    FSM transferToA_fsm <- mkFSM(transferToA);

    Stmt transferToB = 
      seq
        $display("SimpFuncUnit: start transferToB");
	for(i <= 0; i < nn; i <= i + 1)
	  seq
	    for(j <= 0; j < nn; j <= j + 1)
	      seq
		action
		  let fifo = (isNetOpB)? network2bQ : fromMemQ;
		  
		  b_cache.upd((i<<logN) + j,fifo.first());
		  fifo.deq();
		endaction
	      endseq // end j for
	  endseq // end i for
    	  $display("SimpFuncUnit: done transferToB");
      endseq; // end fsm

    FSM transferToB_fsm <- mkFSM(transferToB);

    Stmt transferToC = 
      seq
	$display("SimpFuncUnit: start transferToC");
	for(i <= 0; i < nn; i <= i + 1)
	  seq
	    for(j <= 0; j < nn; j <= j + 1)
	      seq
		action
		  let fifo = (isNetOpC)? network2cQ : fromMemQ;
		  c_cache.upd((i<<logN) + j,fifo.first());
		  fifo.deq();
		endaction
	      endseq // end j for
	  endseq // end i for
    	  $display("SimpFuncUnit: done transferToC");
      endseq; // end fsm

    FSM transferToC_fsm <- mkFSM(transferToC);
    
    Bool new_idle = 
      compute_fsm.done
      && transferFromA_fsm.done
      && transferFromB_fsm.done
      && transferFromC_fsm.done
      && transferToA_fsm.done
      && transferToB_fsm.done
      && transferToC_fsm.done;

    rule updateIdle(True);
      idle <= new_idle;
    endrule
    
    rule startOp (idle && new_idle);
    
      let ins = instQ.first();
      instQ.deq();
      
      case (ins) matches
	tagged ForwardSrc .rsrc:
	begin
         $display("SimpFuncUnit: FwdSrc %s" , showReg(rsrc));
	  case (rsrc)
	    A: begin isNetOpA <= True; transferFromA_fsm.start(); end
	    B: begin isNetOpB <= True; transferFromB_fsm.start(); end
	    C: begin isNetOpC <= True; transferFromC_fsm.start(); end
	  endcase
	end
	tagged ForwardDest .rdst:
	  begin
	    $display("SimpFuncUnit: FwdDest %s" , showReg(rdst));
	    case (rdst)
	      A: begin isNetOpA <= True; transferToA_fsm.start(); end
	      B: begin isNetOpB <= True; transferToB_fsm.start(); end
	      C: begin isNetOpC <= True; transferToC_fsm.start(); end
	    endcase
	  end
	tagged Load  .rgdst:
	  begin
	    $display("SimpFuncUnit: Load %s", showReg(rgdst));
	    case (rgdst)
	      A: begin isNetOpA <= False; transferToA_fsm.start(); end
	      B: begin isNetOpB <= False; transferToB_fsm.start(); end
	      C: begin isNetOpC <= False; transferToC_fsm.start(); end
	    endcase
	  end
	tagged Store .rgsrc:
	  begin
	    $display("SimpFuncUnit: Store %s", showReg(rgsrc));
	    case (rgsrc)
	      A: begin isNetOpA <= False; transferFromA_fsm.start(); end
	      B: begin isNetOpB <= False; transferFromB_fsm.start(); end
	      C: begin isNetOpC <= False; transferFromC_fsm.start(); end
	    endcase
	  end
	tagged Op .op:
	  begin
	    $display("SimpFuncUnit: Op %h", op);
	    cmd <= op;
	    compute_fsm.start();
	  end
      endcase
  
    endrule

  interface functionalUnitCommandInput = 
     interface Put#(FunctionalUnitCommand);
       method Action put(x); 
	 instQ.enq(x);
// 	 case (x) matches
//            tagged ForwardSrc  .fs : $display("FuncUnit: NewCommand - ForwardSrc %s" , showReg(fs));
//            tagged ForwardDest .fd : $display("FuncUnit: NewCommand - ForwardDest %s" , showReg(fd));
//            tagged Load         .l : $display("FuncUnit: NewCommand - Load" );
//            tagged Store        .s : $display("FuncUnit: NewCommand - Store" );
//            tagged Op           .o : $display("FuncUnit: NewCommand - Op" );
// 	 endcase
       endmethod
     endinterface;
    interface switchInput     = fifoToPut(fromMemQ);
    interface switchOutput    = interface Get;//fifoToGet(toMemQ);
				  method get();
				    actionvalue
				      //$display("FuncUnit: Sending %h", toMemQ.first());
				      toMemQ.deq();
				      return toMemQ.first();
				    endactionvalue
				  endmethod endinterface;
				  

    interface FUNetworkLink link;

    interface a_out = fifoToGet(a2networkQ);
    interface b_out = fifoToGet(b2networkQ);
    interface c_out = fifoToGet(c2networkQ);

    interface a_in = fifoToPut(network2aQ);
    interface b_in = fifoToPut(network2bQ);
    interface c_in = fifoToPut(network2cQ);

    endinterface

endmodule
