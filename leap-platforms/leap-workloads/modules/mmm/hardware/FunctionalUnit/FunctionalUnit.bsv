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

`include "awb/provides/fpga_components.bsh"
`include "asim/provides/librl_bsv.bsh"
`include "awb/provides/soft_connections.bsh"
`include "awb/provides/soft_services.bsh"
`include "awb/provides/soft_services_lib.bsh"
`include "awb/provides/soft_services_deps.bsh"
`include "awb/provides/mmm_common.bsh"

import FIFO::*;
import FIFOF::*;
import Vector::*;
import StmtFSM::*;
import RegFile::*;
import GetPut::*;
import ConfigReg::*;
import FixedPoint::*;

//Local imports


typedef 6 LineSize;
typedef TMul#(2,LineSize) LogBlockSize;
typedef Bit#(TMul#(3,LineSize)) MatrixAddr;

typedef TAdd#(1, LogBlockSize) AddrSize; // Add a bit to handle overflow
typedef Bit#(AddrSize)	       Addr;
typedef Bit#(LogBlockSize)     MemAddr; // indexSize


typedef enum{
   RRowMajor,
   RColMajor,
   RMatrixMultC,
   RMatrixMultB,
   RRowMajorScalar
   } ReadAccessPattern deriving(Eq,Bits);


typedef enum{
   WRowMajorScalar,
   WRowMajor,
   WColMajor,
   WMatrixMultC
   } WriteAccessPattern deriving(Eq, Bits);

Addr maxAddr = (1 << valueOf(LogBlockSize));


MatrixAddr mat_nn = fromInteger(valueof(BlockSize));
Addr nn =           fromInteger(valueof(BlockSize));
Addr logN = fromInteger(log2(valueof(BlockSize)));



interface RegQ#(numeric type n); 
   method Action                              startRead(ReadAccessPattern x);
   method Action                              startWrite(WriteAccessPattern x);
   method ActionValue#(Vector#(n,Complex16))  read();
   method Action                              write(Vector#(n, Complex16) x);
   method Bool                                lastRead();
   method Bool                                doneRead();
   method Bool                                doneWrite();
endinterface

function Vector#(3,Bit#(LineSize)) splitMatrixAddr(MatrixAddr x);
   return unpack(pack(x));
endfunction

function Vector#(2,Bit#(LineSize)) splitMemAddr(MemAddr x);
   return unpack(pack(x));
endfunction

module mkRegQ(RegQ#(n)) provisos (Mul#(m,n,TExp#(LineSize)),
				  Log#(n, log_n),
				  Add#(log_n, bank_addr_nbits, LogBlockSize));  
   
   Vector#(n, MEMORY_IFC#(Bit#(bank_addr_nbits), Complex16)) brams <- replicateM(mkBRAM);
   
   
   FIFO#(Bool)        lastReadQ <- mkFIFO;
   FIFO#(Bit#(log_n)) lastBankQ <- mkFIFO; 
   
   Reg#(ReadAccessPattern) readAccessPattern <- mkRegU; 
   Reg#(WriteAccessPattern) writeAccessPattern <- mkRegU; 
   
   Reg#(MatrixAddr)   readPtr <- mkReg(0); // Goes from 0 to n^3-1 {i,j,k}
   Reg#(MemAddr)     writePtr <- mkReg(0); // Goes from 0 to n^2-1 {i,j}
   
   Reg#(Bool)      readParity <- mkReg(False);
   Reg#(Bool)     writeParity <- mkReg(False);

   Reg#(Bool)     readInProgress <- mkReg(False);
   Reg#(Bool)    writeInProgress <- mkReg(False);
   
   Bool readOlder = readParity != writeParity;
   
   let ival_n = fromInteger(valueOf(n));
   
   MemAddr inc_writePtr = writePtr + (case (writeAccessPattern)
					 WRowMajorScalar,
					 WColMajor: 1;
					 WRowMajor: ival_n;
				      endcase);
   
   MatrixAddr inc_readPtr = readPtr + (case (readAccessPattern)
				          RRowMajorScalar,
					  RColMajor: 1;
					  default: ival_n; 
				       endcase);

   MatrixAddr max_readPtr = case (readAccessPattern)
			       RRowMajorScalar,
			       RRowMajor,
			       RColMajor: return (mat_nn*mat_nn);
			       default:   return 0; // wraps around formerly (mat_nn*mat_nn*mat_nn)
			    endcase;
   
   // for i; for j; for k
   // the matrix address is [i|j|k] 
   let rindexs = splitMatrixAddr(readPtr);

   let windexs = splitMemAddr(writePtr);
   
   MemAddr readAddr = case (readAccessPattern)
			 RRowMajorScalar,
			 RRowMajor:   return truncate(readPtr);
			 RMatrixMultC: return {rindexs[2], rindexs[0]}; 
			 RMatrixMultB: return {rindexs[1], rindexs[0]}; 
			 RColMajor:   return {rindexs[0], rindexs[1]}; 
		      endcase;
   
   MemAddr writeAddr = case (writeAccessPattern)
			  WRowMajorScalar,
			  WRowMajor: return writePtr;
			  WColMajor: return {windexs[0], windexs[1]};
		       endcase;
   
   
   // TODO: these two Bools haven't been tested (we haven't yet verified simultaneous reading/writing to the same mkRegQ) --mdk
   Bool readPtr_past_writePtr = case (writeAccessPattern)
				   WRowMajorScalar,
				   WRowMajor: return (case (readAccessPattern)
							 RRowMajorScalar,
							 RRowMajor: (truncate(readPtr) > writePtr);
							 RMatrixMultC: ({rindexs[2],0} > writePtr);
						         RColMajor,
							 RMatrixMultB: False; 
						      endcase);
				   WColMajor: return (case (readAccessPattern)
							 RRowMajorScalar,
							 RRowMajor,
							 RMatrixMultC: False;
							 RColMajor:   (truncate(readPtr)>writePtr);
							 RMatrixMultB: ((rindexs[2]==maxBound)&&(truncate(readPtr) > writePtr));
						      endcase);
				endcase;
   
   Bool writePtr_past_readPtr = case (writeAccessPattern)
				   WRowMajorScalar,
				   WRowMajor: return (case (readAccessPattern)
							 RRowMajorScalar,
							 RRowMajor: (truncate(readPtr) < writePtr);
							 RMatrixMultC: ({rindexs[2],0} < writePtr);
						         RColMajor,
							 RMatrixMultB: False; 
						      endcase);
				   WColMajor: return (case (readAccessPattern)
							 RRowMajorScalar,
							 RRowMajor,
							 RMatrixMultC: False;
							 RColMajor:   (truncate(readPtr) < writePtr);
							 RMatrixMultB: ((rindexs[2]==maxBound)&&(truncate(readPtr) < writePtr));
						      endcase);
				endcase;
   


   Bool canWriteNext = (!readInProgress  || (!readOlder) || (readPtr_past_writePtr));
   Bool canReadNext =  (!writeInProgress || (readOlder)  || (writePtr_past_readPtr));
   
   function Bit#(bank_addr_nbits) bankAddr(MemAddr ra);
      Tuple2#(Bit#(bank_addr_nbits), Bit#(log_n)) b = split(ra);
      return tpl_1(b);
   endfunction

   function Bit#(log_n) bankName(MemAddr ra);
      return truncate(pack(ra));
   endfunction
   
   rule read_rule  (canReadNext && readInProgress);
      case (readAccessPattern)
	 RRowMajorScalar,
	 RColMajor:  begin
			(brams[bankName(readAddr)]).readReq(bankAddr(readAddr));
			lastBankQ.enq(bankName(readAddr));
			$display("reading from bank (col) %d at adder %h readAddr %h", bankName(readAddr), bankAddr(readAddr), readAddr);
		     end
	 default: begin
		     for(int i = 0; i < ival_n; i=i+1)
			begin
			   (brams[i]).readReq(bankAddr(readAddr));
			   $display("reading from bank %d at adder %h readAddr %h", i, bankAddr(readAddr), readAddr);
			end
		  end
      endcase
      readPtr<=(inc_readPtr == max_readPtr) ? 0 : inc_readPtr;
      readInProgress <= (inc_readPtr == max_readPtr) ? False : True;
      lastReadQ.enq(inc_readPtr == max_readPtr);
   endrule

   method Action startRead(x) if (!readInProgress);
      readAccessPattern <= x;
      readParity <= writeParity; // remember the write parity when we start
      readInProgress <= True;
      // not guaranteed to wrap around for every read pattern, therefore we need to reset it here
      readPtr <= 0;
   endmethod   
   
   method Action startWrite(x) if (!writeInProgress);
      if(x==WMatrixMultC)
	 $display("mkRegQ: can't handle WMatrixMultC, use mkRegQ_NEW instead");
      writeAccessPattern <= x;
      writeParity <= !writeParity;
      writeInProgress <= True;
   endmethod
   
   method ActionValue#(Vector#(n,Complex16)) read();
      Vector#(n,Complex16) rvec = newVector();
      case (readAccessPattern)
	 RRowMajorScalar,
	RColMajor: begin
		       let rr <- (brams[lastBankQ.first()]).readRsp();
		       rvec[0] = rr;
		       lastBankQ.deq();
		    end
	 default: begin
		    for(int i = 0; i < ival_n; i=i+1)
		      begin
                        let rr <-  (brams[i]).readRsp();
			rvec[i] = rr;
		      end
		  end
      endcase
      lastReadQ.deq();
      return rvec;
   endmethod
   
   method Action write(Vector#(n,Complex16) x) if (canWriteNext && writeInProgress);
      case (writeAccessPattern)
	 WRowMajorScalar,
	 WColMajor: (brams[bankName(writeAddr)]).write(bankAddr(writeAddr), x[0]);
	 default: begin
		     for(int i = 0; i < ival_n; i=i+1)
			begin
			   (brams[i]).write(bankAddr(writeAddr), x[i]);
			   $display("writing to bank %d at adder %h writeAddr %h", i, bankAddr(writeAddr), writeAddr);
			end
		  end

      endcase
      writePtr <= inc_writePtr;
      writeInProgress <= (inc_writePtr==0) ? False : True;
      if (inc_writePtr==0) $display("FuncUnit: write finished");
   endmethod
   
   method Bool lastRead(); 
      return lastReadQ.first();
   endmethod

   method Bool doneRead();
      return !readInProgress;
   endmethod

   method Bool doneWrite();
      return !writeInProgress;
   endmethod
   
endmodule


function MatrixAddr bits2MatrixAddr(MemAddr x);
   return unpack(zeroExtend(x));
endfunction

module mkRegQ_NEW(RegQ#(n)) provisos (Mul#(m,n,TExp#(LineSize)),
					Log#(n, log_n),
					Add#(log_n, bank_addr_nbits, LogBlockSize));  
   
   Vector#(n,  MEMORY_IFC#(Bit#(bank_addr_nbits), Complex16)) brams <- replicateM(mkBRAM);   
   
   // rather than have two fifos, we should mantain a seperate readReqPtr and readRespPtr --mdk
   FIFO#(Bool)        lastReadQ <- mkFIFO;
   FIFO#(Bit#(log_n)) lastBankQ <- mkFIFO; 
   
   Reg#(ReadAccessPattern) readAccessPattern <- mkRegU; 
   Reg#(WriteAccessPattern) writeAccessPattern <- mkRegU; 
   
   Reg#(MatrixAddr)   readPtr <- mkReg(0); // Goes from 0 to n^3-1 {i,j,k}
   Reg#(MatrixAddr)   writePtr <- mkReg(0);
   //Reg#(MemAddr)     writePtr <- mkReg(0); // Goes from 0 to n^2-1 {i,j}
   
   Reg#(Bool)      readParity <- mkReg(False);
   Reg#(Bool)     writeParity <- mkReg(False);

   Reg#(Bool)     readInProgress <- mkReg(False);
   Reg#(Bool)    writeInProgress <- mkReg(False);
   
   Bool readOlder = readParity != writeParity;
   
   let ival_n = fromInteger(valueOf(n));
   
   MatrixAddr inc_writePtr = writePtr + (case (writeAccessPattern)
					    WRowMajorScalar,
					    WColMajor: 1;
					    WRowMajor,
					    WMatrixMultC: ival_n;
					 endcase);
   
   MatrixAddr inc_readPtr = readPtr + (case (readAccessPattern)
				          RRowMajorScalar,
					  RColMajor: 1;
					  default: ival_n; 
				       endcase);

   MatrixAddr max_readPtr = case (readAccessPattern)
			       RRowMajorScalar,
			       RRowMajor,
			       RColMajor: return (mat_nn*mat_nn);
			       default:   return 0; // wraps around formerly (mat_nn*mat_nn*mat_nn)
			    endcase;

   MatrixAddr max_writePtr = case (writeAccessPattern)
				WMatrixMultC: return 0;//(mat_nn*mat_nn*(mat_nn-1));
				default: return (mat_nn*mat_nn);
			    endcase;
   
   // for i; for j; for k
   // the matrix address is [i|j|k] 
   let rindexs = splitMatrixAddr(readPtr);
   let windexs = splitMatrixAddr(writePtr);
   
   MemAddr readAddr = case (readAccessPattern)
			 RRowMajorScalar,
			 RRowMajor:   return truncate(readPtr);
			 RMatrixMultC: return {rindexs[2], rindexs[0]}; 
			 RMatrixMultB: return {rindexs[1], rindexs[0]}; 
			 RColMajor:   return {rindexs[0], rindexs[1]}; 
		      endcase;
   
   MemAddr writeAddr = case (writeAccessPattern)
			  WRowMajorScalar,
			  WRowMajor: return truncate(writePtr);
			  WMatrixMultC: return {windexs[2], windexs[0]};
			  WColMajor: return {windexs[0], windexs[1]};
		       endcase;
   
   
   MatrixAddr foo = bits2MatrixAddr({rindexs[2],0});
   Bool readPtr_past_writePtr = case (writeAccessPattern)
				   WRowMajorScalar,
				   WRowMajor: return (case (readAccessPattern)
							 RRowMajorScalar,
							 RRowMajor: (readPtr > writePtr);
							 RMatrixMultC: (foo > writePtr);
						         RColMajor,
							 RMatrixMultB: False; 
						      endcase);
				   WColMajor: return (case (readAccessPattern)
							 RRowMajorScalar,
							 RRowMajor,
							 RMatrixMultC: False;
							 RColMajor:   (readPtr>writePtr);
							 RMatrixMultB: ((rindexs[2]==maxBound)&&(readPtr > writePtr));
						      endcase);
				endcase;
   
   Bool writePtr_past_readPtr = case (writeAccessPattern)
				   WRowMajorScalar,
				   WRowMajor: return (case (readAccessPattern)
							 RRowMajorScalar,
							 RRowMajor: (readPtr < writePtr);
							 RMatrixMultC: (foo < writePtr);
						         RColMajor,
							 RMatrixMultB: False; 
						      endcase);
				   WColMajor: return (case (readAccessPattern)
							 RRowMajorScalar,
							 RRowMajor,
							 RMatrixMultC: False;
							 RColMajor:   (readPtr < writePtr);
							 RMatrixMultB: ((rindexs[2]==maxBound)&&(readPtr < writePtr));
						      endcase);
				endcase;
   
   // For simultaneous reads and wrotes if we are in WMatrixMultC, 
   // we had better be in RMatrixMultB or everything falls apart    
   Bool canWriteNext = case (writeAccessPattern)
			  // at the very end, writePtr needs to be able to catch up with readPtr
			  WMatrixMultC: return (!readInProgress  || (readPtr>writePtr)); 
			  default: return (!readInProgress  || (!readOlder) || (readPtr_past_writePtr));
		       endcase;
   
   
   // we need to do this to deal with overflow cruft;
   function Bool readPtr_lt_writePtr_plus_sixtyfour( MatrixAddr rp, MatrixAddr wp );
      Bit#(32) irp = zeroExtend(pack(rp));
      Bit#(32) iwp = zeroExtend(pack(wp));
      return (irp<iwp+64);
   endfunction
   
   Bool canReadNext =  case (writeAccessPattern)
			  WMatrixMultC: return (!writeInProgress || readPtr_lt_writePtr_plus_sixtyfour(readPtr, writePtr));
			  default: return (!writeInProgress || (readOlder)  || (writePtr_past_readPtr));
		       endcase;
			  
   function Bit#(bank_addr_nbits) bankAddr(MemAddr ra);
      Tuple2#(Bit#(bank_addr_nbits), Bit#(log_n)) b = split(ra);
      return tpl_1(b);
   endfunction

   function Bit#(log_n) bankName(MemAddr ra);
      return truncate(pack(ra));
   endfunction
   
   rule read_rule  (canReadNext && readInProgress);
      case (readAccessPattern)
	 RRowMajorScalar,
	 RColMajor:  begin
			(brams[bankName(readAddr)]).readReq(bankAddr(readAddr));
			lastBankQ.enq(bankName(readAddr));
			$display("reading from bank (col) %d at adder %h readAddr %h", bankName(readAddr), bankAddr(readAddr), readAddr);
		     end
	 default: begin
		     for(int i = 0; i < ival_n; i=i+1)
			begin
			   (brams[i]).readReq(bankAddr(readAddr));
			   $display("reading from bank %d at adder %h readAddr %h", i, bankAddr(readAddr), readAddr);
			end
		  end
      endcase
      readPtr<=(inc_readPtr == max_readPtr) ? 0 : inc_readPtr;
      readInProgress <= (inc_readPtr == max_readPtr) ? False : True;
      lastReadQ.enq(inc_readPtr == max_readPtr);
      $display("mkRegQ_NEW: read readPtr %h, inc_readPtr %h, max_readPtr %h", readPtr, inc_readPtr, max_readPtr);
   endrule

   method Action startRead(x) if (!readInProgress);
      readAccessPattern <= x;
      readParity <= writeParity; // remember the write parity when we start
      readInProgress <= True;
      // not guaranteed to wrap around for every read pattern
      readPtr <= 0; 
   endmethod   
   
   method Action startWrite(x) if (!writeInProgress);
      writeAccessPattern <= x;
      writeParity <= !writeParity;
      writeInProgress <= True;
      // we need to zero this since with new size it won't wrap around anymore
      writePtr <= 0; 
   endmethod
   
   method ActionValue#(Vector#(n,Complex16)) read();
      Vector#(n,Complex16) rvec = newVector();
      case (readAccessPattern)
	 RRowMajorScalar,
	RColMajor: begin
		       let rr <- (brams[lastBankQ.first()]).readRsp();
		       rvec[0] = rr;
		       lastBankQ.deq();
		    end
	 default: begin
		    for(int i = 0; i < ival_n; i=i+1)
		      begin
                        let rr <-  (brams[i]).readRsp();
			rvec[i] = rr;
		      end
		  end
      endcase
      lastReadQ.deq();
      return rvec;
   endmethod
   
   method Action write(Vector#(n,Complex16) x) if (canWriteNext && writeInProgress);
      case (writeAccessPattern)
	 WRowMajorScalar,
	 WColMajor: (brams[bankName(writeAddr)]).write(bankAddr(writeAddr), x[0]);
	 default: begin
		     for(int i = 0; i < ival_n; i=i+1)
			begin
			   (brams[i]).write(bankAddr(writeAddr), x[i]);
			   $display("writing to bank %d at adder %h writeAddr %h", i, bankAddr(writeAddr), writeAddr);
			end
		  end

      endcase
      writePtr <= inc_writePtr;
      $display("mkRegQ_NEW: write writePtr %h, inc_writePtr %h, max_writePtr %h", writePtr, inc_writePtr, max_writePtr);
      writeInProgress <= (inc_writePtr==max_writePtr) ? False : True;
   endmethod
   
   method Bool lastRead(); 
      return lastReadQ.first();
   endmethod

   method Bool doneRead();
      return !readInProgress;
   endmethod

   method Bool doneWrite();
      return !writeInProgress;
   endmethod
   
endmodule




module mkRegQ_STRIPPED(RegQ#(n)) provisos (Mul#(m,n,TExp#(LineSize)),
					   Log#(n, log_n),
					   Add#(log_n, bank_addr_nbits, LogBlockSize));  
   
   Vector#(n,  MEMORY_IFC#(Bit#(bank_addr_nbits), Complex16)) brams <- replicateM(mkBRAM);
   
   Reg#(ReadAccessPattern) readAccessPattern <- mkRegU; 
   Reg#(WriteAccessPattern) writeAccessPattern <- mkRegU; 
   
   Reg#(MatrixAddr)    writePtr <- mkConfigReg(0); // Goes from 0 to n^3-1 {i,j,k}
   Reg#(MatrixAddr)  readReqPtr <- mkConfigReg(0);
   Reg#(MatrixAddr) readRespPtr <- mkConfigReg(0);
   
   Reg#(Bool)      readParity <- mkReg(False);
   Reg#(Bool)     writeParity <- mkReg(False);

   Reg#(Bool)     readInProgress <- mkConfigReg(False);
   Reg#(Bool)    writeInProgress <- mkConfigReg(False);
   
   Bool readOlder = readParity != writeParity;
   
   let ival_n = fromInteger(valueOf(n));
   
   MatrixAddr inc_writePtr = writePtr + (case (writeAccessPattern)
					    WRowMajorScalar: 1;
					    WRowMajor,
					    WMatrixMultC: ival_n;
					    default: ?;
					 endcase);
   
   MatrixAddr inc_readReqPtr = readReqPtr + (case (readAccessPattern)
						RRowMajorScalar: 1;
						RMatrixMultB,
						RMatrixMultC: ival_n; 
						default: ?;
					     endcase);
   
   MatrixAddr inc_readRespPtr = readRespPtr + (case (readAccessPattern)
					      RRowMajorScalar: 1;
					      RMatrixMultB,
					      RMatrixMultC: ival_n; 
					      default: ?;
					   endcase);

   MatrixAddr max_readPtr = case (readAccessPattern)
			       RRowMajorScalar: return (mat_nn*mat_nn);
			       RMatrixMultC,
			       RMatrixMultB:   return 0; // wraps around formerly (mat_nn*mat_nn*mat_nn)
			       default: ?;
			    endcase;

   MatrixAddr max_writePtr = case (writeAccessPattern)
				WMatrixMultC: return 0;//(mat_nn*mat_nn*(mat_nn-1));
				WRowMajor,
				WRowMajorScalar: return (mat_nn*mat_nn);
			    endcase;
   
   // for i; for j; for k
   // the matrix address is [i|j|k] 
   let rindexs_req = splitMatrixAddr(readReqPtr);
   let rindexs_resp = splitMatrixAddr(readRespPtr);
   let windexs = splitMatrixAddr(writePtr);
   
   MemAddr readReqAddr = case (readAccessPattern)
			    RRowMajorScalar:   return truncate(readReqPtr);
			    RMatrixMultC: return {rindexs_req[2], rindexs_req[0]}; 
			    RMatrixMultB: return {rindexs_req[1], rindexs_req[0]}; 
			    default:   return ?;
			 endcase;
   
   MemAddr readRespAddr = case (readAccessPattern)
			     RRowMajorScalar:   return truncate(readRespPtr);
			     RMatrixMultC: return {rindexs_resp[2], rindexs_resp[0]}; 
			     RMatrixMultB: return {rindexs_resp[1], rindexs_resp[0]}; 
			     default:   return ?;
			  endcase;
   
   MemAddr writeAddr = case (writeAccessPattern)
			  WRowMajorScalar,
			  WRowMajor: return truncate(writePtr);
			  WMatrixMultC: return {windexs[2], windexs[0]};
			  default: return ?;
		       endcase;
   
   
   MatrixAddr foo = bits2MatrixAddr({rindexs_req[2],0});
   Bool readPtr_past_writePtr = case (writeAccessPattern)
				   WRowMajorScalar,
				   WRowMajor: return (case (readAccessPattern)
							 RRowMajorScalar: (rindexs_req[1] > windexs[1]);
				      			 //RRowMajorScalar: (readReqPtr > writePtr);
							 RMatrixMultC: (rindexs_req[2] > windexs[1]);
							 //RMatrixMultC: (foo > writePtr);
							 RMatrixMultB: False; 
							 default: ?;
						      endcase);
				   WMatrixMultC: return False;
				   default: return ?;
				endcase;
   
   Bool writePtr_past_readPtr = case (writeAccessPattern)
				   WRowMajorScalar,
				   WRowMajor: return (case (readAccessPattern)
				      			 RRowMajorScalar: (rindexs_req[1] < windexs[1]);
							 //RRowMajorScalar: (readReqPtr < writePtr);
				      			 RMatrixMultC: (rindexs_req[2] < windexs[1]);
							 //RMatrixMultC: (foo < writePtr);
							 RMatrixMultB: False; 
							 default: ?;
						      endcase);
				   WMatrixMultC: return False;
				   default: return ?;
				endcase;
   
   Bool canWriteNext = case (writeAccessPattern)
			  WMatrixMultC: return True;
			  default: return (!readInProgress  || (!readOlder) || (readPtr_past_writePtr));
		       endcase;
   
   
   
   //we need to do this to deal with overflow cruft;
   function Bool readPtr_lt_writePtr_plus_sixtyfour( Bit#(12) rp, Bit#(12) wp );
      Bit#(13) irp = zeroExtend(pack(rp));
      Bit#(13) iwp = zeroExtend(pack(wp));
      return (irp<iwp+64);
   endfunction

   Bool canReadNext =  case (writeAccessPattern)
			  WMatrixMultC: return (!writeInProgress || 
						readPtr_lt_writePtr_plus_sixtyfour({rindexs_req[1], rindexs_req[0]}, {windexs[1],windexs[0]}));
			  default: return (!writeInProgress || (readOlder)  || (writePtr_past_readPtr));
		       endcase;
   
			  
   function Bit#(bank_addr_nbits) bankAddr(MemAddr ra);
      Tuple2#(Bit#(bank_addr_nbits), Bit#(log_n)) b = split(ra);
      return tpl_1(b);
   endfunction

   function Bit#(log_n) bankName(MemAddr ra);
      return truncate(pack(ra));
   endfunction
   
   rule read_rule  (canReadNext && readInProgress);
      case (readAccessPattern)
	 RRowMajorScalar: begin
			     (brams[bankName(readReqAddr)]).readReq(bankAddr(readReqAddr));
			     //$display("reading from bank (col) %d at adder %h readAddr %h", bankName(readAddr), bankAddr(readAddr), readAddr);
			  end
	 default: begin
		     for(int i = 0; i < ival_n; i=i+1)
			begin
			   (brams[i]).readReq(bankAddr(readReqAddr));
			   //$display("reading from bank %d at adder %h readAddr %h", i, bankAddr(readAddr), readAddr);
			end
		  end
      endcase
      readReqPtr<=(inc_readReqPtr == max_readPtr) ? 0 : inc_readReqPtr;
      readInProgress <= (inc_readReqPtr == max_readPtr) ? False : True;
      //$display("mkRegQ_NEW: read readPtr %h, inc_readPtr %h, max_readPtr %h", readPtr, inc_readPtr, max_readPtr);
   endrule

   method Action startRead(x) if (!readInProgress);
      readAccessPattern <= x;
      readParity <= writeParity; // remember the write parity when we start
      readInProgress <= True;
   endmethod   
   
   method Action startWrite(x) if (!writeInProgress);
      writeAccessPattern <= x;
      writeParity <= !writeParity;
      writeInProgress <= True;		     
   endmethod
   
   method ActionValue#(Vector#(n,Complex16)) read();
      Vector#(n,Complex16) rvec = newVector();
      case (readAccessPattern)
	 RRowMajorScalar: begin
			     let rr <- (brams[bankName(readRespAddr)]).readRsp();
			     rvec[0] = rr;
			  end
	 default: begin
		     for(int i = 0; i < ival_n; i=i+1)
			begin
                           let rr <-  (brams[i]).readRsp();
			   rvec[i] = rr;
			end
		  end
      endcase
      readRespPtr<=(inc_readRespPtr == max_readPtr) ? 0 : inc_readRespPtr;
      return rvec;
   endmethod
   
   method Action write(Vector#(n,Complex16) x) if (canWriteNext && writeInProgress);
      case (writeAccessPattern)
	 WRowMajorScalar: (brams[bankName(writeAddr)]).write(bankAddr(writeAddr), x[0]);
	 default: begin
		     for(int i = 0; i < ival_n; i=i+1)
			begin
			   (brams[i]).write(bankAddr(writeAddr), x[i]);
			   $display("writing to bank %d at adder %h writeAddr %h", i, bankAddr(writeAddr), writeAddr);
			end
		  end
	 
      endcase
      writePtr <= (inc_writePtr==max_writePtr) ? 0 : inc_writePtr;
      $display("mkRegQ_STRIPPED: write writePtr %h, inc_writePtr %h, max_writePtr %h", writePtr, inc_writePtr, max_writePtr);
      writeInProgress <= (inc_writePtr==max_writePtr) ? False : True;
   endmethod
   
   method Bool lastRead(); 
      return (inc_readRespPtr == max_readPtr) ? True : False;
   endmethod

   method Bool doneRead();
      return !readInProgress;
   endmethod

   method Bool doneWrite();
      return !writeInProgress;
   endmethod
   
endmodule



module mkFunctionalUnit_BLOAT8(FunctionalUnit);
   NumTypeParam#(8) sizer = ?;
   let x <- mkFunctionalUnit_BLOAT(sizer);
   return x;
endmodule   

module mkFunctionalUnit_BLOAT#(NumTypeParam#(t) sizer) (FunctionalUnit) provisos (Mul#(m,t,TExp#(LineSize)),
							     Log#(t, log_n),
							     Add#(log_n, dont_use_me, LogBlockSize),
							     Add#(1, dont_use_me_either, t));

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

   Reg#(MatrixAddr) k <- mkReg(0);  
   Reg#(ComplexWord) a <- mkReg(0);
   
   RegQ#(t) regQ_a <- mkRegQ_NEW();
   RegQ#(t) regQ_b <- mkRegQ_NEW();
   RegQ#(t) regQ_c <- mkRegQ_NEW();
   
   Reg#(Bool) isNetOpA <- mkRegU();
   Reg#(Bool) isNetOpB <- mkRegU();	 
   Reg#(Bool) isNetOpC <- mkRegU(); 

   Reg#(Bool) doneARead <- mkReg(True);
   Reg#(Bool) doneBRead <- mkReg(True);	 
   Reg#(Bool) doneCRead <- mkReg(True); 

   Bool doneAWrite = regQ_a.doneWrite();
   Bool doneBWrite = regQ_b.doneWrite();
   Bool doneCWrite = regQ_c.doneWrite(); 
   
   Reg#(FunctionalUnitOp) cmd <- mkRegU();
   
   function ActionValue#(Vector#(t,a)) av(a x);
   actionvalue
      return replicate(x);
   endactionvalue
   endfunction
   
   function val_t add( val_t x, val_t y) provisos (Arith#(val_t));
      return x+y;
   endfunction

   function val_t sub( val_t x, val_t y) provisos (Arith#(val_t));
      return x-y;
   endfunction
   
   function val_t mul( val_t x, val_t y) provisos (Arith#(val_t));
      return x*y;
   endfunction
   
   let ncubed = case(cmd)
		   Multiply,
		   MultiplyAddAccumulate,
		   MultiplySubAccumulate: return True;
		   default: return(False);
		endcase;
   
   Stmt compute = 
   seq 
      $display("FuncUnit: start compute");
      while (!regQ_c.doneWrite)
	 action
	    k <= k + fromInteger(valueOf(t));

	    // the row is incremented every (64*64) elements
	    Bit#(6) row_iter = truncate(pack((k%(64*64)/64)));
	    
	    // for the Multiply* instructions, we want to read A once for each row of B
	    let read_a = ((k%64)==0);
	    
            if (!doneARead && regQ_a.lastRead)
	       doneARead <= True;
            if (!doneBRead && regQ_b.lastRead)//only if reading B
	       doneBRead <= True;
            if (!doneCRead && regQ_c.lastRead)
	       doneCRead <= True;
	    
	    let a_val  <- (ncubed) ? (read_a ? regQ_a.read() : av(?) ) : regQ_a.read();
	  
	    a <= read_a ? a_val[0] : a;
	    let a_scalar = read_a ? a_val[0] : a;
	    
	    
	    let b_val  <- regQ_b.read();	    
	    let c_val  <- (cmd == Zero) ? av(0) : regQ_c.read();
	    let m_val  = map(mul(a_scalar), b_val);
	    
	    let cc_val = (cmd==Multiply)&&(row_iter==0) ? replicate(0) : c_val;   
	    
	    let nc_val = case (cmd)
			    Multiply,
			    MultiplyAddAccumulate: return zipWith(add, cc_val, m_val);
			    MultiplySubAccumulate: return zipWith(sub, cc_val, m_val);
			    Zero:	           return cc_val;
			    AddAccumulate: return zipWith(add, cc_val, a_val);
			    SubAccumulate: return zipWith(sub, cc_val, a_val);
			 endcase;
	    
	    $display("compute cycle k=%h, row_iter=%h", k, row_iter);
	    
	    regQ_c.write(nc_val);
	 endaction
      $display("FuncUnit: done compute");
   endseq; // end fsm
   
   FSM compute_fsm <- mkFSM(compute);
   
   Stmt transferFromA = 
   seq
      while(!doneARead)
	 action
	    let data <- regQ_a.read();
	    let fifo = (isNetOpA)? a2networkQ : toMemQ;
	    fifo.enq(data[0]);
	    doneARead <= regQ_a.lastRead(); 
	 endaction
   endseq; // end fsm

   FSM transferFromA_fsm <- mkFSM(transferFromA);
   
   Stmt transferFromB = 
   seq
      while(!doneBRead)
	 action
	    let data <- regQ_b.read();
	    let fifo = (isNetOpB)? b2networkQ : toMemQ;
	    fifo.enq(data[0]);
	    doneBRead <= (regQ_b.lastRead);
	 endaction
   endseq; // end fsm
   
   FSM transferFromB_fsm <- mkFSM(transferFromB);

   Stmt transferFromC = 
   seq
      while(!doneCRead)
	 action
	    let data <- regQ_c.read();
	    let fifo = (isNetOpC)? c2networkQ : toMemQ;
	    fifo.enq(data[0]);
	    doneCRead <= (regQ_c.lastRead);
	 endaction
   endseq; // end fsm
   
   FSM transferFromC_fsm <- mkFSM(transferFromC);
   
   Stmt transferToA = 
   seq 
      while(!regQ_a.doneWrite)
	 action
	    let fifo = (isNetOpA)? network2aQ : fromMemQ;
	    regQ_a.write(replicate(fifo.first()));
	    fifo.deq();
	 endaction
   endseq; //end fsm
   
   FSM transferToA_fsm <- mkFSM(transferToA);

   Stmt transferToB = 
   seq 
      while(!regQ_b.doneWrite)
	 action
	    let fifo = (isNetOpB)? network2bQ : fromMemQ;
	    regQ_b.write(replicate(fifo.first()));
	    fifo.deq();
	 endaction
   endseq; //end fsm
   
   FSM transferToB_fsm <- mkFSM(transferToB);

   Stmt transferToC = 
   seq 
      while(!regQ_c.doneWrite)
	 action
	    let fifo = (isNetOpC)? network2cQ : fromMemQ;
	    regQ_c.write(replicate(fifo.first()));
	    fifo.deq();
	 endaction
   endseq; //end fsm
   
   FSM transferToC_fsm <- mkFSM(transferToC);   

   Bool idle =  doneARead && doneAWrite && doneBRead && doneBWrite && doneCRead && doneCWrite;

   rule startOp (idle);
      let ins = instQ.first();
      instQ.deq();
      k <= 0;
      case (ins) matches
	 tagged ForwardSrc .rsrc:
	    begin
               $display("FuncUnit: FwdSrc %s" , showReg(rsrc));
	       case (rsrc)
		  A: begin doneARead <= False; regQ_a.startRead(RRowMajorScalar);
			isNetOpA <= True; transferFromA_fsm.start(); end
		  B: begin doneBRead <= False; regQ_b.startRead(RRowMajorScalar);
			isNetOpB <= True; transferFromB_fsm.start(); end
		  C: begin doneCRead <= False; regQ_c.startRead(RRowMajorScalar);
			isNetOpC <= True; transferFromC_fsm.start(); end
	       endcase
	    end
	 tagged ForwardDest .rdst:
	    begin
	       $display("FuncUnit: FwdDest %s" , showReg(rdst));
	       case (rdst)
		  A: begin regQ_a.startWrite(WRowMajorScalar);
		        isNetOpA <= True; transferToA_fsm.start(); end
		  B: begin regQ_b.startWrite(WRowMajorScalar);
		        isNetOpB <= True; transferToB_fsm.start(); end 
		  C: begin regQ_c.startWrite(WRowMajorScalar); 
		        isNetOpC <= True; transferToC_fsm.start(); end
	       endcase
	    end
	 tagged Load  .rgdst:
	    begin
	       $display("FuncUnit: Load %s", showReg(rgdst));
	       case (rgdst)
		  A: begin regQ_a.startWrite(WRowMajorScalar);
		        isNetOpA <= False; transferToA_fsm.start(); end
		  B: begin regQ_b.startWrite(WRowMajorScalar);
		        isNetOpB <= False; transferToB_fsm.start(); end
		  C: begin regQ_c.startWrite(WRowMajorScalar);
		        isNetOpC <= False; transferToC_fsm.start(); end
	       endcase
	    end
	 tagged Store .rgsrc:
	    begin
	       $display("FuncUnit: Store %s", showReg(rgsrc));
	       case (rgsrc)
		  A: begin doneARead <= False; regQ_a.startRead(RRowMajorScalar);
		        isNetOpA <= False;  transferFromA_fsm.start(); end
		  B: begin doneBRead <= False; regQ_b.startRead(RRowMajorScalar);
		        isNetOpB <= False; transferFromB_fsm.start(); end 
		  C: begin doneCRead <= False; regQ_c.startRead(RRowMajorScalar);
		        isNetOpC <= False; transferFromC_fsm.start(); end
	       endcase
	    end
	 tagged Op .op:
	    begin
	       $display("FuncUnit: Op %h", op);
	       cmd <= op;
	       case (op)
		  Multiply,
		  MultiplyAddAccumulate,
		  MultiplySubAccumulate: begin
					    doneARead <= False; regQ_a.startRead(RRowMajorScalar); 
					    doneBRead <= False; regQ_b.startRead(RMatrixMultB); 
					    doneCRead <= False; regQ_c.startRead(RMatrixMultC);
					    regQ_c.startWrite(WMatrixMultC);
					 end
		  AddAccumulate,
		  SubAccumulate: begin
				    doneARead <= False; regQ_a.startRead(RRowMajor); 
				    doneBRead <= False; regQ_b.startRead(RRowMajor); 
				    doneCRead <= False; regQ_c.startRead(RRowMajor); 
				    regQ_c.startWrite(WRowMajor);
				 end
		  default: begin  //Zero, no reading from C
			      doneARead <= False; regQ_a.startRead(RRowMajor); 
			      doneBRead <= False; regQ_b.startRead(RRowMajor); 
			      regQ_c.startWrite(WRowMajor);
			   end
	       endcase
	       compute_fsm.start();
	    end
      endcase
   endrule

   interface functionalUnitCommandInput = 
      interface Put#(FunctionalUnitCommand);
      method Action put(x); 
	 instQ.enq(x);
      endmethod
      endinterface;
   interface switchInput     = fifoToPut(fromMemQ);
   interface switchOutput    = interface Get;//fifoToGet(toMemQ);
				  method get();
				     actionvalue
					$display("FuncUnit: Sending %h", toMemQ.first());
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


module mkRegQ_STRIPPEDA(RegQ#(n)) provisos (Mul#(m,n,TExp#(LineSize)),
					    Log#(n, log_n),
					    Add#(log_n, bank_addr_nbits, LogBlockSize));  
   
   Vector#(n,  MEMORY_IFC#(Bit#(bank_addr_nbits), Complex16)) brams <- replicateM(mkBRAM);
   
   Reg#(MatrixAddr)    writePtr <- mkConfigReg(0); // Goes from 0 to n^3-1 {i,j,k}
   Reg#(MatrixAddr)  readReqPtr <- mkConfigReg(0);
   Reg#(MatrixAddr) readRespPtr <- mkConfigReg(0);
   
   Reg#(Bool)      readParity <- mkReg(False);
   Reg#(Bool)     writeParity <- mkReg(False);

   Reg#(Bool)     readInProgress <- mkConfigReg(False);
   Reg#(Bool)    writeInProgress <- mkConfigReg(False);
   
   Bool readOlder = readParity != writeParity;
   
   MatrixAddr inc_writePtr = writePtr + 1;   
   MatrixAddr inc_readReqPtr = readReqPtr + 1;
   MatrixAddr inc_readRespPtr = readRespPtr + 1;

   MemAddr readReqAddr = truncate(readReqPtr);
   MemAddr readRespAddr = truncate(readRespPtr);
   MemAddr writeAddr = truncate(writePtr);
   
   // we can compare adders here since they are merely trincated Ptrs
   Bool readPtr_past_writePtr = readReqAddr > writeAddr;
   Bool writePtr_past_readPtr = readReqAddr < writeAddr;
   
   Bool canWriteNext =  (!readInProgress  || (!readOlder) || (readPtr_past_writePtr));
   Bool canReadNext =   (!writeInProgress || (readOlder)  || (writePtr_past_readPtr));
   
   MatrixAddr max_readPtr =  (mat_nn*mat_nn);
   MatrixAddr max_writePtr = (mat_nn*mat_nn);
			  
   function Bit#(bank_addr_nbits) bankAddr(MemAddr ra);
      Tuple2#(Bit#(bank_addr_nbits), Bit#(log_n)) b = split(ra);
      return tpl_1(b);
   endfunction

   function Bit#(log_n) bankName(MemAddr ra);
      return truncate(pack(ra));
   endfunction
   
   rule read_rule  (canReadNext && readInProgress);
      (brams[bankName(readReqAddr)]).readReq(bankAddr(readReqAddr));
      readReqPtr <= (inc_readReqPtr==max_readPtr) ? 0 : inc_readReqPtr;
      readInProgress <= (inc_readReqPtr == max_readPtr) ? False : True;
   endrule

   method Action startRead(x) if (!readInProgress);
      readParity <= writeParity; // remember the write parity when we start
      readInProgress <= True;
      readReqPtr <= 0;
      readRespPtr <= 0;
   endmethod   
   
   method Action startWrite(x) if (!writeInProgress);
      writeParity <= !writeParity;
      writeInProgress <= True;
      writePtr <= 0;
   endmethod
   
   method ActionValue#(Vector#(n,Complex16)) read();
      let rr <- (brams[bankName(readRespAddr)]).readRsp();
      readRespPtr <= (inc_readRespPtr==max_readPtr) ? 0 : inc_readRespPtr;
      return replicate(rr);
   endmethod
   
   method Action write(Vector#(n,Complex16) x) if (canWriteNext && writeInProgress);
      (brams[bankName(writeAddr)]).write(bankAddr(writeAddr), x[0]);
      writePtr <= (inc_writePtr==max_writePtr) ? 0 : inc_writePtr;
      writeInProgress <= (inc_writePtr==max_writePtr) ? False : True;
   endmethod
   
   method Bool lastRead(); 
      return (inc_readRespPtr == max_readPtr);// || !readInProgress;
   endmethod

   method Bool doneRead();
      return !readInProgress;
   endmethod

   method Bool doneWrite();
      return !writeInProgress;
   endmethod
   
endmodule


module mkRegQ_STRIPPEDB(RegQ#(n)) provisos (Mul#(m,n,TExp#(LineSize)),
					    Log#(n, log_n),
					    Add#(log_n, bank_addr_nbits, LogBlockSize));  
   
   Vector#(n, MEMORY_IFC#(Bit#(bank_addr_nbits), Complex16)) brams <- replicateM(mkBRAM);
   
   Reg#(ReadAccessPattern) readAccessPattern <- mkRegU; 
   Reg#(WriteAccessPattern) writeAccessPattern <- mkRegU; 
   
   Reg#(MatrixAddr)    writePtr <- mkConfigReg(0); // Goes from 0 to n^3-1 {i,j,k}
   Reg#(MatrixAddr)  readReqPtr <- mkConfigReg(0);
   Reg#(MatrixAddr) readRespPtr <- mkConfigReg(0);
   
   Reg#(Bool)      readParity <- mkReg(False);
   Reg#(Bool)     writeParity <- mkReg(False);

   Reg#(Bool)     readInProgress <- mkConfigReg(False);
   Reg#(Bool)    writeInProgress <- mkConfigReg(False);
   
   Bool readOlder = readParity != writeParity;
   
   let ival_n = fromInteger(valueOf(n));
   
   MatrixAddr inc_writePtr = writePtr + 1;
   
   MatrixAddr inc_readReqPtr = readReqPtr + (case (readAccessPattern)
						RRowMajorScalar: 1;
						RMatrixMultB: ival_n; 
						default: ?;
					     endcase);
   
   MatrixAddr inc_readRespPtr = readRespPtr + (case (readAccessPattern)
						RRowMajorScalar: 1;
						RMatrixMultB: ival_n; 
						default: ?;
					     endcase);

   MatrixAddr max_readPtr = case (readAccessPattern)
			       RRowMajorScalar: return (mat_nn*mat_nn);
			       RMatrixMultB:   return 0;
			       default: ?;
			    endcase;

   MatrixAddr max_writePtr = (mat_nn*mat_nn);
   
   // for i; for j; for k
   // the matrix address is [i|j|k] 
   let rindexs_req = splitMatrixAddr(readReqPtr);
   let rindexs_resp = splitMatrixAddr(readRespPtr);
   let windexs = splitMatrixAddr(writePtr);
   
   MemAddr readReqAddr = case (readAccessPattern)
			    RRowMajorScalar:   return truncate(readReqPtr);
			    RMatrixMultB: return {rindexs_req[1], rindexs_req[0]}; 
			    default:   return ?;
			 endcase;
   
   MemAddr readRespAddr = case (readAccessPattern)
			     RRowMajorScalar:   return truncate(readRespPtr);
			     RMatrixMultB: return {rindexs_resp[1], rindexs_resp[0]}; 
			     default:   return ?;
			  endcase;
   
   MemAddr writeAddr = truncate(writePtr);   
   
   // something is fishy
   Bool readPtr_past_writePtr =  (case (readAccessPattern)
				     RRowMajorScalar: (rindexs_req[1] > windexs[1]);
				     RMatrixMultB: False; 
				     default: ?;
				  endcase);
   
   // something is fishy
   Bool writePtr_past_readPtr = (case (readAccessPattern)
				    RRowMajorScalar: (rindexs_req[1] < windexs[1]);
				    RMatrixMultB: False; 
				    default: ?;
				 endcase);
   
   Bool canWriteNext = !readInProgress  || (!readOlder) || (readPtr_past_writePtr);
      
   Bool canReadNext =  !writeInProgress || (readOlder)  || (writePtr_past_readPtr);
   			  
   function Bit#(bank_addr_nbits) bankAddr(MemAddr ra);
      Tuple2#(Bit#(bank_addr_nbits), Bit#(log_n)) b = split(ra);
      return tpl_1(b);
   endfunction

   function Bit#(log_n) bankName(MemAddr ra);
      return truncate(pack(ra));
   endfunction
   
   rule read_rule  (canReadNext && readInProgress);
      for(int i = 0; i < ival_n; i=i+1)
	 begin
	    (brams[i]).readReq(bankAddr(readReqAddr));
	 end
      readReqPtr<=(inc_readReqPtr == max_readPtr) ? 0 : inc_readReqPtr;
      readInProgress <= (inc_readReqPtr == max_readPtr) ? False : True;
   endrule

   method Action startRead(x) if (!readInProgress);
      readAccessPattern <= x;
      readParity <= writeParity; // remember the write parity when we start
      readInProgress <= True;
   endmethod   
   
   method Action startWrite(x) if (!writeInProgress);
      writeAccessPattern <= x;
      writeParity <= !writeParity;
      writeInProgress <= True;		     
   endmethod
   
   method ActionValue#(Vector#(n,Complex16)) read();

//       Vector#(n,Complex16) rvec = newVector();
//       case (readAccessPattern)
// 	 RRowMajorScalar: begin
// 			     let rr <- (brams[bankName(readRespAddr)]).readRsp();
// 			     rvec[0] = rr;
// 			  end
// 	 default: begin
// 		     for(int i = 0; i < ival_n; i=i+1)
// 			begin
//                            let rr <-  (brams[i]).readRsp();
// 			   rvec[i] = rr;
// 			end
// 		  end
//       endcase
//       readRespPtr<=(inc_readRespPtr == max_readPtr) ? 0 : inc_readRespPtr;
//       return rvec;

      Vector#(n,Complex16) rvec = newVector();
      for(int i = 0; i < ival_n; i=i+1)
	 begin
	    let rr <-  (brams[i]).readRsp();
	    rvec[i] = rr;
	 end
      readRespPtr<=(inc_readRespPtr == max_readPtr) ? 0 : inc_readRespPtr;
      return rvec;
   endmethod
   
   method Action write(Vector#(n,Complex16) x) if (canWriteNext && writeInProgress);
      (brams[bankName(writeAddr)]).write(bankAddr(writeAddr), x[0]);
      writePtr <= (inc_writePtr==max_writePtr) ? 0 : inc_writePtr;
      writeInProgress <= (inc_writePtr==max_writePtr) ? False : True;
   endmethod
   
   method Bool lastRead();
      return (inc_readRespPtr == max_readPtr);// || !readInProgress; 
   endmethod

   method Bool doneRead();
      return !readInProgress;
   endmethod

   method Bool doneWrite();
      return !writeInProgress;
   endmethod
   
endmodule


module mkRegQ_STRIPPEDC(RegQ#(n)) provisos (Mul#(m,n,TExp#(LineSize)),
					    Log#(n, log_n),
					    Add#(log_n, bank_addr_nbits, LogBlockSize));  
   
   Vector#(n, MEMORY_IFC#(Bit#(bank_addr_nbits), Complex16)) brams <- replicateM(mkBRAM);
   
   Reg#(ReadAccessPattern) readAccessPattern <- mkRegU; 
   Reg#(WriteAccessPattern) writeAccessPattern <- mkRegU; 
   
   Reg#(MatrixAddr)    writePtr <- mkConfigReg(0); // Goes from 0 to n^3-1 {i,j,k}
   Reg#(MatrixAddr)  readReqPtr <- mkConfigReg(0);
   Reg#(MatrixAddr) readRespPtr <- mkConfigReg(0);
   
   Reg#(Bool)      readParity <- mkReg(False);
   Reg#(Bool)     writeParity <- mkReg(False);

   Reg#(Bool)     readInProgress <- mkConfigReg(False);
   Reg#(Bool)    writeInProgress <- mkConfigReg(False);
   
   Bool readOlder = readParity != writeParity;
   
   let ival_n = fromInteger(valueOf(n));
   
   MatrixAddr inc_writePtr = writePtr + (case (writeAccessPattern)
					    WRowMajorScalar: 1;
					    WMatrixMultC: ival_n;
					    default: 1;
					 endcase);
   
   MatrixAddr inc_readReqPtr = readReqPtr + (case (readAccessPattern)
						RRowMajorScalar: 1;
						RMatrixMultC: ival_n; 
						default: ?;
					     endcase);
   
   MatrixAddr inc_readRespPtr = readRespPtr + (case (readAccessPattern)
					      RRowMajorScalar: 1;
					      RMatrixMultC: ival_n; 
					      default: ?;
					   endcase);

   MatrixAddr max_readPtr = case (readAccessPattern)
			       RRowMajorScalar: return (mat_nn*mat_nn);
			       RMatrixMultC:   return 0; // wraps around formerly (mat_nn*mat_nn*mat_nn)
			       default: ?;
			    endcase;

   MatrixAddr max_writePtr = case (writeAccessPattern)
				WMatrixMultC: return 0;//(mat_nn*mat_nn*(mat_nn-1));
				WRowMajorScalar: return (mat_nn*mat_nn);
			    endcase;
   
   // for i; for j; for k
   // the matrix address is [i|j|k] 
   let rindexs_req = splitMatrixAddr(readReqPtr);
   let rindexs_resp = splitMatrixAddr(readRespPtr);
   let windexs = splitMatrixAddr(writePtr);
   
   MemAddr readReqAddr = case (readAccessPattern)
			    RRowMajorScalar:   return truncate(readReqPtr);
			    RMatrixMultC: return {rindexs_req[2], rindexs_req[0]}; 
			    default:   return ?;
			 endcase;
   
   MemAddr readRespAddr = case (readAccessPattern)
			     RRowMajorScalar:   return truncate(readRespPtr);
			     RMatrixMultC: return {rindexs_resp[2], rindexs_resp[0]}; 
			     default:   return ?;
			  endcase;
   
   MemAddr writeAddr = case (writeAccessPattern)
			  WRowMajorScalar: return truncate(writePtr);
			  WMatrixMultC: return {windexs[2], windexs[0]};
			  default: return ?;
		       endcase;
   
   
   MatrixAddr foo = bits2MatrixAddr({rindexs_req[2],0});
   // something fishy here
   Bool readPtr_past_writePtr = case (writeAccessPattern)
				   WRowMajorScalar: return (case (readAccessPattern)
							 RRowMajorScalar: (rindexs_req[1] > windexs[1]);
							 RMatrixMultC: (rindexs_req[2] > windexs[1]);
							 default: ?;
						      endcase);
				   WMatrixMultC: return False;
				   default: return ?;
				endcase;
   
   // something fishy here
   Bool writePtr_past_readPtr = case (writeAccessPattern)
				   WRowMajorScalar: return (case (readAccessPattern)
				      			       RRowMajorScalar: (rindexs_req[1] < windexs[1]);
				      			       RMatrixMultC: (rindexs_req[2] < windexs[1]);
							       default: ?;
							    endcase);
				   WMatrixMultC: return False;
				   default: return ?;
				endcase;
   
   Bool canWriteNext = case (writeAccessPattern)
			  WMatrixMultC: return True;
			  default: return (!readInProgress  || (!readOlder) || (readPtr_past_writePtr));
		       endcase;   
   
   //we need to do this to deal with overflow cruft;
   function Bool readPtr_lt_writePtr_plus_sixtyfour( Bit#(12) rp, Bit#(12) wp );
      Bit#(13) irp = zeroExtend(pack(rp));
      Bit#(13) iwp = zeroExtend(pack(wp));
      return (irp<iwp+64);
   endfunction

   Bool canReadNext =  case (writeAccessPattern)
			  WMatrixMultC: return (!writeInProgress || 
						readPtr_lt_writePtr_plus_sixtyfour({rindexs_req[1], rindexs_req[0]}, {windexs[1],windexs[0]}));
			  default: return (!writeInProgress || (readOlder)  || (writePtr_past_readPtr));
		       endcase;
   
			  
   function Bit#(bank_addr_nbits) bankAddr(MemAddr ra);
      Tuple2#(Bit#(bank_addr_nbits), Bit#(log_n)) b = split(ra);
      return tpl_1(b);
   endfunction

   function Bit#(log_n) bankName(MemAddr ra);
      return truncate(pack(ra));
   endfunction
   
   rule read_rule  (canReadNext && readInProgress);
      for(int i = 0; i < ival_n; i=i+1)
	 begin
	    (brams[i]).readReq(bankAddr(readReqAddr));
	 end
      
      readReqPtr<=(inc_readReqPtr == max_readPtr) ? 0 : inc_readReqPtr;
      readInProgress <= (inc_readReqPtr == max_readPtr) ? False : True;
   endrule

   method Action startRead(x) if (!readInProgress);
      readAccessPattern <= x;
      readParity <= writeParity; // remember the write parity when we start
      readInProgress <= True;
   endmethod   
   
   method Action startWrite(x) if (!writeInProgress);
      writeAccessPattern <= x;
      writeParity <= !writeParity;
      writeInProgress <= True;		     
   endmethod
   
   method ActionValue#(Vector#(n,Complex16)) read();
      Vector#(n,Complex16) rvec = newVector();
      for(int i = 0; i < ival_n; i=i+1)
	 begin
	    let rr <-  (brams[i]).readRsp();
	    rvec[i] = rr;
	 end
      readRespPtr<=(inc_readRespPtr == max_readPtr) ? 0 : inc_readRespPtr;
      return rvec;
      
   endmethod
   
   method Action write(Vector#(n,Complex16) x) if (canWriteNext && writeInProgress);
      case (writeAccessPattern)
	 WRowMajorScalar: (brams[bankName(writeAddr)]).write(bankAddr(writeAddr), x[0]);
	 default: begin
		     for(int i = 0; i < ival_n; i=i+1)
			begin
			   (brams[i]).write(bankAddr(writeAddr), x[i]);
			end
		  end
	 
      endcase
      writePtr <= (inc_writePtr==max_writePtr) ? 0 : inc_writePtr;
      writeInProgress <= (inc_writePtr==max_writePtr) ? False : True;
   endmethod
   
   method Bool lastRead(); 
      return (inc_readRespPtr == max_readPtr);// || !readInProgress;
   endmethod

   method Bool doneRead();
      return !readInProgress;
   endmethod

   method Bool doneWrite();
      return !writeInProgress;
   endmethod
   
endmodule






module mkFunctionalUnit(FunctionalUnit);
   NumTypeParam#(32) sizer = ?;
   let x <- mkFunctionalUnit_STRIPPED(sizer);
   return x;
endmodule   


module mkFunctionalUnit_STRIPPED#(NumTypeParam#(t) width) (FunctionalUnit) provisos (Mul#(m,t,TExp#(LineSize)),
								Log#(t, log_n),
								Add#(log_n, bank_addr_nbits, LogBlockSize),
								Add#(1, dont_use_me_either, t));

   FIFOF#(FunctionalUnitCommand) instQ <- mkFIFOF();
   FIFO#(ComplexWord) toMemQ          <- mkFIFO();   
   FIFO#(ComplexWord) fromMemQ        <- mkFIFO();

   //state used by compute FSM
   Reg#(Bit#(32)) k <- mkReg(0);  
   Reg#(ComplexWord) a <- mkReg(0);
  
   // can get rid of this when done debugging
   Reg#(int) cycle_cnt <- mkReg(0);
   
   RegQ#(t) regQ_a <- mkRegQ_STRIPPEDA();
   RegQ#(t) regQ_b <- mkRegQ_STRIPPEDB();
   RegQ#(t) regQ_c <- mkRegQ_STRIPPEDC();
   
   Vector#(t, MulFirst) muls <- replicateM(mkMulFirst);

   // this may be too large
   Reg#(Bit#(log_n)) regQ_b_bank_sel <- mkReg(0);
   Reg#(Bit#(log_n)) regQ_c_bank_sel <- mkReg(0);
   
   Reg#(Bool) doneARead <- mkReg(True);
   Reg#(Bool) doneBRead <- mkReg(True);	 
   Reg#(Bool) doneCRead <- mkReg(True); 
   Reg#(Bool) toMemQ_InUse   <- mkReg(False);
   Reg#(Bool) fromMemQ_InUse <- mkReg(False);

   Bool doneAWrite = regQ_a.doneWrite();
   Bool doneBWrite = regQ_b.doneWrite();
   Bool doneCWrite = regQ_c.doneWrite(); 
   
   Reg#(FunctionalUnitOp) cmd <- mkRegU();
   
   Reg#(Vector#(t,Vector#(4,Bit#(16)))) mul_reg<- mkRegU();
   Reg#(Bool) wrote_mul_reg <- mkReg(False);
   Reg#(Bool) last_compute_cycle <- mkReg(False);
   
   rule increment_cycle_cnt (True);
      cycle_cnt <= cycle_cnt+1;
   endrule

   function ActionValue#(Vector#(t,a)) av(a x);
   actionvalue
      return replicate(x);
   endactionvalue
   endfunction
   
   function val_t add( val_t x, val_t y) provisos (Arith#(val_t));
      return x+y;
   endfunction

   function val_t mul( val_t x, val_t y) provisos (Arith#(val_t));
      return x*y;
   endfunction

   
   // TODO: compute fsm is far too complicated.  it needs to be simplified (possibly broken apart) --mdk
   Stmt compute = 
   seq 
      $display("FuncUnit: start compute cycle_count=%d", cycle_cnt);
      while (!regQ_c.doneWrite)
	 action
	    k <= k + fromInteger(valueOf(t));
	    if (!doneARead) doneARead <= regQ_a.lastRead;
	    if (!doneBRead) doneBRead <= regQ_b.lastRead;
	    if (!doneCRead) doneCRead <= regQ_c.lastRead;
	    
	    case (cmd)
	       Multiply,
	       MultiplyAddAccumulate: 
	       begin		  
		 // read from regQ_a and regQ_b only the correct number of elements		  

		 Bit#(6) row_iter = truncate(pack((k%(64*64)/64)));
		 let read_a = ((k%64)==0);
		  
		  if(!last_compute_cycle)
		     begin
			let a_val  <- read_a ? regQ_a.read() : av(?);
			a <= read_a ? a_val[0] : a;
			let a_scalar = read_a ? a_val[0] : a;
			let b_val  <- regQ_b.read();
			function f(m, b) = m.doMult(a_scalar,b);
			mul_reg <= zipWith(f, muls, b_val);
			wrote_mul_reg <= True;
			$display("FuncUnit: compute fsm !last_compute_cycle, cycle_cnt=%d", cycle_cnt);
		     end
		  
		  if(wrote_mul_reg)
		     begin
			let c_val  <- regQ_c.read();
			let m_val =  map(mul_second, mul_reg);
			let cc_val = (cmd==Multiply)&&(row_iter==0) ? replicate(0) : c_val;   
			regQ_c.write(zipWith(add, cc_val, m_val));
			$display("FuncUnit: compute writing regQ_c, cycle_cnt=%d", cycle_cnt);
		     end
		  last_compute_cycle <= k + fromInteger(valueOf(t)) == (64*64*64);
	       end
	       Zero: begin
			regQ_c.write(replicate(0));
		     end
	    endcase
	 endaction
      $display("FuncUnit: done compute cycle_count=%d", cycle_cnt);
   endseq; // end fsm   
   FSM compute_fsm <- mkFSM(compute);
   
   Stmt transferFromA = 
   seq
      $display("FuncUnit: start transferFromA cycle_count=%d", cycle_cnt);
      while(!doneARead)
	 action
	    let data <- regQ_a.read();
	    toMemQ.enq(data[0]);
	    doneARead <= regQ_a.lastRead(); 
	 endaction
       toMemQ_InUse <= False;
      $display("FuncUnit: done transferFromA cycle_count=%d", cycle_cnt);
   endseq; // end fsm
   FSM transferFromA_fsm <- mkFSM(transferFromA);
   
   Stmt transferFromB = 
   seq
      $display("FuncUnit: start transferFromB cycle_count=%d", cycle_cnt);
      while(!doneBRead)
	 action
	    regQ_b_bank_sel <= regQ_b_bank_sel+1;
	    let data <- regQ_b.read();
	    toMemQ.enq(data[regQ_b_bank_sel]);
	    doneBRead <= (regQ_b.lastRead);
	 endaction
      toMemQ_InUse <= False;
      $display("FuncUnit: done transferFromB cycle_count=%d", cycle_cnt);
   endseq; // end fsm
   FSM transferFromB_fsm <- mkFSM(transferFromB);

   Stmt transferFromC = 
   seq
      $display("FuncUnit: start transferFromC cycle_count=%d", cycle_cnt);
      while(!doneCRead)
	 action
	    regQ_c_bank_sel <= regQ_c_bank_sel+1;
	    let data <- regQ_c.read();
	    toMemQ.enq(data[regQ_c_bank_sel]);
	    doneCRead <= (regQ_c.lastRead);
	 endaction
      toMemQ_InUse <= False;
      $display("FuncUnit: done transferFromC cycle_count=%d", cycle_cnt);
   endseq; // end fsm
   FSM transferFromC_fsm <- mkFSM(transferFromC);
   
   Stmt transferToA = 
   seq 
      $display("FuncUnit: start transferToA cycle_count=%d", cycle_cnt);
      while(!regQ_a.doneWrite)
	 action
	    regQ_a.write(replicate(fromMemQ.first()));
	    fromMemQ.deq();
	 endaction
      fromMemQ_InUse <= False;
      $display("FuncUnit: done transferToA cycle_count=%d", cycle_cnt);
   endseq; //end fsm
   FSM transferToA_fsm <- mkFSM(transferToA);

   Stmt transferToB = 
   seq 
      $display("FuncUnit: start transferToB cycle_count=%d", cycle_cnt);
      while(!regQ_b.doneWrite)
	 action
	    regQ_b.write(replicate(fromMemQ.first()));
	    fromMemQ.deq();
	 endaction
      fromMemQ_InUse <= False;
      $display("FuncUnit: done transferToB cycle_count=%d", cycle_cnt);
   endseq; //end fsm
   FSM transferToB_fsm <- mkFSM(transferToB);
   
   Stmt transferToC = 
   seq 
      $display("FuncUnit: start transferToCcycle_count=%d", cycle_cnt);
      while(!regQ_c.doneWrite)
	 action
	    regQ_c.write(replicate(fromMemQ.first()));
	    fromMemQ.deq();
	 endaction
      fromMemQ_InUse <= False;
      $display("FuncUnit: done transferToC cycle_count=%d", cycle_cnt);
   endseq; //end fsm   
   FSM transferToC_fsm <- mkFSM(transferToC);

   let instQ_first_matches_Load = case(instQ.first()) matches 
				     tagged Load .rgdst: return tagged Valid rgdst;
				     default: return tagged Invalid;
				  endcase;
   
   rule startOpLoad (isValid(instQ_first_matches_Load) && !fromMemQ_InUse);
      
      let rgdst = fromMaybe(?, instQ_first_matches_Load);
      case (rgdst)
	 A: begin regQ_a.startWrite(WRowMajorScalar);
	       transferToA_fsm.start(); end
	 B: begin regQ_b.startWrite(WRowMajorScalar);
	       transferToB_fsm.start(); end
	 C: begin regQ_c.startWrite(WRowMajorScalar);
	       transferToC_fsm.start(); end
      endcase
      fromMemQ_InUse <= True;
      instQ.deq();
      $display("FuncUnit: Load %s cycle=%d", showReg(rgdst), cycle_cnt);
   endrule   

   let instQ_first_matches_Store = case(instQ.first()) matches 
				     tagged Store .rgsrc: return tagged Valid rgsrc;
				     default: return tagged Invalid;
				  endcase;
   
   rule startOpStore (isValid(instQ_first_matches_Store) && !toMemQ_InUse && compute_fsm.done);
      
      let rgsrc = fromMaybe(?, instQ_first_matches_Store);
      case (rgsrc)
	 A: begin doneARead <= False; regQ_a.startRead(RRowMajorScalar);
	       transferFromA_fsm.start();  end
	 B: begin doneBRead <= False; regQ_b.startRead(RRowMajorScalar);
	       transferFromB_fsm.start();  regQ_b_bank_sel <= 0; end 
	 C: begin doneCRead <= False; regQ_c.startRead(RRowMajorScalar);
	       transferFromC_fsm.start();  regQ_c_bank_sel <= 0; end
      endcase
      toMemQ_InUse <= True; instQ.deq();
      $display("FuncUnit: Store %s, cycle=%d", showReg(rgsrc),cycle_cnt);
      
   endrule

   let instQ_first_matches_Op = case(instQ.first()) matches 
				   tagged Op .op: return tagged Valid op;
				   default: return tagged Invalid;
				endcase;
   

   rule startOpOp (doneCRead && doneCWrite && doneARead && doneBRead && compute_fsm.done && isValid(instQ_first_matches_Op));
      let op = fromMaybe(?, instQ_first_matches_Op);
      case (op)
	 Multiply,
	 MultiplyAddAccumulate:    begin
				      doneARead <= False; regQ_a.startRead(RRowMajorScalar); 
				      doneBRead <= False; regQ_b.startRead(RMatrixMultB); 
				      doneCRead <= False; regQ_c.startRead(RMatrixMultC);
				      regQ_c.startWrite(WMatrixMultC);
				   end
	 Zero: //if(doneCWrite) 
		 begin
		   regQ_c.startWrite(WRowMajorScalar);
		 end
      endcase
            
      begin
	 last_compute_cycle <= False; wrote_mul_reg <= False;
	 compute_fsm.start(); instQ.deq(); k <= 0; cmd <= op;
	 $display("FuncUnit: Op %h, cycle=%d", op, cycle_cnt);
      end
      
   endrule

   interface functionalUnitCommandInput = interface Put#(FunctionalUnitCommand);
					     method Action put(x); 
						instQ.enq(x);
					     endmethod
					  endinterface;

   interface switchInput     = fifoToPut(fromMemQ);
   interface switchOutput    = fifoToGet(toMemQ);
   interface FUNetworkLink link = ?;
    	 
endmodule

