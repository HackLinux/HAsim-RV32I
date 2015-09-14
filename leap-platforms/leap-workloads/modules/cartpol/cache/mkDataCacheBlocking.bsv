/*
Copyright (c) 2009 MIT

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

*/


`include "awb/provides/librl_bsv.bsh"
`include "awb/provides/soft_connections.bsh"
`include "awb/provides/soft_services.bsh"
`include "awb/provides/soft_services_lib.bsh"
`include "awb/provides/soft_services_deps.bsh"
`include "awb/provides/cartpol_common.bsh"
`include "awb/provides/cartpol_cordic.bsh"


import RegFile::*;
import GetPut::*;
import ClientServer::*;
import FIFO::*;
import FIFOF::*;
import RWire::*;
import Vector::*;

// May want to introduce BFIFO at some point.


//----------------------------------------------------------------------
// Cache Types
//----------------------------------------------------------------------

typedef enum 
{ 
  Init,
  Access, 
  RefillResp // We'll get an entire line here.
} 
CacheStage 
deriving (Eq,Bits);

//----------------------------------------------------------------------
// Helper functions
//----------------------------------------------------------------------

function Bit#(AddrSz) getAddr( DataReq req );
  Bit#(AddrSz) addr = ?;
  case ( req ) matches
    tagged LoadReq  .ld : addr = ld.addr;
  endcase
  return addr;
endfunction

function CacheLineIndex getCacheLineIndex( DataReq req );
  Bit#(AddrSz) addr = getAddr(req);
  Bit#(CacheLineIndexSz) index = truncate( addr >> fromInteger(valueof(CacheLineBlockSz)));
  return index;
endfunction

function CacheLineTag getCacheLineTag( DataReq req );
  Bit#(AddrSz)         addr = getAddr(req);
  Bit#(CacheLineTagSz) tag  = truncate(addr >> fromInteger(valueOf(CacheLineIndexSz)+valueof(CacheLineBlockSz)));
  return tag;
endfunction

function Bit#(AddrSz) getCacheLineAddr( DataReq req );
  Bit#(AddrSz) addr = getAddr(req);
  return ((addr >> fromInteger(valueof(CacheLineBlockSz))) << fromInteger(valueof(CacheLineBlockSz)));
endfunction

function  CacheLineBlockIndex getCacheLineBlockAddr(  DataReq req );
  Bit#(AddrSz) addr = getAddr(req);
  CacheLineBlock block = truncate(addr);
  return truncateLSB(block);
endfunction

//----------------------------------------------------------------------
// Main module
//----------------------------------------------------------------------

(* synthesize *)
module mkDataCacheBlocking( ICache#(DataReq,DataResp) );

  //-----------------------------------------------------------
  // State

  Reg#(CacheStage) stage <- mkReg(Init);

  // May want to use non-regfile so that we can have zeros
  RegFile#(CacheLineIndex,Maybe#(CacheLineTag)) cacheTagRam   <- mkRegFileFull();

  // Long, skinny reg file, with width equal to busword.
  RegFile#(CacheLineIndex,CacheLine)            cacheDataRam  <- mkRegFileFull();

  FIFO#(DataReq)   reqQ  <- mkFIFO();
  FIFOF#(DataResp) respQ <- mkFIFOF();

  FIFO#(MainMemReq)  mainMemReqQ  <- mkFIFO();
  FIFO#(MainMemResp) mainMemRespQ <- mkFIFO();

  Reg#(CacheLineIndex) initCounter <- mkReg(1);

  // Statistics state
  Reg#(Bool)     statsEn        <- mkReg(False);
  Reg#(Int#(25)) num_accesses   <- mkReg(25'h0);
  Reg#(Int#(25)) num_misses     <- mkReg(25'h0);

  //-----------------------------------------------------------
  // Name some wires

  let req              = reqQ.first();
  let reqIndex         = getCacheLineIndex(req);
  let reqTag           = getCacheLineTag(req);
  let reqCacheLineAddr = getCacheLineAddr(req);

  //-----------------------------------------------------------
  // Initialize

  rule init ( stage == Init );    
     initCounter <= initCounter + 1;
     cacheTagRam.upd(initCounter,Invalid);
     if ( initCounter == 0 )
	stage <= Access;
  endrule
   
  //-----------------------------------------------------------
  // Access cache rule

  rule access ( (stage == Access) && respQ.notFull() );
      
     // Statistics
     if ( statsEn )
	num_accesses <= num_accesses + 1;
     // Get the corresponding tag from the rams
     Maybe#(CacheLineTag) cacheLineTag = cacheTagRam.sub(reqIndex);
     
     // Handle cache hits ...
     if ( isValid(cacheLineTag) && ( unJust(cacheLineTag) == reqTag ) )
	begin
           reqQ.deq();
           case ( req ) matches
              tagged LoadReq .ld :
		 begin
		    Vector#(TDiv#(CacheLineSz,CacheWordSize),CacheWord) line = unpack(cacheDataRam.sub(reqIndex));
		    CacheLineBlockIndex foo = getCacheLineBlockAddr(req);
		    respQ.enq( LoadResp {tag: ld.tag, data: line[foo]});
                    $display("Cache Hit: %d addr %h data %h", reqCacheLineAddr,line[foo]);
		 end       
	   endcase
	end
     
     // Handle cache misses ...
     else 
	begin
           if ( statsEn )
              num_misses <= num_misses + 1;
           // since cache is read only... 
           $display("Cache Miss: addr %h", reqCacheLineAddr);
           mainMemReqQ.enq( LoadReq { tag: 0, addr: reqCacheLineAddr } );
           stage <= RefillResp;    
	end
  endrule


  //-----------------------------------------------------------
  // Refill response rule
  
  
  Reg#(Bit#(TLog#(BeatsPerBurst))) refillCounter <- mkReg(0);
  // this we can probably subsume, at some point.
  Vector#(WordsPerBurst,Reg#(BusWord)) linestore <- replicateM(mkRegU);

  rule refillResp ( stage == RefillResp );
    // May take many cycles to obtain values...
    // Write the new data into the cache and update the tag
    mainMemRespQ.deq();
    $display("Cache got main memresp %d of %d",refillCounter, valueof(BeatsPerBurst)-1);
    case ( mainMemRespQ.first() ) matches

      tagged LoadResp .ld :
       begin
         if(refillCounter == fromInteger(valueof(BeatsPerBurst)-1)) 
           begin
             Vector#(1,BusWord) presentData = replicate(ld.data);
             cacheTagRam.upd(reqIndex,Valid(reqTag));
	     cacheDataRam.upd(reqIndex,unpack(pack(append(readVReg(take(linestore)),presentData))));	
             stage <= Access;
             refillCounter <= 0;
           end
         else
           begin // load a single word
             refillCounter <= refillCounter + 1;
             linestore[refillCounter] <= ld.data;  
           end
       end       
     
    endcase


  endrule

  //-----------------------------------------------------------
  // Methods

   method Action reset();
      stage <= Init;
   endmethod 

  interface Client mmem_client;
    interface Get request  = fifoToGet(mainMemReqQ);
    interface Put response = fifoToPut(mainMemRespQ);
  endinterface

  interface Server proc_server;
    interface Put request  = fifoToPut(reqQ);
    interface Get response = fifoToGet(fifofToFifo(respQ));
  endinterface

endmodule


