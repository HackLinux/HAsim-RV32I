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

typedef struct {
  Bit#(TagSz) tag;
  Bit#(TLog#(TDiv#(SizeOf#(BusWord),CacheWordSize))) indexHi;
  Bit#(TLog#(TDiv#(SizeOf#(BusWord),CacheWordSize))) indexLo;
} LoadCtrl deriving (Bits,Eq);

typedef struct {
  DataReq reqHi;
  DataReq reqLo;
} ExpandedReq deriving (Bits,Eq);
 
//----------------------------------------------------------------------
// Helper functions
//----------------------------------------------------------------------

function Bit#(AddrSz) getAddr( DataReq req );
   return  case ( req ) matches
	      tagged LoadReq  .ld : ld.addr;
	      default             : ?;
	   endcase;
endfunction

function Bit#(TagSz) getTag( DataReq req );
   return case ( req ) matches
	     tagged LoadReq  .ld : ld.tag;
	     default             : ?;
	  endcase;
endfunction

function DataReq getNextReq( DataReq req );
   return case ( req ) matches
	     tagged LoadReq  .ld : (tagged  LoadReq{tag: ld.tag, 
                                                  addr: ld.addr + fromInteger(valueof(BytesPerCacheWord))
                                                 
                                                 });
	     default             : ?;
	  endcase;
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
module mkDataCacheBlockingBramDoublePump( ICache#(DataReq,DoublePumpDataResp));

   //-----------------------------------------------------------
   // State
   
   Reg#(CacheStage) stage <- mkReg(Init);
   
   // May want to use non-regfile so that we can have zeros
   RegFile#(CacheLineIndex,Maybe#(CacheLineTag)) cacheTagRamLo   <- mkRegFileFull();
   RegFile#(CacheLineIndex,Maybe#(CacheLineTag)) cacheTagRamHi   <- mkRegFileFull();
   
   // Long, skinny reg file, with width equal to busword.
   BRAM#(SkinnyCacheIndex,BusWord)           cacheDataRam  <- mkBRAM();
   
   FIFO#(ExpandedReq)         reqQ     <- mkFIFO();
   Reg#(DataReq)              refillReq<- mkRegU(); 
   FIFOF#(DoublePumpDataResp) respQ    <- mkLFIFOF();
   
   FIFO#(MainMemReq)  mainMemReqQ  <- mkSizedFIFO(1);
   FIFO#(MainMemResp) mainMemRespQ <- mkFIFO();
   
   Reg#(CacheLineIndex) initCounter <- mkReg(1);

   // Statistics state
   Reg#(Bool)     statsEn        <- mkReg(False);
   Reg#(Int#(25)) num_accesses   <- mkReg(25'h0);
   Reg#(Int#(25)) num_misses     <- mkReg(25'h0);
   
   FIFO#(LoadCtrl) in_flight <- mkFIFO();

   //-----------------------------------------------------------
   // Name some wires
   
   //-----------------------------------------------------------
   // Initialize
   // XXX We should get rid of init.
   rule init ( stage == Init );    
      initCounter <= initCounter + 1;
      cacheTagRamLo.upd(initCounter,Invalid);
      cacheTagRamHi.upd(initCounter,Invalid);
      if ( initCounter == 0 )
	 stage <= Access;
   endrule
   
  //-----------------------------------------------------------
  // Access cache rule

  rule accessReq ( (stage == Access) && respQ.notFull());
     // I feel kinda bad about putting this addition on the CP, but we'll see
     let reqLo              = reqQ.first().reqLo;
     let reqHi              = reqQ.first().reqHi;
 
     let reqIndexLo         = getCacheLineIndex(reqLo);
     let reqTagLo           = getCacheLineTag(reqLo);
     let reqCacheLineAddrLo = getCacheLineAddr(reqLo);
     let reqCacheLineBlockAddrLo = getCacheLineBlockAddr(reqLo);

     let reqIndexHi         = getCacheLineIndex(reqHi);
     let reqTagHi           = getCacheLineTag(reqHi);
     let reqCacheLineAddrHi = getCacheLineAddr(reqHi);
     let reqCacheLineBlockAddrHi = getCacheLineBlockAddr(reqHi);
     
     // Statistics
     if ( statsEn )
	num_accesses <= num_accesses + 1;
     // Get the corresponding tag from the rams
     Maybe#(CacheLineTag) cacheLineTagLo = cacheTagRamLo.sub(reqIndexLo);
     Maybe#(CacheLineTag) cacheLineTagHi = cacheTagRamHi.sub(reqIndexHi);
     
     // Handle cache hits ...
     Bool tagMatchLo =  isValid(cacheLineTagLo) && ( unJust(cacheLineTagLo) == reqTagLo );
     Bool tagMatchHi =  isValid(cacheLineTagHi) && ( unJust(cacheLineTagHi) == reqTagHi );

     if ( tagMatchLo && tagMatchHi )
	begin
           reqQ.deq();
           case ( reqLo ) matches
              tagged LoadReq .ld :
		 begin
                    
                    SkinnyCacheIndex indexLo = truncateLSB({reqIndexLo,reqCacheLineBlockAddrLo});
                    SkinnyCacheIndex indexHi = truncateLSB({reqIndexHi,reqCacheLineBlockAddrHi});
                    
                    

                    in_flight.enq(LoadCtrl{tag: getTag(reqLo), 
                                           indexHi: truncate(getCacheLineBlockAddr(reqHi)), 
                                           indexLo: truncate(getCacheLineBlockAddr(reqLo))
                                          });// XXX may want to reuse above compute 

                    $display("Lo Index Total: %h, Index Trunk: %h",{reqIndexLo,reqCacheLineBlockAddrLo}, indexLo); 
                    $display("Hi Index Total: %h, Index Trunk: %h",{reqIndexHi,reqCacheLineBlockAddrHi}, indexHi); 

		    let read_reqLo = BRAMRequest{write:False, 
                          address: indexLo, 
                          datain:?};
                    let read_reqHi = BRAMRequest{write:False, 
                          address: indexHi, 
                          datain:?};

		    cacheDataRam.portA.request.put(read_reqLo);
		    cacheDataRam.portB.request.put(read_reqHi);

		    $display("Cache Hit: tag %h req %h reqaddr %h %h", 
                             ld.tag, 
                             reqLo, 
                             read_reqLo.address,
                             read_reqHi.address);

		    
		 end       
	   endcase
	end
     
     // Handle cache misses ...
     else if(!tagMatchLo)
	begin
           if ( statsEn )
              num_misses <= num_misses + 1;
           // since cache is read only... 
           $display("Cache Miss: addr %h", reqCacheLineAddrLo);
           mainMemReqQ.enq( LoadReq { tag: 0, addr: reqCacheLineAddrLo } );
           refillReq <= reqLo;
           stage <= RefillResp; //Hmmm need to not replicate this....    
	end
     else // I believe it is okay to have this fall through
       begin
           if ( statsEn )
              num_misses <= num_misses + 1;
           // since cache is read only... 
           $display("Cache Miss: addr %h", reqCacheLineAddrHi);
           mainMemReqQ.enq( LoadReq { tag: 0, addr: reqCacheLineAddrHi } );
           refillReq <= reqHi;
           stage <= RefillResp; //Hmmm need to not replicate this....    
       end
  endrule
   
   rule accessResp;   
      let dataLo <- cacheDataRam.portA.response.get();
      let dataHi <- cacheDataRam.portB.response.get(); 
      Vector#(TDiv#(SizeOf#(BusWord),CacheWordSize),CacheWord) lineLo = unpack(dataLo);
      Vector#(TDiv#(SizeOf#(BusWord),CacheWordSize),CacheWord) lineHi = unpack(dataHi);
      let ctrl = in_flight.first();
      $display("IndexLo final: %h", ctrl.indexLo);
      $display("IndexHi final: %h", ctrl.indexHi);
      // Do something about in-flight
      respQ.enq( LoadResp {tag: ctrl.tag, 
                           data: DoubleWord{lo:lineLo[ctrl.indexLo], hi:lineHi[ctrl.indexHi]}});
      in_flight.deq();
   endrule   

  //-----------------------------------------------------------
  // Refill response rule
  
  
  Reg#(Bit#(TLog#(BeatsPerBurst))) refillCounter <- mkReg(0);
  // this we can probably subsume, at some point.
  //Vector#(WordsPerBurst,Reg#(BusWord)) linestore <- replicateM(mkRegU);

   rule refillResp ( stage == RefillResp);
      
      let req              = refillReq;
      let reqIndex         = getCacheLineIndex(req);
      let reqTag           = getCacheLineTag(req);
      let reqCacheLineAddr = getCacheLineAddr(req);
      
      
      // May take many cycles to obtain values...
      // Write the new data into the cache and update the tag
      mainMemRespQ.deq();
      $display("Cache got main memresp %d of %d",refillCounter, valueof(BeatsPerBurst)-1);
      
    case ( mainMemRespQ.first() ) matches

      tagged LoadResp .ld :
       begin 
         let wr_req = BRAMRequest{write:True, address:{reqIndex,refillCounter}, datain:ld.data};
         $display("Cache Fill: data %h addr: %h", wr_req.datain, wr_req.address);
	 cacheDataRam.portA.request.put(wr_req);	        
         if(refillCounter == fromInteger(valueof(BeatsPerBurst)-1)) 
           begin
             cacheTagRamLo.upd(reqIndex,Valid(reqTag));
             cacheTagRamHi.upd(reqIndex,Valid(reqTag));
             stage <= Access;
             refillCounter <= 0;
           end
         else
           begin // load a single word
             refillCounter <= refillCounter + 1;    
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
    interface Put request;
      method Action put(DataReq req);
        reqQ.enq(ExpandedReq{reqLo: req, reqHi: getNextReq(req)});
      endmethod
    endinterface
    interface Get response = fifoToGet(fifofToFifo(respQ));
  endinterface

endmodule


