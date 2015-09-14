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
import Vector::*;
import FShow::*;

//----------------------------------------------------------------------
// Cache Types
//----------------------------------------------------------------------

typedef Bit#(TLog#(BeatsPerBurst)) BRAMSel;

typedef enum 
{ 
  Init,
  Access
} 
CacheStage 
deriving (Eq,Bits);

typedef enum {
   TagLookup,
   TagUpdate
} TagStage
deriving (Eq,Bits);

typedef struct {
   Bit#(TagSz)   tag;
   BRAMSel bsEven;
   BRAMSel bsOdd;
} LoadCtrl deriving (Bits,Eq);

typedef struct {
   DataReq reqHi;
   DataReq reqLo;
} ExpandedReq deriving (Bits,Eq);

typedef struct {
   Bit#(TagSz) tag;
   Bool hitHi;
   Bool hitLo;
   CacheLineBlockIndex blkIdxLo;
   CacheLineIndex lineIndexHi;
   CacheLineIndex lineIndexLo;
} AccessCtrl deriving (Bits,Eq);

typedef struct {
   BRAMSel bsEven;
   BRAMSel bsOdd;   
   CacheLineIndex raOdd;
   CacheLineIndex raEven;
} AccessCtrlp deriving (Bits,Eq);

typedef struct {
  Bit#(TagSz) tag;
  Bool hitHi;
  Bool hitLo;
  Bit#(AddrSz) addrHi;
  Bit#(AddrSz) addrLo;
  CacheLineTag tagHi;
  CacheLineTag tagLo;
  CacheLineBlockIndex blockIndexHi;
  CacheLineBlockIndex blockIndexLo;
  CacheLineIndex indexHi;
  CacheLineIndex indexLo;
} TagCheckCtrl deriving (Bits,Eq);

instance FShow#(LoadCtrl);
   function Fmt fshow (LoadCtrl lc);
      return fshow("[x]");
   endfunction
endinstance

instance FShow#(ExpandedReq);
   function Fmt fshow (ExpandedReq er);
      return fshow("[x]");
   endfunction
endinstance

instance FShow#(AccessCtrl);
   function Fmt fshow (AccessCtrl ac);
      return fshow("[x]");
   endfunction
endinstance

instance FShow#(MemResp#(a,b));
   function Fmt fshow (MemResp#(a,b) mr);
      return fshow("[x]");
   endfunction
endinstance

instance FShow#(MemReq#(a,b,c));
   function Fmt fshow (MemReq#(a,b,c) mr);
      return fshow("[x]");
   endfunction
endinstance

typedef 4 OutstandingReqs;
 
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
	     tagged LoadReq  .ld : (tagged LoadReq{tag: ld.tag, 
						   addr: ld.addr + fromInteger(valueof(BytesPerCacheWord))});
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
(* descending_urgency = "tagCheck, tagLookup, updateTags"*)
module mkDataCacheBlockingBramDoublePumpOverlappedGang#(int cache_id) (ICache#(DataReq,DoublePumpDataResp));

   Reg#(CacheStage) cacheStage <- mkReg(Init);
   Reg#(TagStage) tagStage <- mkReg(TagLookup);

   // tag banks
   RegFile#(CacheLineIndex,Maybe#(CacheLineTag))     cacheTagRamLoFuture   <- mkRegFileFull();
   RegFile#(CacheLineIndex,Maybe#(CacheLineTag))     cacheTagRamHiFuture   <- mkRegFileFull();

   // data storage -- cache lines are striped across the skinny brams
   Vector#(BeatsPerBurst,BRAM#(CacheLineIndex,CacheWord))  cacheDataRamsOdd  <- replicateM(mkBRAM_Full());
   Vector#(BeatsPerBurst,BRAM#(CacheLineIndex,CacheWord)) cacheDataRamsEven  <- replicateM(mkBRAM_Full());
   
   // Below FIFO may be reduced to index only at some point
   FIFOF#(CacheLineIndex) reqRefillQ      <- mkSizedFIFOF(valueof(OutstandingReqs)); 
   FIFOF#(AccessCtrl)           reqQ      <- mkSizedFIFOF(fromInteger(valueof(TMul#(OutstandingReqs,BeatsPerBurst))));
   FIFO#(AccessCtrlp)     reqQ_prime      <- mkFIFO();
   FIFOF#(ExpandedReq)     memIssueQ      <- mkFIFOF();
   FIFOF#(DoublePumpDataResp)  respQ      <- mkLFIFOF();
   FIFO#(TagCheckCtrl)     tagCheckQ      <- mkLFIFO();
   
   FIFOF#(MainMemReq)    mainMemReqQ      <- mkSizedFIFOF(2);
   FIFOF#(MainMemResp)  mainMemRespQ      <- mkSizedFIFOF(fromInteger(valueof(TMul#(OutstandingReqs,BeatsPerBurst))));
   
   Reg#(CacheLineIndex)  initCounter      <- mkReg(1);

   // Statistics state
   Reg#(Bool)     statsEn        <- mkReg(False);
   Reg#(Int#(25)) num_accesses   <- mkReg(25'h0);
   Reg#(Int#(25)) num_misses     <- mkReg(25'h0);
   
   // State for tracking Hits/Misses
   Reg#(Bool) issuedLo <- mkReg(False);
   Reg#(Bool) issuedHi <- mkReg(False);

   Reg#(Bool) loadedHi <- mkReg(False);
   Reg#(Bool) loadedLo <- mkReg(False);
   
   // not sure if this is right
   FIFOF#(Bit#(0))  ld_tokens <- mkSizedFIFOF(valueof(OutstandingReqs));
   // needs to be long enough to cover BRAM read latency
   FIFOF#(LoadCtrl) in_flight <- mkSizedFIFOF(valueof(OutstandingReqs));
   


//    rule print_fifo_status;
//       $display("CacheID(%h) reqRefillQ   ",cache_id,fshow(reqRefillQ));
//       $display("CacheID(%h) reqQ         ",cache_id,fshow(reqQ));
//       $display("CacheID(%h) memIssueQ    ",cache_id,fshow(memIssueQ));
//       $display("CacheID(%h) respQ        ",cache_id,fshow(respQ));
//       $display("CacheID(%h) mainMemReqQ  ",cache_id,fshow(mainMemReqQ));
//       $display("CacheID(%h) mainMemRespQ ",cache_id,fshow(mainMemRespQ));
//       $display("CacheID(%h) ld_tokens    ",cache_id,fshow(ld_tokens));
//       $display("CacheID(%h) in_flight    ",cache_id,fshow(in_flight));
//    endrule



   rule init ( cacheStage == Init );    
      initCounter <= initCounter + 1;      
      cacheTagRamLoFuture.upd(initCounter,Invalid);
      cacheTagRamHiFuture.upd(initCounter,Invalid);
      if ( initCounter == 0 )
	 cacheStage <= Access;
   endrule
   
   let accessAGuard = ((reqQ.first.hitLo || loadedLo) && (reqQ.first.hitHi || loadedHi));
   
   rule accessReqA (cacheStage == Access && accessAGuard);
      let ctrl = reqQ.first;
      // Statistics
      if ( statsEn )
	 num_accesses <= num_accesses + 1;

      // We either a) got a hit or b) loaded the result into the cache
      loadedLo <= False;
      loadedHi <= False;
      reqQ.deq();
	    
      BRAMSel bramLo = truncate((ctrl.blkIdxLo)>>1);
      
	    
      let t <- $time;
      $display("CacheID(%h) cycl req %d", cache_id, t/10);
      
      if (ctrl.blkIdxLo[0] == 1'b0)
	 begin
	    reqQ_prime.enq(AccessCtrlp{bsEven:bramLo, bsOdd:bramLo, raEven:ctrl.lineIndexLo, raOdd:ctrl.lineIndexHi});
	    in_flight.enq(LoadCtrl{tag: ctrl.tag, bsOdd:bramLo, bsEven:bramLo});
	 end
      else
	 begin
	    reqQ_prime.enq(AccessCtrlp{bsEven:bramLo+1, bsOdd:bramLo, raEven:ctrl.lineIndexHi, raOdd:ctrl.lineIndexLo});
	    in_flight.enq(LoadCtrl{tag: ctrl.tag, bsOdd:bramLo, bsEven:bramLo+1});	   
	 end
      
   endrule
   
   rule accessReqC;
      let ctrl = reqQ_prime.first;
      reqQ_prime.deq;      
      cacheDataRamsOdd[ctrl.bsOdd].read_req(ctrl.raOdd);
      cacheDataRamsEven[ctrl.bsEven].read_req(ctrl.raEven);
      $display("CacheID(%h) Even[%h].read_req(%h)",cache_id, ctrl.bsEven, ctrl.raEven); 
      $display("CacheID(%h)  Odd[%h].read_req(%h)",cache_id, ctrl.bsOdd,  ctrl.raOdd); 
   endrule
   
   let accessBGuard =  (!(reqQ.first.hitLo || loadedLo)) || (!(reqQ.first.hitHi || loadedHi));
   
   rule accessReqB (cacheStage == Access && accessBGuard);
      
      let ctrl = reqQ.first;
      if(!(ctrl.hitLo || loadedLo))
	 begin
            if ( statsEn )
               num_misses <= num_misses + 1;   
            loadedLo <= True;
	    $display("CacheID(%h) ld_tokens.deq()",cache_id); 
	    ld_tokens.deq();
	 end
      else if (!(ctrl.hitHi || loadedHi))
	 begin
            if ( statsEn )
               num_misses <= num_misses + 1;
            loadedHi <= True;
	    $display("CacheID(%h) ld_tokens.deq()",cache_id); 
	    ld_tokens.deq();
	 end
   endrule

      
   rule tagLookup (cacheStage == Access);

      let reqLo              = memIssueQ.first().reqLo;
      let reqHi              = memIssueQ.first().reqHi;
      
      let reqIndexLo         = getCacheLineIndex(reqLo);
      let reqTagLo           = getCacheLineTag(reqLo);
      let reqCacheLineAddrLo = getCacheLineAddr(reqLo);
      let reqCacheLineBlockAddrLo = getCacheLineBlockAddr(reqLo);
      
      let reqIndexHi         = getCacheLineIndex(reqHi);
      let reqTagHi           = getCacheLineTag(reqHi);
      let reqCacheLineAddrHi = getCacheLineAddr(reqHi);
      let reqCacheLineBlockAddrHi = getCacheLineBlockAddr(reqHi);
      
      // Get the corresponding tag from the rams
      Maybe#(CacheLineTag) cacheLineTagLo = cacheTagRamLoFuture.sub(reqIndexLo);
      Maybe#(CacheLineTag) cacheLineTagHi = cacheTagRamHiFuture.sub(reqIndexHi);
      
      // Handle cache hits ...
      Bool tagMatchLo =  isValid(cacheLineTagLo) && ( unJust(cacheLineTagLo) == reqTagLo );
      Bool tagMatchHi =  isValid(cacheLineTagHi) && ( unJust(cacheLineTagHi) == reqTagHi );   
   
      let t <- $time;
      $display("CacheID(%h) tagLookup[%h] %h", cache_id, reqIndexLo, t/10);
      memIssueQ.deq;
      tagCheckQ.enq(TagCheckCtrl {tag:        getTag(reqLo),
				  tagHi:        reqTagHi,
				  tagLo:        reqTagLo,
				  addrHi:       reqCacheLineAddrHi,
				  addrLo:       reqCacheLineAddrLo,
				  hitHi:        tagMatchHi, 
				  hitLo:        tagMatchLo, 
				  blockIndexHi: reqCacheLineBlockAddrHi,
				  blockIndexLo: reqCacheLineBlockAddrLo,
				  indexHi:      reqIndexHi,
				  indexLo:      reqIndexLo
				  }); 
   endrule

   let tagData = tagCheckQ.first;
   rule tagCheck (cacheStage == Access && tagStage == TagLookup);
      let loSubsumesHi = (tagData.addrLo == tagData.addrHi);      
      if ((tagData.hitLo || issuedLo) && ((tagData.hitHi || issuedHi) || loSubsumesHi))
	 begin
	    let t <- $time;
	    $display("CacheID(%h) tagCheck (hit)  %h", cache_id, tagData.blockIndexLo);
            tagCheckQ.deq;
            reqQ.enq(AccessCtrl {tag:          tagData.tag,
				 hitHi:        tagData.hitHi || loSubsumesHi,
				 hitLo:        tagData.hitLo,
				 blkIdxLo:     tagData.blockIndexLo,
				 lineIndexHi:  tagData.indexHi,
				 lineIndexLo:  tagData.indexLo
				 });
	    issuedHi <= False;
            issuedLo <= False;
	 end     
      // Handle cache misses ...
      else if(!tagData.hitLo  && !issuedLo)
	 begin
	    issuedLo <= True;
            // since cache is read only...
            $display("CacheID(%h) Cache Miss Lo: addr %h %h", cache_id, tagData.addrLo, tagData.indexLo);
            mainMemReqQ.enq( LoadReq { tag: 0, addr: tagData.addrLo } );
            reqRefillQ.enq(tagData.indexLo);
	    tagStage <= TagUpdate;          
	 end
      else if(!tagData.hitHi  && !issuedHi && !loSubsumesHi)
	 begin
            issuedHi <= True;
            $display("CacheID(%h) Cache Miss Hi: addr %h %h", cache_id, tagData.addrHi, tagData.indexHi);
            mainMemReqQ.enq( LoadReq { tag: 0, addr: tagData.addrHi } );
            reqRefillQ.enq(tagData.indexHi);     
            tagStage <= TagUpdate;          
	 end
   endrule

   rule updateTags (cacheStage == Access && tagStage == TagUpdate);
      tagStage <= TagLookup;
      if(issuedHi) 
	 begin
	    $display("CacheID(%h) updateTags[%h] = %h", cache_id,  tagData.indexHi, tagData.tagHi);
            cacheTagRamLoFuture.upd(tagData.indexHi,Valid(tagData.tagHi));
            cacheTagRamHiFuture.upd(tagData.indexHi,Valid(tagData.tagHi));           
	 end
      else 
	 begin
	    $display("CacheID(%h) updateTags[%h] = %h", cache_id,  tagData.indexLo, tagData.tagLo);
            cacheTagRamLoFuture.upd(tagData.indexLo,Valid(tagData.tagLo));
            cacheTagRamHiFuture.upd(tagData.indexLo,Valid(tagData.tagLo));          
	 end        
   endrule
   
   rule accessResp (cacheStage == Access);
      let ctrl = in_flight.first();
      let dataLo <- cacheDataRamsEven[ctrl.bsEven].read_resp();
      let dataHi <- cacheDataRamsOdd[ctrl.bsOdd].read_resp();
      $display("CacheID(%h)final: %h %h", cache_id,dataLo,dataHi);
      respQ.enq( LoadResp {tag: ctrl.tag,
			   data: DoubleWord{lo:dataLo, hi:dataHi}});
      in_flight.deq();
   endrule   

   //-----------------------------------------------------------
   // Refill response rule
   
   
   Reg#(Bit#(TLog#(BeatsPerBurst))) refillCounter <- mkReg(0);
   
   rule refillResp (cacheStage == Access);
      
      // May take many cycles to obtain values...
      // Write the new data into the cache and update the tag
      mainMemRespQ.deq();
      
      case ( mainMemRespQ.first() ) matches
	 tagged LoadResp .ld : 
	    begin
	       let wr_address = reqRefillQ.first;
	       BRAMSel ram_sel = refillCounter;
	       Vector#(TDiv#(SizeOf#(BusWord),CacheWordSize),CacheWord) line = unpack(ld.data);	       
	       cacheDataRamsEven[ram_sel].write(wr_address, line[0]);	        
	       cacheDataRamsOdd[ram_sel].write(wr_address, line[1]);	        
	       let t <- $time;	       
	       $display("CacheID(%h) cycl ref %d", cache_id, t/10);
	       $display("CacheID(%h) Cache Fill Lo[%h]: wr_address %h dataLo: %h", cache_id, ram_sel,   wr_address, line[0]);
	       $display("CacheID(%h) Cache Fill Hi[%h]: wr_address %h dataHi: %h", cache_id, ram_sel, wr_address, line[1]);

               if(refillCounter == fromInteger(valueof(BeatsPerBurst)-1)) 
		  begin
		     refillCounter <= 0;
		     reqRefillQ.deq();
		     ld_tokens.enq(?);
		     $display("CacheID(%h) ld_tokens.enq()", cache_id);
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
      cacheStage <= Init;
   endmethod 
   
  interface Client mmem_client;
    interface Get request  = fifoToGet(fifofToFifo(mainMemReqQ));
    interface Put response = fifoToPut(fifofToFifo(mainMemRespQ));
  endinterface

  interface Server proc_server;
    interface Put request;
      method Action put(DataReq req);
        memIssueQ.enq(ExpandedReq{reqLo: req, reqHi: getNextReq(req)});
      endmethod
    endinterface
    interface Get response = fifoToGet(fifofToFifo(respQ));
  endinterface

endmodule


