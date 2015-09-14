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
`include "awb/provides/fpga_components.bsh"
`include "awb/provides/librl_bsv_storage.bsh"
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

//----------------------------------------------------------------------
// Cache Types
//----------------------------------------------------------------------

typedef enum 
{ 
  Init,
  Access
} 
CacheStage 
deriving (Eq,Bits);

typedef enum
{
  TagLookup,
  TagUpdate
} TagStage
deriving (Eq,Bits);

typedef struct {
   Bit#(TagSz) tag;
} LoadCtrl deriving (Bits,Eq);

typedef struct {
  DataReq reqHi;
  DataReq reqLo;
} ExpandedReq deriving (Bits,Eq);

typedef struct {
  Bit#(TagSz) tag;
  Bool hitHi;
  Bool hitLo;
  CacheLineBlockIndex blockIndexHi;
  CacheLineBlockIndex blockIndexLo;
  CacheLineIndex indexHi;
  CacheLineIndex indexLo;
} AccessCtrl deriving (Bits,Eq);

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

typedef 2 OutstandingReqs;
 
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
  let index = addr >> fromInteger(valueof(CacheLineBlockSz));
  return {index[9:6],index[4:0]};
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

(* descending_urgency = "accessReq, refillResp"*)
module mkDataCacheBlockingBramDoublePumpOverlappedSkinny#(int cache_id) (ICache#(DataReq,DoublePumpDataResp));

   Reg#(CacheStage) cacheStage <- mkReg(Init);
   Reg#(TagStage) tagStage <- mkReg(TagLookup);

   // These tag banks contain the future, so that we can issue memory requests early
   RegFile#(CacheLineIndex,Maybe#(CacheLineTag)) cacheTagRamLoFuture   <- mkRegFileFull();
   RegFile#(CacheLineIndex,Maybe#(CacheLineTag)) cacheTagRamHiFuture   <- mkRegFileFull();
   
   // Long, skinny reg file, with width equal to busword.
   MEMORY_IFC#(SkinnyCacheIndex,CacheWord)           cacheDataRamHi  <- mkBRAM();
   MEMORY_IFC#(SkinnyCacheIndex,CacheWord)           cacheDataRamLo  <- mkBRAM();
   
   // Below FIFO may be reduced to index only at some point
   FIFO#(CacheLineIndex) reqRefillQ      <- mkSizedFIFO(valueof(OutstandingReqs)); 
   FIFO#(AccessCtrl)           reqQ      <- mkSizedFIFO(fromInteger(valueof(TMul#(OutstandingReqs,BeatsPerBurst))));
   FIFO#(TagCheckCtrl)    tagCheckQ      <- mkLFIFO();
   FIFO#(ExpandedReq)     memIssueQ      <- mkFIFO();
   FIFOF#(DoublePumpDataResp) respQ      <- mkLFIFOF();
   
   FIFO#(MainMemReq)    mainMemReqQ      <- mkSizedFIFO(1);
   FIFO#(MainMemResp)  mainMemRespQ      <- mkSizedFIFO(fromInteger(valueof(TMul#(OutstandingReqs,BeatsPerBurst))));
   
   Reg#(CacheLineIndex) initCounter <- mkReg(1);

   // Statistics state
   Reg#(Bool)     statsEn        <- mkReg(False);
   Reg#(Int#(25)) num_accesses   <- mkReg(25'h0);
   Reg#(Int#(25)) num_misses     <- mkReg(25'h0);
   
   // State for tracking Hits/Misses
   Reg#(Bool) issuedLo <- mkReg(False);
   Reg#(Bool) issuedHi <- mkReg(False);

   Reg#(Bool) loadedHi <- mkReg(False);
   Reg#(Bool) loadedLo <- mkReg(False);
   
   FIFO#(Bit#(0))  ld_tokens <- mkSizedFIFO(valueof(OutstandingReqs));
   FIFO#(LoadCtrl) in_flight <- mkFIFO();
   
   //-----------------------------------------------------------
   // Initialize

   rule init ( cacheStage == Init );    
      initCounter <= initCounter + 1;      
      cacheTagRamLoFuture.upd(initCounter,Invalid);
      cacheTagRamHiFuture.upd(initCounter,Invalid);
      if ( initCounter == 0 )
	 cacheStage <= Access;
   endrule
   
   //-----------------------------------------------------------
   // Access cache rule

   rule accessReq (cacheStage == Access);
      // Statistics
      if ( statsEn )
	 num_accesses <= num_accesses + 1;
      let ctrl = reqQ.first;
      // We either a) got a hit or b) loaded the result into the cache
      if ( (ctrl.hitLo || loadedLo) && (ctrl.hitHi || loadedHi) )
	 begin
            loadedLo <= False;
            loadedHi <= False;
            reqQ.deq();
            SkinnyCacheIndex skinnyIndexLo = truncateLSB({ctrl.indexLo,ctrl.blockIndexLo});
            SkinnyCacheIndex skinnyIndexHi = truncateLSB({ctrl.indexHi,ctrl.blockIndexHi});
	    in_flight.enq(LoadCtrl{tag: ctrl.tag });
	    
            $display("CacheID(%d)Lo Index Total: %h, Index Trunk: %h %d",cache_id,ctrl.indexLo, skinnyIndexLo, $time); 
            $display("CacheID(%d)Hi Index Total: %h, Index Trunk: %h %d",cache_id,ctrl.indexHi, skinnyIndexHi, $time); 
	    
	    if (ctrl.blockIndexLo[0] == 1'b0)
	       begin
		  cacheDataRamLo.readReq(skinnyIndexLo);
		  cacheDataRamHi.readReq(skinnyIndexHi);
	       end
	    else
	       begin
		  cacheDataRamLo.readReq(skinnyIndexHi);
		  cacheDataRamHi.readReq(skinnyIndexLo);
	       end
	    
	    
	    
	    $display("CacheID(%d)Cache Hit: tag %h  reqaddr %h %h", cache_id,
		     ctrl.tag,                               
		     skinnyIndexLo,
                     skinnyIndexHi);
	 end
      
      // Handle cache misses ...
      // Since we have decoupled the request, we merely go to collect 
      // the result now.
      else if(!(ctrl.hitLo || loadedLo))
	 begin
            if ( statsEn )
               num_misses <= num_misses + 1;   
            loadedLo <= True;
	    ld_tokens.deq();
	 end
      else // I believe it is okay to have this fall through
	 begin
            if ( statsEn )
               num_misses <= num_misses + 1;
            loadedHi <= True;
	    ld_tokens.deq();
	 end
   endrule

   
   //may need to move this tag match == after this cycle.
   rule tagLookup (cacheStage == Access); // Necessary?
      
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
      
      Bool tagEvictLo =  isValid(cacheLineTagLo) && ( unJust(cacheLineTagLo) != reqTagLo );
      Bool tagEvictHi =  isValid(cacheLineTagHi) && ( unJust(cacheLineTagHi) != reqTagHi );      
      
      let t <- $time;
      $display("CacheID(%d) tagLookup %h", cache_id, t/10);
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
      let t <- $time;
      $display("CacheID(%d) tagCheck  %h", cache_id, t/10);
      if ((tagData.hitLo || issuedLo) && ((tagData.hitHi || issuedHi) || loSubsumesHi))
	 begin
            tagCheckQ.deq;
            reqQ.enq(AccessCtrl {tag:          tagData.tag,
                                 hitHi:        tagData.hitHi || loSubsumesHi,
                                 hitLo:        tagData.hitLo, 
                                 blockIndexHi: tagData.blockIndexHi,
                                 blockIndexLo: tagData.blockIndexLo,
                                 indexHi:      tagData.indexHi,
                                 indexLo:      tagData.indexLo
			      });
            issuedHi <= False;
            issuedLo <= False;
	 end     
      // Handle cache misses ...
      // Order matters !!!
      else if(!tagData.hitLo  && !issuedLo)
	 begin
            issuedLo <= True;
            // since cache is read only... 
            $display("CacheID(%d)Cache Miss: lo addr %h", cache_id,tagData.addrLo);
            mainMemReqQ.enq( LoadReq { tag: 0, addr: tagData.addrLo } );
            reqRefillQ.enq(tagData.indexLo);   
            tagStage <= TagUpdate;  
	 end
      else if(!tagData.hitHi  && !issuedHi && !loSubsumesHi)
	 begin
            issuedHi <= True;
            $display("CacheID(%d)Cache Miss: hi addr %h", cache_id,tagData.addrHi);
            mainMemReqQ.enq( LoadReq { tag: 0, addr: tagData.addrHi } );
            reqRefillQ.enq(tagData.indexHi);
            tagStage <= TagUpdate;          
	 end
   endrule

   rule updateTags (cacheStage == Access && tagStage == TagUpdate);
     tagStage <= TagLookup;
     if(issuedHi) 
       begin
         cacheTagRamLoFuture.upd(tagData.indexHi,Valid(tagData.tagHi)); 
         cacheTagRamHiFuture.upd(tagData.indexHi,Valid(tagData.tagHi));           
       end
     else 
       begin
         cacheTagRamLoFuture.upd(tagData.indexLo,Valid(tagData.tagLo));
         cacheTagRamHiFuture.upd(tagData.indexLo,Valid(tagData.tagLo));          
       end        
   endrule
   
   rule accessResp (cacheStage == Access);
      let dataLo <- cacheDataRamLo.readRsp();
      let dataHi <- cacheDataRamHi.readRsp();
      let t <- $time;
      $display("CacheID(%d)final: %h %h", cache_id,dataLo,dataHi);
      respQ.enq( LoadResp {tag: in_flight.first.tag, 
                           data: DoubleWord{lo:dataLo, hi:dataHi}});
      in_flight.deq();
   endrule   
   
   
   Reg#(Bit#(TLog#(BeatsPerBurst))) refillCounter <- mkReg(0);
   // this we can probably subsume, at some point.
   //Vector#(WordsPerBurst,Reg#(BusWord)) linestore <- replicateM(mkRegU);

   rule refillResp (cacheStage==Access);
      
      $display("CacheID(%d)Cache got main memresp %d of %d",cache_id,refillCounter, valueof(BeatsPerBurst)-1);
      // May take many cycles to obtain values...
      // Write the new data into the cache and update the tag
      mainMemRespQ.deq();
      
      case ( mainMemRespQ.first() ) matches
	 
	 tagged LoadResp .ld :
	    begin 
	       let wr_address = {reqRefillQ.first,refillCounter};            
	       Vector#(TDiv#(SizeOf#(BusWord),CacheWordSize),CacheWord) line = unpack(ld.data);
	       $display("CacheID(%d)Cache Fill: wr_address %h dataLo: %h dataHi: %h", cache_id,wr_address, line[0],line[1]);
	       cacheDataRamLo.write(wr_address, line[0]);	        
	       cacheDataRamHi.write(wr_address, line[1]);	        
               if(refillCounter == fromInteger(valueof(BeatsPerBurst)-1)) 
		  begin
		     refillCounter <= 0;
		     reqRefillQ.deq();     
		     ld_tokens.enq(?);
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
      interface Get request  = fifoToGet(mainMemReqQ);
      interface Put response = fifoToPut(mainMemRespQ);
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
