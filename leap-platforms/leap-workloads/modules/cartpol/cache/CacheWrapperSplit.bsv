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

Author: Myron King.
*/

`include "awb/provides/librl_bsv.bsh"
`include "awb/provides/soft_connections.bsh"
`include "awb/provides/soft_services.bsh"
`include "awb/provides/soft_services_lib.bsh"
`include "awb/provides/soft_services_deps.bsh"
`include "awb/provides/cartpol_common.bsh"
`include "awb/provides/cartpol_cordic.bsh"


import ClientServer::*;
import Connectable::*;
import FIFO::*;
import GetPut::*;
import Vector::*;


// this module accepts a cartesian coordinate (x,y), memory locations
// corresponding to coordinates {(x,y),(x,y+1),(x+1,y),(x+1,y+1)}, and
// returns their average

module mkCacheWrapperSplit#(Vector#(2,ICache#(DataReq,DataResp)) caches) (CacheWrapper);
   
   Reg#(Bit#(AddrSz)) basePtr <- mkReg(0);   

   function DataReq coord2Req (Coord coord);
      Bit#(AddrSz) ra = basePtr + 
                        (zeroExtend(coord.y)*1024+zeroExtend(coord.x))*4;   
      return tagged LoadReq {addr:ra, tag:0};      
   endfunction
   
   function Bit#(DataSz) extData(DataResp resp);
      return case (resp) matches
		tagged LoadResp .ld: ld.data;
		default: ?;
	     endcase;
   endfunction
   
   FIFO#(Coord)     req_fifo <- mkFIFO();
   FIFO#(Bit#(32)) resp_fifo <- mkFIFO();

   Reg#(Bit#(1))              cache_req_ctr <- mkReg(0);
   Vector#(2,Reg#(Bit#(1)))  cache_resp_ctr <- replicateM(mkReg(0));
   Vector#(2,Reg#(Bit#(DataSz)))  cache_rsp <- replicateM(mkRegU());
   Vector#(2,FIFO#(Bit#(DataSz))) cache_avg <- replicateM(mkFIFO());


   IMemArb mem_arb <- mkMemArb();   
   mkConnection(caches[0].mmem_client, mem_arb.cache0_server);   
   mkConnection(caches[1].mmem_client, mem_arb.cache1_server); 
   
   rule req_data;
      let coord = req_fifo.first();
      let xx = coord.x;
      let yy = coord.y;
      Coord rc0 = ?;
      Coord rc1 = ?;
      case (cache_req_ctr) matches
	 0: begin
	       rc0 = Coord{x:xx,y:yy};
	       rc1 = Coord{x:xx,y:yy+1};
	    end
	 1: begin
	       rc0 = Coord{x:xx+1,y:yy};
	       rc1 = Coord{x:xx+1,y:yy+1};
	       req_fifo.deq();
	    end
      endcase
      cache_req_ctr <= cache_req_ctr+1;
      caches[0].proc_server.request.put(coord2Req(rc0));
      caches[1].proc_server.request.put(coord2Req(rc1));
   endrule

   for (int idx = 0; idx < 2; idx = idx+1)
      rule resp_data;
	 let resp <- caches[idx].proc_server.response.get();
	 let data = extData(resp);
	 case (cache_resp_ctr[idx]) matches
	    0: cache_rsp[idx] <= data;
	    1: cache_avg[idx].enq((cache_rsp[idx]+data)>>1);
	 endcase
	 cache_resp_ctr[idx] <= cache_resp_ctr[idx]+1;
      endrule
   
   rule final_avg;
      let rv = (cache_avg[0].first()+cache_avg[1].first())>>1;
      cache_avg[0].deq(); cache_avg[1].deq();
      resp_fifo.enq(rv);
   endrule
   
   method Action reset();
      caches[0].reset();
      caches[1].reset();
   endmethod 

   interface mmem_server = putGetToServer(fifoToPut(req_fifo),fifoToGet(resp_fifo));
   interface mmem_client = mem_arb.mmem_client;
   
endmodule
