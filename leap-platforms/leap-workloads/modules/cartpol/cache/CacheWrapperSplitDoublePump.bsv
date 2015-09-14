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
import Complex::*;

// this module accepts a cartesian coordinate (x,y), fetches the
// memory locations corresponding to coordinates
// {(x,y),(x,y+1),(x+1,y),(x+1,y+1)}, and returns their average

module mkCacheWrapperSplitDoublePump#(Vector#(2,ICache#(DataReq,DoublePumpDataResp)) caches) (CacheWrapper);
   
   Reg#(Bit#(AddrSz)) basePtr <- mkReg(0);

   function DataReq coord2Req (Coord coord);
      Bit#(TAdd#(AddrSz,4)) ra = zeroExtend(basePtr) +  (zeroExtend(coord.y)*1024+zeroExtend(coord.x))*4;
      return tagged LoadReq {addr:truncate(ra), tag:0};
   endfunction
   
   function DoubleWord extData(DoublePumpDataResp resp);
      return case (resp) matches
		tagged LoadResp .ld: ld.data;
		default: ?;
	     endcase;
   endfunction
   
   FIFO#(Coord)     req_fifo <- mkFIFO();
   FIFO#(Bit#(32))  resp_fifo <- mkFIFO();
   Vector#(2,FIFO#(Complex#(Int#(18)))) cache_avg <- replicateM(mkFIFO());
   Vector#(2,FIFO#(DataReq)) cache_fifos <- replicateM(mkFIFO());


   IMemArb mem_arb <- mkMemArb();   
   mkConnection(caches[0].mmem_client, mem_arb.cache0_server);   
   mkConnection(caches[1].mmem_client, mem_arb.cache1_server); 
   
   rule req_data;
      let coord = req_fifo.first();
      let xx = coord.x;
      let yy = coord.y;
      Coord rc0 = Coord{x:xx,y:yy};
      Coord rc1 = Coord{x:xx,y:yy+1};
      req_fifo.deq();
      let t <- $time;
      if (yy[0] == 1'b0)
	 begin
	    cache_fifos[0].enq(coord2Req(rc0));
	    cache_fifos[1].enq(coord2Req(rc1));
	    $display("Wrapper: req_data (%h,%h), %h %d", rc0.x, rc0.y, coord2Req(rc0), t/10);
	 end
      else
	 begin
	    cache_fifos[0].enq(coord2Req(rc1));
	    cache_fifos[1].enq(coord2Req(rc0));
	    $display("Wrapper: req_data (%h,%h), %h %d", rc1.x, rc1.y, coord2Req(rc1), t/10);
	 end
   endrule

   rule req_data_prime;
      caches[1].proc_server.request.put(cache_fifos[1].first);
      caches[0].proc_server.request.put(cache_fifos[0].first);
      cache_fifos[0].deq;
      cache_fifos[1].deq;
   endrule

   for (int idx = 0; idx < 2; idx = idx+1)
      rule resp_data;
	 let resp <- caches[idx].proc_server.response.get();
	 let data = extData(resp);
         Complex#(Int#(18)) hi = Complex{rel:extend(unpack(data.hi[31:16])), img: extend(unpack(data.hi[15:0]))};
         Complex#(Int#(18)) lo = Complex{rel:extend(unpack(data.lo[31:16])), img: extend(unpack(data.lo[15:0]))};
	 cache_avg[idx].enq(hi+lo);
      endrule
   
   rule final_avg;
      let sum = (cache_avg[0].first()+cache_avg[1].first());
      Complex#(Int#(18)) rv = cmplx(sum.rel/4,sum.img/4);
      Complex#(Int#(16)) rvTrunc = Complex{rel: truncate(rv.rel),img: truncate(rv.img)};
      cache_avg[0].deq(); cache_avg[1].deq();
      Bit#(32) res = {pack(rvTrunc.rel),pack(rvTrunc.img)};
      resp_fifo.enq(res);
   endrule   

   method Action reset();
      caches[0].reset();
      caches[1].reset();
   endmethod 
      
   method Action set_n(Bit#(AddrSz) new_n);
      $display("ERROR: this version of the cach wrapper does not support packed data\n \
        it assumes data is always layed out in rows of length 1024 words.\n \
        this is optimized to work on the vertex II pro which doesn't make \n \
        use of open lines.");
      $finish;
   endmethod
   
   interface mmem_server = putGetToServer(fifoToPut(req_fifo),fifoToGet(resp_fifo));
   interface mmem_client = mem_arb.mmem_client;
   
endmodule
