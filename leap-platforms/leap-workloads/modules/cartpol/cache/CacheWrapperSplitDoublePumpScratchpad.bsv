/*
Copyright (c) 2013 MIT

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

Author: Kermin Fleming.
*/

`include "awb/provides/librl_bsv.bsh"
`include "awb/provides/soft_connections.bsh"
`include "awb/provides/soft_services.bsh"
`include "awb/provides/soft_services_lib.bsh"
`include "awb/provides/soft_services_deps.bsh"
`include "awb/provides/cartpol_common.bsh"
`include "awb/provides/cartpol_cordic.bsh"
`include "awb/provides/mem_services.bsh"
`include "awb/provides/common_services.bsh"
`include "awb/provides/scratchpad_memory_common.bsh"
`include "awb/dict/VDEV_SCRATCH.bsh"

import ClientServer::*;
import Connectable::*;
import FIFO::*;
import GetPut::*;
import Vector::*;
import Complex::*;
import DefaultValue::*;

// this module accepts a cartesian coordinate (x,y), fetches the
// memory locations corresponding to coordinates
// {(x,y),(x,y+1),(x+1,y),(x+1,y+1)}, and returns their average

module [CONNECTED_MODULE] mkCacheWrapperSplitDoublePump#(Vector#(2,ICache#(DataReq,DoublePumpDataResp)) cacheIgnore) (CacheWrapper);
   
   Reg#(Bit#(TSub#(MainMemAddrSz,3))) basePtr <- mkReg(0);

   function Bit#(TSub#(MainMemAddrSz,3)) coord2Req (Coord coord);
      Bit#(TSub#(MainMemAddrSz,3)) ra = zeroExtend(basePtr) +  (zeroExtend(coord.y)*1024+zeroExtend(coord.x));
      return ra;
   endfunction
   
   function DoubleWord extData(Bit#(MainMemDataSz) resp);
      return unpack(resp);
   endfunction
   
   Vector#(2, MEMORY_IFC#(Bit#(TSub#(MainMemAddrSz,3)), Bit#(MainMemDataSz))) dataStore = newVector;
   dataStore[0] <- mkScratchpad(`VDEV_SCRATCH_BANK_A, defaultValue);
   dataStore[1] <- mkScratchpad(`VDEV_SCRATCH_BANK_B, defaultValue);

   FIFO#(Coord)     req_fifo <- mkFIFO();
   FIFO#(Bit#(32))  resp_fifo <- mkFIFO();
   Vector#(2,FIFO#(Complex#(Int#(18)))) cache_avg <- replicateM(mkFIFO());
   Vector#(2,FIFO#(DataReq)) cache_fifos <- replicateM(mkFIFO());


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
	    dataStore[0].readReq(coord2Req(rc0));
	    dataStore[1].readReq(coord2Req(rc1));
	    $display("Wrapper: req_data (%h,%h), %h %d", rc0.x, rc0.y, coord2Req(rc0), t/10);
	 end
      else
	 begin
	    dataStore[0].readReq(coord2Req(rc1));
	    dataStore[1].readReq(coord2Req(rc0));
	    $display("Wrapper: req_data (%h,%h), %h %d", rc1.x, rc1.y, coord2Req(rc1), t/10);
	 end
   endrule


   for (int idx = 0; idx < 2; idx = idx+1)
      rule resp_data;
	 let resp <- dataStore[idx].readRsp();
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
       // reset makes no sense 
   endmethod 
      
   method Action set_n(Bit#(AddrSz) new_n);
      $display("ERROR: this version of the cach wrapper does not support packed data\n \
        it assumes data is always layed out in rows of length 1024 words.\n \
        this is optimized to work on the vertex II pro which doesn't make \n \
        use of open lines.");
      $finish;
   endmethod
   
   interface mmem_server = putGetToServer(fifoToPut(req_fifo),fifoToGet(resp_fifo));
   interface mmem_client = ?;
   
endmodule
