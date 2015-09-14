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
import FIFOF::*;
import GetPut::*;
import Vector::*;

// this module accepts a cartesian coordinate (x,y), memory locations
// corresponding to coordinates {(x,y),(x,y+1),(x+1,y),(x+1,y+1)}, and
// returns their average

module mkCacheWrapper#(ICache#(DataReq,DataResp) cache) (CacheWrapper);

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

   Reg#(Bit#(2))  cache_req_ctr <- mkReg(0);
   Reg#(Bit#(2)) cache_resp_ctr <- mkReg(0);
   Reg#(Vector#(3,Bit#(DataSz))) rsp <- mkRegU();


   
   rule req_data;
      let coord = req_fifo.first();
      let xx = coord.x;
      let yy = coord.y;
      Coord rc = ?;
      case (cache_req_ctr) matches
	 0: rc = Coord{x:xx,y:yy};
	 1: rc = Coord{x:xx,y:yy+1};
	 2: rc = Coord{x:xx+1,y:yy};
	 3: begin
	       rc = Coord{x:xx+1,y:yy+1};
	       req_fifo.deq();
	    end
      endcase
      cache_req_ctr <= cache_req_ctr+1;
      cache.proc_server.request.put(coord2Req(rc));
      $display("CacheWrapper: req_data %h", coord2Req(rc));
   endrule
   
   rule resp_data;
      let resp <- cache.proc_server.response.get();
      let data = extData(resp);
      case (cache_resp_ctr) matches
	 0: rsp[0] <= data;
	 1: rsp[1] <= data;
	 2: rsp[2] <= data;
	 3: begin
	       resp_fifo.enq(((rsp[0]+rsp[1])+(rsp[2]+data)>>2));
	       $display("CacheWrapper: resp_data %h %h %h %h", rsp[0], rsp[1],rsp[2],data);
	    end
      endcase
      cache_resp_ctr <= cache_resp_ctr+1;
      $display("CacheWrapper: resp_data %h", data);
   endrule
 
   method Action reset() = cache.reset();
   interface mmem_server = putGetToServer(toPut(req_fifo),toGet(resp_fifo));
   interface mmem_client = cache.mmem_client;
 

endmodule
