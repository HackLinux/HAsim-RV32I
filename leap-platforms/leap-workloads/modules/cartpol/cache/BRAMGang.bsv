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

Author: Myron King
 
*/
import MemTypes::*;
import PLBMasterDefaultParameters::*;
import Vector::*;
import BRAM::*;
import GetPut::*;
import ClientServer::*;
import ClientServerUtils::*;
import FIFO::*;


module mkBRAMGang (BRAM#(CacheLineIndex,CacheLine));

   Vector#(BeatsPerBurst,BRAM#(CacheLineIndex,BusWord))  cacheDataRams  <- replicateM(mkBRAM());   
   
   FIFO#(BRAMRequest#(CacheLineIndex,CacheLine)) req_fifo_a <- mkFIFO();
   FIFO#(CacheLine)                             resp_fifo_a <- mkFIFO();   
   
   FIFO#(BRAMRequest#(CacheLineIndex,CacheLine)) req_fifo_b <- mkFIFO();
   FIFO#(CacheLine)                             resp_fifo_b <- mkFIFO();   
   
   let vv = fromInteger(valueOf(BeatsPerBurst));

   function BRAMRequest#(CacheLineIndex,BusWord) xlate (BRAMRequest#(CacheLineIndex,CacheLine) in, int sel);
      Vector#(BeatsPerBurst,BusWord) words = unpack(in.datain);
      return BRAMRequest{write:in.write, address:in.address, datain:words[sel]};
   endfunction
   
   rule read_req_a;
      let req = req_fifo_a.first();
      req_fifo_a.deq();
      for(int sel = 0; sel < vv; sel = sel+1)
	 cacheDataRams[sel].portA.request.put(xlate(req,sel));
   endrule
   
   rule read_req_b;
      let req = req_fifo_b.first();
      req_fifo_b.deq();
      for(int sel = 0; sel < vv; sel = sel+1)
	 cacheDataRams[sel].portB.request.put(xlate(req,sel));
   endrule
   
   rule read_resp_a;
      Vector#(BeatsPerBurst,BusWord) words = ?;
      for(int sel = 0; sel < vv; sel = sel+1)
	 words[sel] <- cacheDataRams[sel].portA.response.get();
      resp_fifo_a.enq(pack(words));
   endrule

   rule read_resp_b;
      Vector#(BeatsPerBurst,BusWord) words = ?;
      for(int sel = 0; sel < vv; sel = sel+1)
	 words[sel] <- cacheDataRams[sel].portB.response.get();
      resp_fifo_b.enq(pack(words));
   endrule

   interface portA = putGetToServer(fifoToPut(req_fifo_a),fifoToGet(resp_fifo_a));
   interface portB = putGetToServer(fifoToPut(req_fifo_b),fifoToGet(resp_fifo_b));   
   
endmodule