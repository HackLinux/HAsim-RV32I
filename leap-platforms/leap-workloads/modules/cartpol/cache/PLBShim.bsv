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
`include "awb/provides/mem_services.bsh"
`include "awb/provides/common_services.bsh"
`include "awb/provides/scratchpad_memory_common.bsh"
`include "awb/dict/VDEV_SCRATCH.bsh"

import FIFO::*;
import GetPut::*;
import ClientServer::*;
import Connectable::*;
import DefaultValue::*;

// this shim sits between the PLB and the cache.

interface PLBShim#(type mem_req_t, type mem_resp_t);
   interface Server#(mem_req_t,mem_resp_t) mem_server;   
   method Bool initialized();
endinterface


module [CONNECTED_MODULE] mkPLBShim (PLBShim#(MainMemReq,MainMemResp));
   
   // We should add an initialization step. Since this workload is not data dependent, the actual values don't matter though.

   MEMORY_IFC#(Bit#(TSub#(MainMemAddrSz,3)), Bit#(MainMemDataSz)) dataStore <-
       mkScratchpad(`VDEV_SCRATCH_BANK_A, defaultValue);

   Reg#(Bit#(TLog#(BeatsPerBurst)))     burstCnt <- mkReg(0);
   FIFO#(Bit#(MainMemTagSz))            main_tags <- mkSizedFIFO(valueof(BeatsPerBurst));

   Reg#(Bool) init <- mkReg(False);
   Reg#(Bit#(TSub#(MainMemAddrSz,3))) initAddr <- mkReg(0);


   FIFO#(MainMemReq)  cache_to_plb <- mkFIFO();
   FIFO#(MainMemResp) plb_to_cache <- mkSizedFIFO(128);
   
   rule doInit(!init);
       dataStore.write(initAddr,0);
       initAddr <= initAddr + 1;
       if(initAddr + 1 == 0)
       begin
           init <= True;
       end
   endrule


   // only doing loads
   rule req_to_plb if(init);
      let req = cache_to_plb.first();            
      case (req) matches
	 tagged LoadReq .ld :
	    begin

               // sanity check the data format.
               if(ld.addr[2:0] != 0) 
               begin
                   $display("Loads were malformed");
                   $finish;
               end
	      
	       let addr = truncate((ld.addr >> 3) + zeroExtend(burstCnt));
	       main_tags.enq(ld.tag);
	       dataStore.readReq(addr);

	       $display("req_to_plb %x", ld.addr);
               //$display("burstCnt %d", valueOf(BeatsPerBurst));

	       burstCnt <= burstCnt + 1;
	       if(burstCnt + 1 == 0)
	       begin
                  cache_to_plb.deq();   
               end
	    end
	 default:
	    begin
               $display("PLB Shim got a bogus request");
               $finish;
            end
      endcase
   endrule
   
   rule resp_from_plb;
      let data <- dataStore.readRsp();
      //$display("resp_from_plb %x", word);
      
      let resp = tagged LoadResp {tag:main_tags.first(),data: data};
      plb_to_cache.enq(resp);
      main_tags.deq();
   endrule   
   
   interface Server mem_server = putGetToServer(fifoToPut(cache_to_plb),fifoToGet(plb_to_cache));
      
   method initialized = init;
endmodule
