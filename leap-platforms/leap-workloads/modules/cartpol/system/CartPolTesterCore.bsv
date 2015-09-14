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

Author: Kermin Fleming
*/

`define COMPUTE_1

`include "asim/provides/librl_bsv.bsh"
`include "awb/provides/soft_connections.bsh"
`include "awb/provides/soft_services.bsh"
`include "awb/provides/soft_services_lib.bsh"
`include "awb/provides/soft_services_deps.bsh"
`include "awb/provides/cartpol_common.bsh"
`include "awb/provides/cartpol_cache.bsh"
`include "awb/provides/cartpol_core.bsh"
`include "awb/provides/cartpol_cordic.bsh"
`include "awb/rrr/remote_server_stub_CARTPOLCONTROLRRR.bsh"

import FIFO::*;
import GetPut::*;
import ClientServer::*;
import Connectable::*;
import Vector::*;
import StmtFSM::*;
import Float::*;

typedef enum{
   Idle,
   Kickoff,
   Requesting
} TesterState deriving (Bits,Eq);


module [CONNECTED_MODULE] mkConnectedApplication (Empty);
 
   ServerStub_CARTPOLCONTROLRRR serverStub <- mkServerStub_CARTPOLCONTROLRRR();
  
   Reg#(TesterState)                     state <- mkReg(Idle);
   Reg#(Bit#(32))                        cycleCount <- mkReg(0);
   
   // core generates the indices
   ComputeTop computer <- mkComputeTop;

   // this shim sits between the cache and the main memory
   PLBShim#(MainMemReq,MainMemResp) plbShim <- mkPLBShim();

   // cache-wrapper performs address translation and lookup
   Vector#(2,ICache#(DataReq,DoublePumpDataResp)) caches;
   caches[0] <- mkDataCacheBlockingBramDoublePumpOverlappedSkinny(0);
   caches[1] <- mkDataCacheBlockingBramDoublePumpOverlappedSkinny(1);

   CacheWrapper cacheWrap <- mkCacheWrapperSplitDoublePump(caches); 
   mkConnection(cacheWrap.mmem_client, plbShim.mem_server);

   // State for dealing with main mem writeback
   Reg#(PLBAddr)                      storeAddress <- mkRegU; 
   Reg#(Bool)                         hiWord       <- mkReg(False); 
   Reg#(Bit#(32))                    lowerHalfWord <- mkRegU;
   Reg#(Bit#(TAdd#(1,TLog#(WordsPerBurst)))) words <- mkReg(0);
   Reg#(Index)                                rows <- mkRegU;    
   Reg#(Index)                                cols <- mkRegU;    

   Reg#(Index) n     <- mkRegU();
   Reg#(Float) r     <- mkRegU();
   Reg#(Float) theta <- mkRegU();   
  
   rule start(state == Idle && plbShim.initialized);
      let command <- serverStub.acceptRequest_PutCommand();       

      n <= unpack(truncate(command.n));	
      r <= unpack(command.r);
      theta <= unpack(command.theta);
      state <= Kickoff;	
      cycleCount <= 0;
   endrule


  rule dropCountReq(state != Idle);
     let inst <- serverStub.acceptRequest_ReadCycleCount();       
     serverStub.sendResponse_ReadCycleCount(0,?);
  endrule

  rule readCount(state == Idle);
     let inst <- serverStub.acceptRequest_ReadCycleCount();       
     serverStub.sendResponse_ReadCycleCount(1,zeroExtend(cycleCount));
  endrule
   
   rule countCycle(state == Requesting);
     cycleCount <= cycleCount + 1;
   endrule

   rule kickoff(state==Kickoff);
//      Data  fixed_r = fromMaybe(?,floatToFixedPoint(r));
//      TData fixed_theta = fromMaybe(?,floatToFixedPoint(theta));
//      $write("CartPolTester: Kickoff r ");
//      fxptWrite(9,fixed_r);
//      $write(", theta ");
//      fxptWrite(9,fixed_theta);
//      $display(", n",n);
      $display("CartPolTester: Kickoff");
      computer.setParam(r,theta,n);
      cacheWrap.reset;
      storeAddress <= fromInteger(valueof(OutputBufferBase));
      rows <= 0;
      cols <= 0;
      cycleCount <= 0;
      state <= Requesting;
   endrule
   
   rule computer_to_wrapper;
      let pos <- computer.getPos();
      $display("CartPolTest: getPos x %d y %d",pos.x,pos.y);
      let coord = Coord{x:pos.x,y:pos.y};
      cacheWrap.mmem_server.request.put(coord);
   endrule
   
   // There's no real reason to write out the data for this test bench. 
   
   rule resp_from_wrapper(cols < n && state == Requesting); 
      let res <- cacheWrap.mmem_server.response.get;
      $display("resp_from_wrapper %h", res);
       
      if(rows + 1 == n)
        begin
          rows <= 0;
          cols <= cols + 1;
        end
      else
        begin
          rows <= rows + 1;
        end

   endrule


   // Handle the last store to memory
   // Must produce proper number of Words
   rule handleLastStore(cols == n && state == Requesting);
     $display("Firing handle last store n: %d cols: %d", n, cols);
     state <= Idle;
   endrule
   
endmodule
