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

import PLBMaster::*;
import BRAMFeeder::*;
import FIFO::*;
import GetPut::*;
import ICache::*;
import MemTypes::*;
import ClientServer::*;
import Connectable::*;
import ClientServerUtils::*;
import PLBShim::*;
import Vector::*;
import LFSR::*;
import Complex::*;
import ConfigReg::*;
import mkDataCacheBlockingBramDoublePumpOverlappedSkinny::*;
import mkDataCacheBlockingBramDoublePumpOverlappedSkinnyXUPV5::*;
import mkDataCacheBlockingBramDoublePumpOverlappedGang::*;
import CacheWrapperSplitDoublePump::*;


typedef enum{
   Idle,
   Requesting,
   Draining
   } TesterState deriving (Bits,Eq);

module mkCacheTesterCore#(Feeder feeder, PLBMaster plbMaster) (Empty);
   
   Reg#(TesterState)                     state <- mkReg(Idle);
   Reg#(Bit#(SizeOf#(Coord)))         readAddr <- mkReg(0);
   Reg#(Bit#(SizeOf#(Coord)))        readLimit <- mkReg(~0);// Cover coord space
   Reg#(Bit#(10))                       xcoord <- mkReg(0);             
   Reg#(Bit#(10))                       ycoord <- mkReg(0);            
   Reg#(Bool)                            error <- mkReg(False);
   Reg#(Bit#(TAdd#(1,TLog#(NumTests))))   test <- mkConfigReg(0);

   Bit#(10) max_n = 10'd1023;   
   
   Vector#(2,ICache#(DataReq,DoublePumpDataResp)) caches;
   caches[0] <- mkDataCacheBlockingBramDoublePumpOverlappedSkinny(0);
   caches[1] <- mkDataCacheBlockingBramDoublePumpOverlappedSkinny(1);


   FIFO#(Bit#(MainMemTagSz))          main_tags <- mkFIFO();   
   FIFO#(Tuple2#(Bool,Coord))         mem_req_bypass <- mkSizedFIFO(32);
   
   PLBShim#(MainMemReq,MainMemResp) plbShim <- mkPLBShim(plbMaster);

   LFSR#(Bit#(32)) randCoord <- mkLFSR_32;

   CacheWrapper cacheWrap <- mkCacheWrapperSplitDoublePump(caches); 
   mkConnection(cacheWrap.mmem_client, plbShim.mem_server);   
   
   // get the test number
   rule startA(state==Idle);
      PPCMessage cmd <- feeder.ppcMessageOutput.get();
      if(cmd > fromInteger(valueof(NumTests))) 
        begin
          test <= 0;
        end
      else
         begin
	    // TODO: this will actually happen after a test has finished
            test <= truncate(cmd);
	    cacheWrap.reset();
         end
      $display("feederCmd test %h",cmd);
      state  <= Requesting;
      readAddr <= 0;
      xcoord <= 0;
      ycoord <= 0;
      randCoord.seed(~0);
   endrule
   
   rule end_test(state == Requesting && test == 0);
     feeder.ppcMessageInput.put((error)?-1:1);
     state <= Idle;
   endrule

   // Test 1 as simple walk.
   rule req_to_cache(state==Requesting && test == 3);
      Coord reqCoord = Coord{y:ycoord,x:xcoord};
      if(ycoord == max_n-1 && xcoord == max_n-1)
        begin
          state <= Idle;
          mem_req_bypass.enq(tuple2(True,reqCoord));       
          cacheWrap.mmem_server.request.put(reqCoord);
        end
      else 
        begin
          if(xcoord + 1 == max_n) 
            begin
              xcoord <= 0;
              ycoord <= ycoord + 1;
            end
          else
            begin
              xcoord <= xcoord + 1;
            end 
          cacheWrap.mmem_server.request.put(reqCoord);
          mem_req_bypass.enq(tuple2(False,reqCoord));       
	   $display("mkCacheTesterCore6: req_to_cache %h %h",reqCoord.x,reqCoord.y);
        end        
   endrule


   //walk horizontal edge 
   rule req_to_cache2(state==Requesting && test == 2);
      Coord reqCoord = Coord{y:ycoord,x:xcoord};
      if(ycoord == max_n - 1 && xcoord == max_n - 1)
        begin
          state <= Idle;
          mem_req_bypass.enq(tuple2(True,reqCoord));       
          cacheWrap.mmem_server.request.put(reqCoord);
        end
      else 
        begin          
         if(xcoord + 1 == max_n) 
            begin
              xcoord <= 0;
              ycoord <= max_n - 1;
            end 
          else
            begin
              xcoord <= xcoord + 1;
            end 
           cacheWrap.mmem_server.request.put(reqCoord);
           mem_req_bypass.enq(tuple2(False,reqCoord));       
          $display("mkCacheTesterCore6: req_to_cache %h %h",reqCoord.x,reqCoord.y);
        end
   endrule

   //walk vertical edge 
   rule req_to_cache3(state==Requesting && test == 1);
      Coord reqCoord = Coord{y:ycoord,x:xcoord};
      if(ycoord == max_n - 1 && xcoord == max_n - 1)
        begin
          state <= Idle;
          cacheWrap.mmem_server.request.put(reqCoord);
          mem_req_bypass.enq(tuple2(True,reqCoord));       
        end
      else 
        begin
          if(ycoord + 1 == max_n) 
            begin
              xcoord <= max_n - 1;
              ycoord <= 0;
            end 
          else 
            begin
              ycoord <= ycoord + 1;
            end
           cacheWrap.mmem_server.request.put(reqCoord);
           mem_req_bypass.enq(tuple2(False,reqCoord));       
	   $display("mkCacheTesterCore6: req_to_cache %h %h",reqCoord.x,reqCoord.y);
        end
   endrule

   // Walk up, rather 
   // hit the diagonal
   rule req_to_cache4(state==Requesting && test == 4);
      Coord reqCoord = Coord{y:ycoord,x:xcoord};
      if(ycoord == max_n - 1 && xcoord == max_n - 1)
        begin
          state <= Idle;
          cacheWrap.mmem_server.request.put(reqCoord);
          mem_req_bypass.enq(tuple2(True,reqCoord));       
        end
      else 
        begin
          xcoord <= xcoord + 1;
          ycoord <= ycoord + 1;
         
          cacheWrap.mmem_server.request.put(reqCoord);
          mem_req_bypass.enq(tuple2(False,reqCoord));       
          $display("mkCacheTesterCore6: req_to_cache %h %h",reqCoord.x,reqCoord.y);
        end
   endrule


   // Random access test
   rule req_to_cache5(state==Requesting && test == 6);
      Coord reqCoord = unpack(truncate(randCoord.value));
      randCoord.next;
      if(reqCoord.x < max_n && reqCoord.y < max_n)
        begin
          if(ycoord == max_n - 1 && xcoord == max_n - 1)
            begin
              state <= Idle;
              cacheWrap.mmem_server.request.put(reqCoord);
              mem_req_bypass.enq(tuple2(True,reqCoord));       
            end
          else 
            begin
              if(ycoord + 1 == max_n) 
                begin
                  ycoord <= 0;
                  xcoord <= xcoord + 1;
                end 
            else
              begin
                ycoord <= ycoord + 1;
              end 

            cacheWrap.mmem_server.request.put(reqCoord);
            mem_req_bypass.enq(tuple2(False,reqCoord));       
            $display("mkCacheTesterCore5: req_to_cache %h %h",reqCoord.x,reqCoord.y);
          end
        end
   endrule



   // Walk up, rather 
   rule req_to_cache6(state==Requesting && test == 5);
      Coord reqCoord = Coord{y:ycoord,x:xcoord};
      if(ycoord == max_n - 1 && xcoord == max_n - 1)
        begin
          state <= Idle;
          cacheWrap.mmem_server.request.put(reqCoord);
          mem_req_bypass.enq(tuple2(True,reqCoord));       
        end
      else 
        begin
          if(ycoord + 1 == max_n) 
            begin
              ycoord <= 0;
              xcoord <= xcoord + 1;
            end 
          else
            begin
              ycoord <= ycoord + 1;
            end 

          cacheWrap.mmem_server.request.put(reqCoord);
          mem_req_bypass.enq(tuple2(False,reqCoord));       
          $display("mkCacheTesterCore6: req_to_cache %h %h",reqCoord.x,reqCoord.y);
        end
   endrule


   // may need to change this for 64 bit words
   rule resp_from_cache(test != 0);
      let result <- cacheWrap.mmem_server.response.get();
      mem_req_bypass.deq();
      match {.last, .base} = mem_req_bypass.first();
      let sz = 1024;
      
      // we must now determine the average of the values.. 
      Bit#(32) lowerLeft   = (zeroExtend(base.y)*sz)+zeroExtend(base.x);
      Bit#(32) upperLeft   = (zeroExtend(base.y+1)*sz)+zeroExtend(base.x);
      Bit#(32) lowerRight  = (zeroExtend(base.y)*sz)+zeroExtend(base.x+1);
      Bit#(32) upperRight  = (zeroExtend(base.y+1)*sz)+zeroExtend(base.x+1);

      Complex#(Int#(16)) llC = Complex{rel: unpack(lowerLeft[31:16]), img: unpack(lowerLeft[15:0])};
      Complex#(Int#(16)) ulC = Complex{rel: unpack(upperLeft[31:16]), img: unpack(upperLeft[15:0])};
      Complex#(Int#(16)) lrC = Complex{rel: unpack(lowerRight[31:16]), img: unpack(lowerRight[15:0])};
      Complex#(Int#(16)) urC = Complex{rel: unpack(upperRight[31:16]), img: unpack(upperRight[15:0])};

      $display("Added in: %h %h", pack(upperLeft), pack(upperRight));
      $display("          %h %h", pack(lowerLeft), pack(lowerRight));
      
      Complex#(Int#(18)) ex_18 = cmplx((extend(llC.rel)+extend(ulC.rel)+extend(lrC.rel)+extend(urC.rel))/4,
					    (extend(llC.img)+extend(ulC.img)+extend(lrC.img)+extend(urC.img))/4);
      Complex#(Int#(16)) ex_16 = cmplx(truncate(ex_18.rel), truncate(ex_18.img));
      let expected = truncate(pack(ex_16));

      if(result != pack(expected))
        begin 
          $display("Error: expected: %h, got %h",  expected, result);
          $display("FAIL");
          $finish;
          error <= True; // Needed for FPGA
        end
      $display("resp_from_cache: %h expected: %h", result, expected);
      if(last) 
         begin 
            feeder.ppcMessageInput.put((error)?-1:1);
        end
   endrule
   
endmodule
