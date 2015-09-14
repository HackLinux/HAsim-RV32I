/*
Copyright (c) 2007 MIT

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

import MCDBRAM::*;
import FIFO::*;
import Clocks::*;
import GetPut::*;
import BRAMModel::*;

module mkMCDBRAMTestBench ();
  Clock firstClock <- exposeCurrentClock();
  Reset firstReset <- exposeCurrentReset();
  Clock secondClock <- mkAbsoluteClock(3, 3);
  Reset secondReset <- mkInitialReset(3, clocked_by secondClock);   

  MCDBRAM#(16) fifo_p1 <- mkMCDBRAM(firstClock, firstReset, secondClock, secondReset);
  MCDBRAM#(16) fifo_g1 <- mkMCDBRAM(secondClock, secondReset, firstClock, firstReset);

  Empty     secondDomain <- mkSecondDomain(fifo_g1,fifo_p1,clocked_by secondClock, reset_by secondReset);

  Reg#(Bit#(32))   dout <- mkReg(0);
  Reg#(Bit#(32))   iterOut <- mkReg(0);
  Reg#(Bit#(32))   iterIn  <- mkReg(0);
  Reg#(Bit#(32))   din  <- mkReg(0);
  Reg#(Bit#(7))    staggerOut <- mkReg(25);
  Reg#(Bit#(7))    staggerIn  <- mkReg(25);
  Reg#(Bit#(3))    ackWait <- mkReg(0);
  

  rule enq;
    iterOut <= iterOut + 1;
    if(iterOut % ((staggerOut==0)?25:zeroExtend(staggerOut)) == 0)
      begin
        staggerOut <= staggerOut + 25;
        fifo_p1.put_nack();
        dout[3:0] <= 0;
      end
    else
      begin
        $display("Testbench: CLK1: Putting: %d",dout);  
        fifo_p1.put.put(dout);
        dout <= dout + 1;
      end
  endrule

  rule deq;
    iterIn <= iterIn + 1;
    if(iterIn%((staggerIn== 0)?25:zeroExtend(staggerIn)) == 0)
      begin
        staggerIn <= staggerIn + 25;
        if((iterIn%2 == 1) && fifo_g1.get_valid() ) 
          begin
            let dinNext <- fifo_g1.get.get();
          end  
        if(ackWait > 0)  // This means we already finished the transfer
          begin
            ackWait <= 0;
            if(din[3:0] != 0)
              begin
                $display("Error: We are waiting to ack and we nacked, but din[3:0] was not 0");  
              end
            din <= din - 16;
          end
        else
          begin
            din[3:0] <= 0;
          end 
        fifo_g1.get_nack();
      end
    else
      begin
       if(ackWait > 0)
         begin
           ackWait <= ackWait - 1;
           if(ackWait - 1 == 0)
             begin
               fifo_g1.get_ack();
             end         
         end
       else
         begin
           let dinNext <- fifo_g1.get.get();
           din <= din + 1;
           if(din[3:0] + 1 == 0)
             begin
               if(iterIn[2:0] == 0)
                 begin
                   fifo_g1.get_ack();
                 end
               ackWait <= iterIn[2:0];
             end   
           if(dinNext != din)
             begin
               $display("Error: got %h, expected %h",dinNext,din); 
             end
         end
      end
  endrule
endmodule


module mkSecondDomain#(MCDBRAM#(16,64) fifo_p2, MCDBRAM#(16,64) fifo_g2) ();
  FIFO#(Bit#(32))  fifo <- mkFIFO;

  Reg#(Bit#(32))   iterOut <- mkReg(0);
  Reg#(Bit#(32))   iterIn  <- mkReg(0);
  Reg#(Bit#(4))    counter <- mkReg(0);
  Reg#(Bit#(7))    staggerOut <- mkReg(25);
  Reg#(Bit#(7))    staggerIn  <- mkReg(25);
  Reg#(Bit#(3))    ackWait <- mkReg(0);  
  
  rule enq;
    iterOut <= iterOut + 1;
    if(iterOut % ((staggerOut ==0)?25:(zeroExtend(staggerOut))) == 0)
      begin
        staggerOut <= staggerOut + 25;
        fifo_p2.put_nack();
        fifo_g2.get_nack();
        counter <= 0;
      end
    else
      begin
        counter <= counter + 1;         
        if(counter + 1 == 0)
          begin
            fifo_g2.get_ack();
          end
        let data <- fifo_g2.get.get();
        fifo_p2.put.put(data);
        $display("Testbench: CLK2: forwarding: %d",data);  
      end
  endrule


endmodule
