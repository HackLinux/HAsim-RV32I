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

// Global Imports
import GetPut::*;
import FIFO::*;
import RegFile::*;
import BRAMInitiatorWires::*;
import RegFile::*;
import FIFOF::*;

import Types::*;
import Interfaces::*;
import Parameters::*;
import DebugFlags::*;
 
import PLBMasterWires::*;
 
typedef enum {   
  Idle,
  Requesting,
  Data,
  RearbitrateDeadCycle
} State 
    deriving(Bits, Eq);
  

(* synthesize *)
module mkPLBMaster (PLBMaster);
  Clock plbClock <- exposeCurrentClock();
  Reset plbReset <- exposeCurrentReset();
  // state for the actual magic memory hardware
  FIFO#(ComplexWord)       wordInfifo <- mkFIFO();
  FIFO#(ComplexWord)       wordOutfifo <- mkFIFO();
  FIFO#(PLBMasterCommand)  plbMasterCommandInfifo <- mkFIFO(); 

  
  // Output buffer
//  RegFile#(Bit#(5),Bit#(32))                           storeBuffer <- mkRegFileFull();   
  RegFile#(Bit#(4),Bit#(32))                           storeBuffer_odd  <- mkRegFileFull();   
  RegFile#(Bit#(4),Bit#(32))                           storeBuffer_even <- mkRegFileFull();  
  
  // Input buffer
  RegFile#(Bit#(5),Bit#(32))                           loadBuffer <- mkRegFileFull();   


  Reg#(Bit#(LogBlockSize))                  rowCounterLoad  <- mkReg(0);
  Reg#(Bit#(LogBlockSize))                  rowCounterStore <- mkReg(0);

  Reg#(Bit#(LogRowSize))                    rowOffset <- mkReg(0);  // stored in terms of words
  
  
  Reg#(Bit#(TLog#(TDiv#(BlockSize,
                        BurstSize))))         burstAddrOffset <- mkReg(0);
  Reg#(Bit#(24))                              rowAddrOffsetLoad <- mkReg(0);
  Reg#(Bit#(24))                              rowAddrOffsetStore <- mkReg(0);  

  Reg#(Bool)                                doingLoad <- mkReg(False);
  Reg#(Bool)                                doingStore <- mkReg(False);

  BlockAddr addressOffset                   = zeroExtend({(doingStore)?rowAddrOffsetStore:rowAddrOffsetLoad,burstAddrOffset,4'b0});
  Bit#(TLog#(TDiv#(BlockSize,
                        BurstSize)))      nextBurstAddrOffset = burstAddrOffset + 1;
  Bit#(24)  nextRowAddrOffsetLoad               = rowAddrOffsetLoad + zeroExtend(rowOffset>>valueof(LogBlockSize));//ndave: Shifted by
  Bit#(24)  nextRowAddrOffsetStore              = rowAddrOffsetStore + zeroExtend(rowOffset>>valueof(LogBlockSize));

//  Reg#(BlockAddr)                           addressOffset <- mkReg(0);       
//  Reg#(BlockAddr)                           addressOffsetRow  <- mkReg(0); // could be optimized away, if we need it


  Reg#(State)                               state <- mkRegU;  
  Reg#(Bit#(1))                             request <- mkReg(0);
  Reg#(Bit#(1))                             rnw <- mkReg(0);
  Reg#(Bit#(1))                             buslock <- mkReg(0);

  Reg#(Bit#(TLog#(BurstSize)))              loadDataCount <- mkReg(0);
  Reg#(Bit#(TLog#(BurstSize)))              storeDataCount <-mkReg(0);// If you change this examine mWrDBus_o
  Reg#(Bit#(TAdd#(1,TLog#(BurstSize))))      loadDataCount_plus2 <- mkReg(2);
  Reg#(Bit#(TAdd#(1,TLog#(BurstSize))))      storeDataCount_plus2 <-mkReg(2);  
  
  Reg#(Bool)                                doAckinIdle <- mkReg(False);

  Reg#(Bit#(1))                             rdBurst <- mkReg(0);
  Reg#(Bit#(1))                             wrBurst <- mkReg(0);
  Reg#(Bit#(TLog#(TDiv#(BlockSize,
                        BurstSize))))       burstCounter <- mkReg(0); // Counts the bursts the per row 

  Reg#(Bit#(5))                             storeBufferWritePointer <- mkReg(0);
  FIFOF#(Bit#(0))                           storeValid <- mkUGFIFOF;//XXX: This could be bad
  Reg#(Bit#(5))                             loadBufferReadPointer <- mkReg(0);
  FIFOF#(Bit#(0))                           loadValid <- mkUGFIFOF;//XXX: This could be bad  


  Wire#(Bool) address_switching <- mkDWire(False);

  // Input wires  
  Wire#(Bit#(1)) mRst <- mkBypassWire();
  Wire#(Bit#(1)) mAddrAck <- mkBypassWire();	
  Wire#(Bit#(1)) mBusy <- mkBypassWire(); 	
  Wire#(Bit#(1)) mErr <- mkBypassWire();		
  Wire#(Bit#(1)) mRdBTerm <- mkBypassWire(); 	
  Wire#(Bit#(1)) mRdDAck <- mkBypassWire();
  Wire#(Bit#(64))mRdDBus <- mkBypassWire(); 
  Wire#(Bit#(3)) mRdWdAddr <- mkBypassWire(); 	
  Wire#(Bit#(1)) mRearbitrate <- mkBypassWire(); 
  Wire#(Bit#(1)) mWrBTerm <- mkBypassWire(); 	
  Wire#(Bit#(1)) mWrDAck <- mkBypassWire(); 	
  Wire#(Bit#(1)) mSSize <- mkBypassWire(); 	
  Wire#(Bit#(1)) sMErr <- mkBypassWire(); // on a read, during the data ack		
  Wire#(Bit#(1)) sMBusy <- mkBypassWire();

  //  Outputs


  Bit#(PLBAddrSize) mABus_o = {addressOffset,2'b00}; // Our address Address Bus, we extend to compensate for word 

  
  Bit#(5) sbuf_addr = {truncate({rowCounterStore,burstCounter}),storeDataCount};
  
  Bit#(32)mWrDBus_val_odd  =  storeBuffer_odd.sub(sbuf_addr[4:1]);
  Bit#(32)mWrDBus_val_even = storeBuffer_even.sub(sbuf_addr[4:1]); // write data bus 
                                                     // alignment, which we gaurantee.
  Bit#(64)mWrDBus_o   = {mWrDBus_val_even, mWrDBus_val_odd}; //{odd,even}
  Bit#(1) mRequest_o  = request & ~mRst; // Request
  Bit#(1) mBusLock_o  = buslock & ~mRst; // Bus lock
  Bit#(1) mRdBurst_o  = rdBurst & ~mRst; // read burst 
  Bit#(1) mWrBurst_o  = wrBurst & ~mRst; // write burst
  Bit#(1) mRNW_o      = rnw; // Read Not Write
  Bit#(1) mAbort_o    = 1'b0; // Abort
  Bit#(2) mPriority_o = 2'b11;// priority indicator
  Bit#(1) mCompress_o = 1'b0;// compressed transfer
  Bit#(1) mGuarded_o  = 1'b0;// guarded transfer
  Bit#(1) mOrdered_o  = 1'b0;// synchronize transfer
  Bit#(1) mLockErr_o  = 1'b0;// lock erro
  Bit#(4) mSize_o     = 4'b1010; // Burst word transfer - see PLB p.24
  Bit#(3) mType_o     = 3'b000; // Memory Transfer
  Bit#(8) mBE_o       = 8'b00001111; // 16 word burst
  Bit#(2) mMSize_o    = 2'b00;


  // precompute the next address offset.  Sometimes
  
  //  BlockAddr addressOffsetRowNext = addressOffsetRow + zeroExtend(rowOffset);
  //  BlockAddr addressOffsetNext = addressOffset + 16;
  
  PLBMasterCommand cmd_in_first = plbMasterCommandInfifo.first();

  let newloadDataCount  =  loadDataCount + 1;
  let newstoreDataCount = storeDataCount + 1;
  let newloadDataCount_plus2  =  loadDataCount_plus2 + 1;
  let newstoreDataCount_plus2 = storeDataCount_plus2 + 1;
    
  rule writeStoreData(storeValid.notFull());
    storeBufferWritePointer <= storeBufferWritePointer + 1;
     
    if (storeBufferWritePointer[0] == 1)
      storeBuffer_odd.upd(storeBufferWritePointer[4:1], pack(wordInfifo.first()));
    else
      storeBuffer_even.upd(storeBufferWritePointer[4:1], pack(wordInfifo.first()));
    
    wordInfifo.deq();
    if(truncate(storeBufferWritePointer + 1) == 4'b0000)
      begin
        $display("Store Data finished a flight");
        storeValid.enq(0);
      end
  endrule
     
  rule readLoadData(loadValid.notEmpty());
    loadBufferReadPointer <= loadBufferReadPointer + 1;
    wordOutfifo.enq(unpack(loadBuffer.sub(loadBufferReadPointer)));
    if(truncate(loadBufferReadPointer + 1) == 4'b0000)
      begin
        $display("Load Data finished a flight");
        loadValid.deq();
      end
  endrule


  //rule rowSize(cmd_in_first matches tagged RowSize .rs);   
  //  debug(plbMasterDebug, $display("PLBMaster: processing RowSize command: %d", rs));
  //  rowOffset <= (1<<rs);
  //  plbMasterCommandInfifo.deq();
  //endrule
  
  rule startPage(!doingLoad && !doingStore);
        $display("Start Page");
        plbMasterCommandInfifo.deq();
    	case (cmd_in_first) matches
          tagged RowSize .rs:
            begin
              debug(plbMasterDebug, $display("PLBMaster: processing RowSize command: %d", rs));
              rowOffset <= (1<<rs);
            end 
	  tagged LoadPage .ba:
            begin   
              $display("Load Page");
              burstAddrOffset <= truncate(ba>>4); //bottom 4 bits are zero
	      rowAddrOffsetLoad   <= truncate(ba>>(4+valueof(TLog#(TDiv#(BlockSize,BurstSize))))); // this is the log 
                                                                                                   // size of burst addr
	      if (ba[3:0] != 0)
		$display("ERROR:Address not 64-byte aligned");
	      //addressOffset <= ba;
	      //addressOffsetRow <= ba;
	      state <= Idle;
              doingLoad <= True;
	    end 
	  tagged StorePage .ba: 
	    begin  
              $display("Store Page");
              burstAddrOffset <= truncate(ba>>4); //bottom 4 bits are zero
	      rowAddrOffsetStore <= truncate(ba>>(4+valueof(TLog#(TDiv#(BlockSize,BurstSize))))); // this is the log 
                                                                                                  // size of burst addr
	      if (ba[3:0] != 0)
		$display("ERROR:Address not 64-byte aligned");
	      //addressOffset <= ba;
	      //addressOffsetRow <= ba;
	      state <= Idle;
	      doingStore <= True;
	    end
	endcase
  endrule


//  rule doAck(doAckinIdle); // the first two make it idle
//  $display("Acking in Idle");
//    plbMasterCommandInfifo.deq();  
//    doAckinIdle <= False;
//    doingLoad <= False;
//    doingStore <= False;
//  endrule

  
  rule loadPage_Idle(doingLoad && !doingStore && state == Idle);
    // We should not initiate a transfer if the wordOutfifo is not valid
    //$display("loadPage_Idle");  
    if(loadValid.notFull())// Check for a spot.       
      begin  
	request <= 1'b1;
	state <= Requesting;
      end
    else
      begin
	request <= 1'b0;  // Not Sure this is needed
      end 
    buslock <= 1'b1;
    rnw <= 1'b1; // We're reading
  endrule
  
  rule loadPage_Requesting(doingLoad && !doingStore && state == Requesting);
    // We've just requested the bus and are waiting for an ack
    //$display("loadPage_Requesting");
    if(mAddrAck == 1 )
      begin
	// Check for error conditions  
	if(mRearbitrate == 1) 
	  begin
	    // Got terminated by the bus
	    $display("Terminated by BUS @ %d",$time);
	    state <= RearbitrateDeadCycle;
	    rdBurst <= 1'b0; // if we're rearbing this should be off. It may be off anyway? 
	    request <= 1'b0;
	    buslock <= 1'b0; 
	  end 
        else
	  begin
	    //Whew! didn't die yet.. wait for acks to come back
	    state <= Data;
	    // Not permissible to assert burst until after addrAck p. 35
	    rdBurst <= 1'b1;
	    // Set down request, as we are not request pipelining
	    request <= 1'b0;
	  end
      end
  endrule    

  rule loadPage_Data(doingLoad && !doingStore && state == Data);
    if(((mRdBTerm == 1) && (loadDataCount_plus2 < (fromInteger(valueof(BurstSize))) )) || (mErr == 1))
      begin
	// We got terminated / Errored 
	rdBurst <= 1'b0;
	loadDataCount <= 0;
 	loadDataCount_plus2 <= 2;   
	// Set up request, as we are not requesting
	request <= 1'b1;
	state <= Requesting;             
      end
    else if(mRdDAck == 1)
      begin
	loadDataCount <= newloadDataCount;
        loadDataCount_plus2 <= newloadDataCount_plus2;
        loadBuffer.upd({truncate({rowCounterLoad,burstCounter}),loadDataCount},
                       (loadDataCount[0] == 0)?mRdDBus[31:0]:mRdDBus[63:32]);                   
	if(newloadDataCount == 0)
	  begin
	    //We're now done reading... what should we do?
	    burstCounter <= burstCounter + 1;
            loadValid.enq(0);  // This signifies that the data is valid Nirav could probably remove this
	    if(burstCounter == maxBound)
	      begin
		//done bursting this row  
                burstAddrOffset <= nextBurstAddrOffset;
		rowAddrOffsetLoad   <= nextRowAddrOffsetLoad;
		//addressOffset <= addressOffsetRowNext;
		//addressOffsetRow <= addressOffsetRowNext;
		rowCounterLoad <= rowCounterLoad + 1;
		if(rowCounterLoad + 1 == 0)
		  begin
		    // Data transfer complete
                    doingLoad <= False;
                    buslock <= 1'b0;
		    //doAckinIdle <= True;
		  end
		else 
		  begin
		    // Set up request, as we are not requesting
		    state <= Idle;
		  end
	      end
	    else
	      begin 
		// Set up request, as we are not requesting
		state <= Idle;
                burstAddrOffset <= nextBurstAddrOffset;
		//addressOffset <= addressOffsetNext;
	      end
	  end
	else if(newloadDataCount == maxBound) // YYY: ndave used to ~0
	  begin
	    // Last read is upcoming.  Need to set down the 
	    // rdBurst signal.
	    rdBurst <= 1'b0;
	  end
      end
  endrule

  rule loadPage_RearbitrateDeadCycle(doingLoad && !doingStore && state == RearbitrateDeadCycle);
    // An idle cycle fulfills the two cycle delay required by p.19 2.2.8
    state <= Idle; 
  endrule

  rule storePage_Idle(doingStore && !doingLoad && state == Idle);
    if(storeValid.notEmpty())
      begin
	request <= 1'b1;
	state <= Requesting;
      end
    else
      begin
	request <= 1'b0;
      end
    buslock <= 1'b1;
    wrBurst <= 1'b1; // Write burst is asserted with the write request
    rnw <= 1'b0; // We're writing
  endrule
  
  rule storePage_Requesting(doingStore && !doingLoad && state == Requesting);
    // We've just requested the bus and are waiting for an ack
    if(mAddrAck == 1 )
      begin
	// Check for error conditions
	if(mRearbitrate == 1)
	  begin
	    // Got terminated by the bus
	    state <= RearbitrateDeadCycle;
	    wrBurst <= 1'b0;
	    request <= 1'b0;
	    buslock <= 1'b0;
	  end
        else
	  begin
	    // Set down request, as we are not request pipelining
	    request <= 1'b0;
	    // We can be WrDAck'ed at this time p.29 or WrBTerm p.30 
	    if(mWrBTerm == 1)
	      begin  
		wrBurst <= 1'b0;
		state <= Idle; 
	      end
            else if(mWrDAck == 1)
	      begin
		storeDataCount <= newstoreDataCount;
    		storeDataCount_plus2 <= newstoreDataCount_plus2;
		state <= Data;
	      end
            else
	      begin
		state <= Data;
	      end                             
	  end
      end
  endrule

  rule storePage_Data(doingStore && !doingLoad && state == Data);
    if((mWrBTerm == 1) && (storeDataCount_plus2 < (fromInteger(valueof(BurstSize)))) || (mErr == 1))
      begin
	// We got terminated / Errored 
	wrBurst <= 1'b0;
	storeDataCount <= 0;
        storeDataCount_plus2 <= 2;
	// Set up request, as we are not requesting
	request <= 1'b0;
        buslock <= 1'b0;
	state <= Idle; // Can't burst for a cycle p. 30             
      end
    else if(mWrDAck == 1)
      begin
	storeDataCount <= newstoreDataCount;                   
	storeDataCount_plus2 <= newstoreDataCount_plus2;
	if(newstoreDataCount == 0)
	  begin
	    //We're now done reading... what should we do?
	    burstCounter <= burstCounter + 1;
	    if(burstCounter == maxBound)
	      begin
		//done bursting this row  
                burstAddrOffset <= nextBurstAddrOffset;
                rowAddrOffsetStore <= nextRowAddrOffsetStore; 
		//addressOffset <= addressOffsetRowNext;
		//addressOffsetRow <= addressOffsetRowNext;
		rowCounterStore <= rowCounterStore + 1;
		if(rowCounterStore + 1 == 0)
		  begin
		    // Data transfer complete
                    doingStore <= False;
                    buslock <= 1'b0;
                    //doAckinIdle <= True;
		  end
		else 
		  begin
		    // Set up request, as we are not requesting
		    state <= Idle;
		  end
	      end
	    else
	      begin 
		// Set up request, as we are not requesting
		state <= Idle;
		burstAddrOffset <= nextBurstAddrOffset;
		//addressOffset <= addressOffsetNext;
	      end
            storeValid.deq();
	  end
	else if(newstoreDataCount == maxBound) //YYY: used to be ~0
	  begin
	    // Last read is upcoming.  Need to set down the 
	    // wrBurst signal.
	    wrBurst <= 1'b0;
	  end
      end
  endrule

  rule storePage_RearbitrateDeadCycle(doingStore && !doingLoad && state == RearbitrateDeadCycle);
    // An idle cycle fulfills the two cycle delay required by p.19 2.2.8
    state <= Idle; 
  endrule
  


  interface Put wordInput = fifoToPut(wordInfifo);

  interface Get wordOutput = fifoToGet(wordOutfifo);

  interface Put plbMasterCommandInput = fifoToPut(plbMasterCommandInfifo);


  interface PLBMasterWires  plbMasterWires;
 
    method Bit#(PLBAddrSize) mABus();     // Address Bus
      return mABus_o;   
    endmethod
    method Bit#(8)           mBE();       // Byte Enable
      return mBE_o;    
    endmethod
	
    method Bit#(1)           mRNW();      // Read Not Write
      return mRNW_o;    
    endmethod

    method Bit#(1)           mAbort();    // Abort
      return mAbort_o;    
    endmethod

    method Bit#(1)           mBusLock();  // Bus lock
      return mBusLock_o;    
    endmethod

    method Bit#(1)           mCompress(); // compressed transfer
      return mCompress_o;    
    endmethod

    method Bit#(1)           mGuarded();  // guarded transfer
      return mGuarded_o;    
    endmethod

    method Bit#(1)           mLockErr();  // lock error
      return mLockErr_o;    
    endmethod

    method Bit#(2)           mMSize();    // data bus width?
      return mMSize_o;    
    endmethod

    method Bit#(1)           mOrdered();  // synchronize transfer
      return mOrdered_o;    
    endmethod

    method Bit#(2)           mPriority(); // priority indicator
      return mPriority_o;    
    endmethod

    method Bit#(1)           mRdBurst();  // read burst
      return mRdBurst_o;    
    endmethod

    method Bit#(1)           mRequest();  // bus request
      return mRequest_o;    
    endmethod
	
    method Bit#(4)           mSize();     // transfer size 
      return mSize_o;    
    endmethod

    method Bit#(3)           mType();     // transfer type (dma) 
      return mType_o;    
    endmethod

    method Bit#(1)           mWrBurst();  // write burst
      return mWrBurst_o;    
    endmethod
	
    method Bit#(64)          mWrDBus();   // write data bus
      return mWrDBus_o;    
    endmethod

    method Action plbIN(
      Bit#(1) mRst_in,            // PLB reset
      Bit#(1) mAddrAck_in,	       // Addr Ack
      Bit#(1) mBusy_in,           // Master Busy
      Bit#(1) mErr_in,            // Slave Error
      Bit#(1) mRdBTerm_in,	       // Read burst terminate signal
      Bit#(1) mRdDAck_in,	       // Read data ack
      Bit#(64)mRdDBus_in,	       // Read data bus
      Bit#(3) mRdWdAddr_in,       // Read word address
      Bit#(1) mRearbitrate_in,    // Rearbitrate
      Bit#(1) mWrBTerm_in,	       // Write burst terminate
      Bit#(1) mWrDAck_in,	       // Write data ack
      Bit#(1) mSSize_in, 	       // Slave bus size
      Bit#(1) sMErr_in,	       // Slave error
      Bit#(1) sMBusy_in);	       // Slave busy       
      mRst <= mRst_in;       
      mAddrAck <= mAddrAck_in;	
      mBusy <= mBusy_in;		
      mErr <= mErr_in;		
      mRdBTerm <= mRdBTerm_in;	
      mRdDAck <= mRdDAck_in;	
      mRdDBus <= mRdDBus_in;	
      mRdWdAddr <= mRdWdAddr_in;	
      mRearbitrate <= mRearbitrate_in;
      mWrBTerm <= mWrBTerm_in;	
      mWrDAck <= mWrDAck_in;	
      mSSize <= mSSize_in; 	
      sMErr <= sMErr_in;		
      sMBusy <= sMBusy_in;	
    endmethod
   endinterface
endmodule
