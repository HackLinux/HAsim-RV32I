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

Author: Michael Pellauer
*/

// Global Imports
import FIFO::*;
import Vector::*;
import GetPut::*;
import FIFOLevel::*;

// Local Imports
`include "asim/provides/librl_bsv.bsh"
`include "awb/provides/soft_connections.bsh"
`include "awb/provides/soft_services.bsh"
`include "awb/provides/soft_services_lib.bsh"
`include "awb/provides/soft_services_deps.bsh"
`include "awb/provides/mmm_common.bsh"
`include "awb/provides/mmm_functional_unit.bsh"
`include "awb/provides/mmm_functional_unit_network.bsh"
`include "awb/provides/mmm_memory_unit.bsh"
`include "awb/provides/mmm_memory_switch.bsh"
`include "awb/rrr/remote_server_stub_MMMCONTROLRRR.bsh"


// Local imports


//Simple controller, FIFOs everywhere.
//Works, but room for improvement.

typedef enum {
    Idle,
    Running,
    Reporting
} ControlState deriving (Bits, Eq);

module [CONNECTED_MODULE] mkSimpleController (Controller);

  Vector#(FunctionalUnitNumber, FIFO#(FunctionalUnitCommand)) fuQs <- replicateM(mkFIFO);
  Reg#(ControlState) state <- mkReg(Idle);
  Reg#(Bit#(40)) cycleCount <- mkReg(0);

  FIFO#(PLBMasterCommand)     plbQ <- mkFIFO();
  FIFO#(MemorySwitchCommand)  switchQ <- mkFIFO();
  FIFO#(FUNetworkCommand)     funetQ <- mkFIFO();

  ServerStub_MMMCONTROLRRR serverStub <- mkServerStub_MMMCONTROLRRR();
  FIFOCountIfc#(Instruction,4096) instQ <- mkSizedBRAMFIFOCount();
  CONNECTION_RECV#(Bit#(1)) syncQ <- mkConnectionRecv("Sync");  
  FIFO#(Bit#(1)) syncBuffer <- mkSizedFIFO(128);

  rule syncTokens;
      syncBuffer.enq(?);
      syncQ.deq();
  endrule

  //Note: The CAN_FIRE of this will be bad with aggressive-conditions

  rule enqInst;
     let insRaw <- serverStub.acceptRequest_PutInstruction();
     serverStub.sendResponse_PutInstruction(zeroExtend(pack(instQ.count)));
     Instruction ins = unpack(truncate(insRaw));
     instQ.enq(ins);
  endrule

  rule startExec(state == Idle);
     let insRaw <- serverStub.acceptRequest_Execute();
     state <= Running;
  endrule

  rule reportResult(state == Reporting);
     serverStub.sendResponse_Execute(zeroExtend(cycleCount));
     cycleCount <= 0;
     state <= Idle;
  endrule

  rule countCycles (state == Running);
     cycleCount <= cycleCount + 1;
  endrule

  Bool isSync = case (instQ.first()) matches
      tagged SyncInstruction: 
        return True;
      default:
        return False;
    endcase;

  rule decode (state == Running && !isSync);
    let ins = instQ.first();
    instQ.deq();
    // Some debug
    case (ins) matches
      tagged ArithmeticInstruction .i:   //{.fus, .op}
         debug(controllerDebug, $display("Controller: processing Arithmetic instruction: op: %s fus: %b",
                                showFunctionalUnitOp(i.op),
                                i.fus));
      tagged LoadInstruction .i:         //{.fus, .regName, .addr}
         debug(controllerDebug, $display("Controller: processing Load instruction: fus: %b reg: %s addr: %h",
                                i.fus,
                                showReg(i.regName),
                                i.addr));

      tagged StoreInstruction .i:        //{.fu, .regName, .addr}
         if(valueof(FunctionalUnitNumber) > 1)
           begin
             debug(controllerDebug, $display("Controller: processing Store instruction: fu: %d reg: %s addr: %h",
                                  i.fu,
                                  showReg(i.regName),
                                  i.addr));
           end
         else
           begin
             debug(controllerDebug, $display("Controller: processing Store instruction: fu: %d reg: %s addr: %h",
                                  0,
                                  showReg(i.regName),
                                  i.addr));
           end   

      tagged ForwardInstruction .i:      //{.fuSrc, .regSrc, .fuDests, .regDest}
         if(valueof(FunctionalUnitNumber) > 1)
           begin
         debug(controllerDebug, $display("Controller: processing Forward instruction: fuSrc: %d regSrc: %s fuDests: %b regDest: %s",
                                i.fuSrc,
                                showReg(i.regSrc),
                                i.fuDests,
                                showReg(i.regDest)));
           end
         else
           begin
          debug(controllerDebug, $display("Controller: processing Forward instruction: fuSrc: %d regSrc: %s fuDests: %b regDest: %s",
                                0,
                                showReg(i.regSrc),
                                i.fuDests,
                                showReg(i.regDest)));
           end
      tagged SetRowSizeInstruction .sz:
         debug(controllerDebug, $display("Controller: processing RowSize instruction: log size: %d",
                                         sz));
      tagged FinishInstruction:
         debug(controllerDebug, $display("Controller: processing Finish Executing"));
      default:
         $display("Controller: ERROR, illegal instruction: %h", ins);
    endcase


     //Handle FU commands
    
    let destFUs = case (ins) matches
      tagged ArithmeticInstruction .i:   //{.fus, .op}
        return i.fus;
      tagged LoadInstruction .i:         //{.fus, .regName, .addr}
        return i.fus;
      tagged StoreInstruction .i:        //{.fu, .regName, .addr}
        return oneHot(i.fu);
      tagged ForwardInstruction .i:      //{.fuSrc, .regSrc, .fuDests, .regDest}
        return i.fuDests; //The src is handled separately
      tagged SetRowSizeInstruction .sz:
        return 0;
    endcase;
  
    let fu_cmd =  case (ins) matches
      tagged ArithmeticInstruction .i:   //{.fus, .op}
        return tagged Op i.op;
      tagged LoadInstruction .i:         //{.fus, .regName, .addr}
        return tagged Load i.regName;
      tagged StoreInstruction .i:        //{.fus, .regName, .addr}
        return tagged Store i.regName;
      tagged ForwardInstruction .i:      //{.fuSrc, .regSrc, .fuDests, .regDest}
        return tagged ForwardDest i.regDest;
      tagged SetRowSizeInstruction .sz:
        return ?;
    endcase;
    
    Bool isFwd =  case (ins) matches
      tagged ForwardInstruction .i: 
        return True;
      default:
        return False;
    endcase;
    
    let fwd_src = case (ins) matches
      tagged ForwardInstruction .i: 
        return i.fuSrc;
      default:
        return ?;
    endcase;
    
    let fwd_cmd = case (ins) matches
      tagged ForwardInstruction .i: 
        return ForwardSrc (i.regSrc);
      default:
        return ?;
    endcase;
    
    //Handle Switch Commands
    
    Bool isSwitch = case (ins) matches
      tagged LoadInstruction .i: 
        return True;
      tagged StoreInstruction .i:
        return True;
      default:
        return False;
    endcase;
    
    MemorySwitchCommand sw_cmd = case (ins) matches
      tagged LoadInstruction .i: 
        return tagged LoadToFUs i.fus;
      tagged StoreInstruction .i:
        return tagged StoreFromFU i.fu;
      default:
        return ?;
    endcase;
    
    //Handle PLB Commands
    
    Bool isPLB = case (ins) matches
      tagged SetRowSizeInstruction .sz:
        return True;
      tagged LoadInstruction .i:
        return True;
      tagged StoreInstruction .i:
        return True;
      default:
        return False;
    endcase;
    
    PLBMasterCommand plb_cmd = case (ins) matches
      tagged LoadInstruction .i: 
        return tagged LoadPage truncate(i.addr>>2);
      tagged StoreInstruction .i:
        return tagged StorePage truncate(i.addr>>2);
      tagged SetRowSizeInstruction .sz:
        return tagged RowSize sz;
      default:
        return ?;
    endcase;

    //Handle FUNetwork Commands
    
    Bool isFUNet = case (ins) matches
      tagged ForwardInstruction .i:      //{.fuSrc, .regSrc, .fuDests, .regDest}
        return True;
      default:
        return False;
    endcase;
    
    FUNetworkCommand funet_cmd = case (ins) matches
      tagged ForwardInstruction .i:
        return FUNetworkCommand {fuSrc: i.fuSrc, regSrc: i.regSrc, fuDests: i.fuDests, regDest: i.regDest};
      default:
        return ?;
      endcase;
    
    //Handle Finish Commands
    
    Bool isFinish = case (ins) matches
      tagged FinishInstruction:      
        return True;
      default:
        return False;
    endcase;

    //Actually do the enqueues
    
    for (Integer x = 0; x < valueOf(FunctionalUnitNumber); x = x + 1)
      if (unpack(destFUs[x]))
      begin
          fuQs[x].enq(fu_cmd);
	  if (x != 0)
	    $display("WARNING: Controller sending commnad to an FU other than 0!");
      end
      else if (isFwd && fwd_src == fromInteger(x))
      begin
	  fuQs[x].enq(fwd_cmd); //Add the forward src command
	  if (x != 0)
	    $display("WARNING: Controller sending fwdSrc to an FU other than 0!");
      end
  
    if (isFinish)
    begin
        state <= Reporting;
    end

    if (isSwitch)
      switchQ.enq(sw_cmd);
      
    if (isPLB) 
      begin
        debug(controllerDebug, $display("Controller: enqueuing PLB command"));
        plbQ.enq(plb_cmd);
      end

    if (isFUNet)
      funetQ.enq(funet_cmd);
  
  
  endrule

  // need to sync with the storage pipeline
  rule syncInst (isSync && state == Running);
     syncBuffer.deq;
     instQ.deq;
     debug(controllerDebug, $display("Controller: processing Sync"));
  endrule
  
  interface plbMasterCommandOutput = fifoToGet(plbQ);
  interface memorySwitchCommandOutput = fifoToGet(switchQ);
  interface fuNetworkCommandOutput = fifoToGet(funetQ);
  interface functionalUnitCommandOutputs = map(fifoToGet, fuQs);


endmodule
