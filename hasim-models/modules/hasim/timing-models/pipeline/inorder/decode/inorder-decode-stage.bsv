//
// Copyright (c) 2014, Intel Corporation
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// Neither the name of the Intel Corporation nor the names of its contributors
// may be used to endorse or promote products derived from this software
// without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
 

// ****** Bluespec imports  *****

import FShow::*;
import Vector::*;
import FIFO::*;


// ****** Project imports ******

`include "awb/provides/hasim_common.bsh"
`include "awb/provides/soft_connections.bsh"
`include "awb/provides/fpga_components.bsh"
`include "awb/provides/common_services.bsh"
`include "awb/provides/hasim_modellib.bsh"
`include "awb/provides/hasim_isa.bsh"
`include "awb/provides/chip_base_types.bsh"
`include "awb/provides/pipeline_base_types.bsh"
`include "awb/provides/hasim_model_services.bsh"
`include "awb/provides/funcp_interface.bsh"


// ****** Generated files ******

`include "awb/dict/EVENTS_DECODE.bsh"

// ****** Local Datatypes ******

typedef union tagged
{
    void         STAGE3_bubble;
    void         STAGE3_rewindRsp;
    void         STAGE3_depsReady;
    void         STAGE3_depsRsp;
}
DEC_STAGE3_STATE deriving (Bits, Eq);

typedef union tagged
{
    void  STAGE4_bubble;
    void  STAGE4_depsCheck;
}
DEC_STAGE4_STATE deriving (Bits, Eq);
 
typedef struct
{
    TOKEN_INDEX numInstrsInFlight;
    Bool drainingAfter;
    FETCH_BUNDLE currentBundle;
    Maybe#(FUNCP_RSP_GET_DEPENDENCIES) instToIssue;
    TOKEN_EPOCH epoch;
}
DECODE_STATE deriving (Eq, Bits);

DECODE_STATE initialDecodeState =
    DECODE_STATE
    {
        numInstrsInFlight: 0,
        drainingAfter: False,
        currentBundle: ?,
        instToIssue: Invalid,
        epoch: initEpoch(0, 0)
    };


// mkDecode

// Multi-context inorder decode module which stalls the first instruction until its
// source registers have been written. 

// Writes to registers can be reported by the Exe, Mem, or Com stages.

// Certain instructions must stall the pipeline either before or after their execution.

// This module is pipelined across instance. Stages:

// Stage 1 -> Stage 2* -> Stage 3 
// * Stage 2 stalls once for each instruction until it gets a
//   response from the functional partition. Thus we only pay 
//   this penalty once if an instruction stalls in the InstQ.

// Possible ways the model cycle can end:
//   Path 1: InstQ is empty or the IssueQ is full, so we can't issue.
//   Path 2: We issue succesfully.
//   Path 3: The instruction must be stalled on a dependency.

module [HASIM_MODULE] mkDecode ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_decode.out");

    STDIO#(Bit#(64)) stdioDbg <- mkStdIO_Debug();
    let stdio <- mkStdIO_CondPrintf(ioMask_TIMEP_START, stdioDbg);

    let msgSendInst <- getGlobalStringUID("TIMEP Decode %016lld: PC 0x%016llx, TOKEN (%lld, %lld), dst0=%lld\n");
    Reg#(Bit#(64)) modelCycle <- mkReg(-1);

    // ****** Ports *****
    PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS, BUNDLE)        bundleToIssueQ <- mkPortStallSend_Multiplexed("IssueQ");
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, VOID)                    deqToInstQ <- mkPortSend_Multiplexed("Dec_to_InstQ_deq");
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, TOKEN)                    allocToSB <- mkPortSend_Multiplexed("Dec_to_SB_alloc");

    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, FETCH_BUNDLE)      bundleFromInstQ      <- mkPortRecv_Multiplexed("InstQ_to_Dec_first", 0);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, VOID)              creditFromSB         <- mkPortRecv_Multiplexed("SB_to_Dec_credit", 1);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, Tuple2#(TOKEN, TOKEN_FAULT_EPOCH)) mispredictFromExe    <- mkPortRecv_Multiplexed("Exe_to_Dec_mispredict", 1);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, TOKEN)             faultFromCom         <- mkPortRecv_Multiplexed("Com_to_Dec_fault", 1);

    // 0 latency on notification that tokens are ready implies full bypass
    // of register updates, permitting back-to-back execution of register
    // dependent instructions.
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, BUS_MESSAGE)       writebackFromExe     <- mkPortRecv_Multiplexed("Exe_to_Dec_writeback", 0);

    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, BUS_MESSAGE)       writebackFromMemHit  <- mkPortRecv_Multiplexed("DMem_to_Dec_hit_writeback", 1);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, BUS_MESSAGE)       writebackFromMemMiss <- mkPortRecv_Multiplexed("DMem_to_Dec_miss_writeback", 1);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, BUS_MESSAGE)       writebackFromStore   <- mkPortRecv_Multiplexed("SB_to_Dec_writeback", 1);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, TOKEN)             writebackFromCom     <- mkPortRecv_Multiplexed("Com_to_Dec_writeback", 1);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, STORE_TOKEN)       writebackFromWB      <- mkPortRecv_Multiplexed("MemWriteBuf_to_Dec_writeback", 1);

    // ****** Soft Connections ******

    Connection_Client#(FUNCP_REQ_GET_DEPENDENCIES,
                       FUNCP_RSP_GET_DEPENDENCIES) getDependencies <- mkConnection_Client("funcp_getDependencies");

 
    Connection_Client#(FUNCP_REQ_REWIND_TO_TOKEN,
                       FUNCP_RSP_REWIND_TO_TOKEN)  rewindToToken <- mkConnection_Client("funcp_rewindToToken");

    // ****** Model State (per Instance) ******

    MULTIPLEXED_STATE_POOL#(MAX_NUM_CPUS, DECODE_STATE) statePool <- mkMultiplexedStatePool(initialDecodeState);

    // PRF valid bits.
    DECODE_PRF_SCOREBOARD prfScoreboard <- mkPRFScoreboardLUTRAM();
    // DECODE_PRF_SCOREBOARD prfScoreboard <- mkPRFScoreboardMultiWrite(); // Can be switched for expensive version.

    COUNTER_Z#(CPU_INSTANCE_ID_SZ) nGetDepInFlight <- mkLCounter_Z(0);
    COUNTER_Z#(CPU_INSTANCE_ID_SZ) nRewindInFlight <- mkLCounter_Z(0);

    // ****** Local Controller ******

    DEPENDENCE_CONTROLLER#(NUM_CONTEXTS) wbExeCtrl  <- mkDependenceController();
    DEPENDENCE_CONTROLLER#(NUM_CONTEXTS) wbHitCtrl  <- mkDependenceController();
    DEPENDENCE_CONTROLLER#(NUM_CONTEXTS) wbMissCtrl <- mkDependenceController();
    DEPENDENCE_CONTROLLER#(NUM_CONTEXTS) wbStoreCtrl  <- mkDependenceController();

    Vector#(8, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS))  inports  = newVector();
    Vector#(8, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS))  depports = newVector();
    Vector#(3, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outports = newVector();
    inports[0]  = bundleToIssueQ.ctrl.in;
    inports[1]  = creditFromSB.ctrl;
    inports[2]  = mispredictFromExe.ctrl;
    inports[3]  = faultFromCom.ctrl;
    inports[4]  = bundleFromInstQ.ctrl;
    inports[5]  = writebackFromCom.ctrl;
    inports[6]  = writebackFromWB.ctrl;
    inports[7]  = statePool.ctrl;
    depports[0] = writebackFromExe.ctrl;
    depports[1] = writebackFromMemHit.ctrl;
    depports[2] = writebackFromMemMiss.ctrl;
    depports[3] = writebackFromStore.ctrl;
    depports[4] = wbExeCtrl.ctrl;
    depports[5] = wbHitCtrl.ctrl;
    depports[6] = wbMissCtrl.ctrl;
    depports[7] = wbStoreCtrl.ctrl;
    outports[0] = bundleToIssueQ.ctrl.out;
    outports[1] = deqToInstQ.ctrl;
    outports[2] = allocToSB.ctrl;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkNamedLocalControllerWithUncontrolled("Decode", inports, depports, outports);

    STAGE_CONTROLLER#(MAX_NUM_CPUS, DECODE_STATE) stage1aExeCtrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, DECODE_STATE) stage1aMemHitCtrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, DECODE_STATE) stage1aMemMissCtrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, DECODE_STATE) stage1aStoreCtrl <- mkStageController();

    STAGE_CONTROLLER#(MAX_NUM_CPUS, DECODE_STATE) stage2Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(DEC_STAGE3_STATE, DECODE_STATE)) stage3Ctrl <- mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(DEC_STAGE4_STATE, DECODE_STATE)) stage4Ctrl <- mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Maybe#(Tuple2#(TOKEN, ISA_DST_MAPPING))) stage5Ctrl <- mkStageController();

    // ****** Events ******
    EVENT_RECORDER_MULTIPLEXED#(MAX_NUM_CPUS) eventDec <- mkEventRecorder_Multiplexed(`EVENTS_DECODE_INSTRUCTION_DECODE);


    // ***** Helper Functions ******

    // readyToGo
    
    // Check if a token's sources are ready.

    function ActionValue#(Bool) readyToGo(TOKEN tok, ISA_SRC_MAPPING srcmap);
    actionvalue

        // Extract local state from the context.
        let cpu_iid = tokCpuInstanceId(tok);

        // Check if each source register is ready.
        Bool rdy = True;
        for (Integer i = 0; i < valueof(ISA_MAX_SRCS); i = i + 1)
        begin
            if (srcmap[i] matches tagged Valid { .ar, .pr })
            begin
                let is_rdy = prfScoreboard.isReady(pr);
                rdy = rdy && is_rdy;

                if (!is_rdy)
                begin
                    debugLog.record(cpu_iid, fshow(tok) + $format(": PR %0d (AR %0d) not ready", pr, ar));
                end
            end
        end

        return rdy;

    endactionvalue
    endfunction

    // readyDrainBefore
    
    // If an instruction is marked drainBefore, then we must wait until
    // the instructions older than it have committed.

    function Bool readyDrainBefore(DECODE_STATE local_state, ISA_INSTRUCTION inst);
    
        if (isaDrainBefore(inst))
            return local_state.numInstrsInFlight == 0;
        else
            return True;
    
    endfunction
    
    // readyDrainAfter
    
    // If we had previously issued an instruction that was marked drainAfter,
    // then we must wait until instructions older than the next instruction
    // have committed.
    
    function Bool readyDrainAfter(DECODE_STATE local_state);
    
        if (local_state.drainingAfter)
            return local_state.numInstrsInFlight == 0;
        else
            return True;
    
    endfunction


    // makeBundle
    
    // Marshall up a bundle of useful information to send to the rest of the pipeline.

    function BUNDLE makeBundle(TOKEN tok, FETCH_BUNDLE fbndl, ISA_DST_MAPPING dstmap);
        Vector#(ISA_MAX_DSTS,Maybe#(FUNCP_PHYSICAL_REG_INDEX)) dests = newVector();
        for (Integer i = 0; i < valueof(ISA_MAX_DSTS); i = i + 1)
        begin
            if (dstmap[i] matches tagged Valid { .ar, .pr })
                dests[i] = Valid(pr);
            else
                dests[i] = Invalid;
        end

        let is_load = isaIsLoad(fbndl.inst);
        let is_store = isaIsStore(fbndl.inst);

        return BUNDLE { token:   tok,
                        branchEpoch: fbndl.branchEpoch,
                        faultEpoch: fbndl.faultEpoch,
                        isLoad: is_load,
                        isStore: is_store,
                        isTerminate: Invalid,
                        pc: fbndl.pc,
                        branchAttr: fbndl.branchAttr,
                        effAddr: ?,
                        dests: dests,
                        writtenAtEXE: isaWrittenAtEXE(fbndl.inst),
                        writtenAtMEM: (is_store ? isaWrittenAtST(fbndl.inst) :
                                                  isaWrittenAtLD(fbndl.inst)) };
    endfunction

    // ****** Rules ******

    // stage1

    // Head of the pipeline.  Load state.

    (* conservative_implicit_conditions *)
    rule stage1 (True);
        // Begin model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        
        // Extract local state from the cpu instance.
        let local_state <- statePool.extractState(cpu_iid);

        stage1aExeCtrl.ready(cpu_iid, local_state);
        stage1aMemHitCtrl.ready(cpu_iid, local_state);
        stage1aMemMissCtrl.ready(cpu_iid, local_state);
        stage1aStoreCtrl.ready(cpu_iid, local_state);
        stage2Ctrl.ready(cpu_iid, local_state);
    endrule


    // stage1a_writebacks
    
    // Begin simulating a new context.
    // Get any writebacks from Exe, Mem, or Com and update the scoreboard.

    // Ports read:
    // * writebackFromExe
    // * writebackFromMemHit
    // * writebackFromMemMiss
    // * writebackFromStore
    
    // Ports written:
    // * None

    //
    // markPRegsReady --
    //   Common code for updating physical register ready flag, used by all
    //   writeback update rules.
    //
    function Action markPRegsReady(INSTANCE_ID#(MAX_NUM_CPUS) cpu_iid,
                                   DECODE_STATE local_state,
                                   Maybe#(BUS_MESSAGE) m_msg,
                                   Vector#(ISA_MAX_DSTS,
                                           DECODE_PRF_SCOREBOARD_WB_PORT) wbPort,
                                   String dbgSource,
                                   Bool testBranchEpoch);
    action
        if (m_msg matches tagged Valid .msg)
        begin
            // Does the epoch match?  If not, the PR may have been reassigned
            // already.  Ignore the update.
            Bool epoch_valid = (msg.epoch.fault == local_state.epoch.fault);

            // Late pipeline stages don't track the branch epoch, since it
            // is relevant only up to EXE.
            if (testBranchEpoch)
            begin
                epoch_valid = epoch_valid &&
                              (msg.epoch.branch == local_state.epoch.branch);
            end

            // Also ignore dummy tokens for which the destination physical
            // registers may have been reassigned.
            Bool do_update = epoch_valid && ! tokIsDummy(msg.token);

            for (Integer x = 0; x < valueof(ISA_MAX_DSTS); x = x + 1)
            begin
                if (msg.destRegs[x] matches tagged Valid .pr)
                begin
                    if (do_update)
                    begin
                        wbPort[x].ready(pr);
                        debugLog.record(cpu_iid, fshow(msg.token) + $format(": PR %0d is ready -- %s", pr, dbgSource));
                    end
                    else
                    begin
                        debugLog.record(cpu_iid, fshow(msg.token) + $format(": Squash dead PR %0d update -- %s", pr, dbgSource));
                    end
                end
            end
        end
    endaction
    endfunction


    (* conservative_implicit_conditions *)
    rule stage1a_writebackExe (writebackFromExe.ctrl.nextReadyInstance() matches tagged Valid .iid 
                               &&& wbExeCtrl.producerCanStart());
        match {.cpu_iid, .local_state} <- stage1aExeCtrl.nextReadyInstance();

        // Process writes from EXE
        let m_msg <- writebackFromExe.receive(cpu_iid);
        wbExeCtrl.producerStart();
        wbExeCtrl.producerDone();

        markPRegsReady(cpu_iid, local_state,
                       m_msg, prfScoreboard.wbExe, "EXE", True);
    endrule
    
    (* conservative_implicit_conditions *)
    rule stage1a_writebackMemHit (writebackFromMemHit.ctrl.nextReadyInstance() matches tagged Valid .iid
                                  &&& wbHitCtrl.producerCanStart());
        match {.cpu_iid, .local_state} <- stage1aMemHitCtrl.nextReadyInstance();
    
        // Process writes from MEM Hit
        let m_msg <- writebackFromMemHit.receive(cpu_iid);
        wbHitCtrl.producerStart();
        wbHitCtrl.producerDone();

        markPRegsReady(cpu_iid, local_state,
                       m_msg, prfScoreboard.wbHit, "HIT", False);
    endrule

    (* conservative_implicit_conditions *)
    rule stage1a_writebackMemMiss (writebackFromMemMiss.ctrl.nextReadyInstance() matches tagged Valid .iid
                                   &&& wbMissCtrl.producerCanStart());
        match {.cpu_iid, .local_state} <- stage1aMemMissCtrl.nextReadyInstance();
    
        // Process writes from MEM Miss
        let m_msg <- writebackFromMemMiss.receive(cpu_iid);
        wbMissCtrl.producerStart();
        wbMissCtrl.producerDone();

        markPRegsReady(cpu_iid, local_state,
                       m_msg, prfScoreboard.wbMiss, "MISS", False);
    endrule

    (* conservative_implicit_conditions *)
    rule stage1a_writebackStore (writebackFromStore.ctrl.nextReadyInstance() matches tagged Valid .iid 
                                &&& wbStoreCtrl.producerCanStart());
        match {.cpu_iid, .local_state} <- stage1aStoreCtrl.nextReadyInstance();
    
        // Process writes from STORE
        let m_msg <- writebackFromStore.receive(cpu_iid);
        wbStoreCtrl.producerStart();
        wbStoreCtrl.producerDone();

        markPRegsReady(cpu_iid, local_state,
                       m_msg, prfScoreboard.wbStore, "STORE", False);
    endrule
    
        
    // stage2_dependencies
    
    // Check if there's an instruction waiting to be issued. 
    // In order to issue we have to have the dependencies from the functional partition.
    // If we don't have them, request them.
    // If we previously got them, just advance to the next stage.
    
    // Dependence requests and rewind requests conflict in the functional partition.
    // The model may deadlock if the queues for both fill.  When one type of
    // request is outstanding, the other type is blocked here to prevent deadlock.
    // One request of each is fine, since the response FIFO here guarantees
    // buffering for at least one outside the functional partition.  Rewind is
    // expected to be relatively rare, so the effect on simulator performance
    // is negligible.

    // Ports read:
    // * mispredictFromExe
    // * faultFromCom
    // * bundleFromInstQ
    // * writebackFromCom
    // * writebackFromWB
    
    // Ports written:
    // * None
    
    (* conservative_implicit_conditions *)
    rule stage2_dependencies (nGetDepInFlight.isZero() || nRewindInFlight.isZero());
        match {.cpu_iid, .local_state} <- stage2Ctrl.nextReadyInstance();
        
        let m_mispred <- mispredictFromExe.receive(cpu_iid);
        let m_fault   <- faultFromCom.receive(cpu_iid);
        let m_bundle  <- bundleFromInstQ.receive(cpu_iid);
    
        // Process retired instructions from Com.
        let commit <- writebackFromCom.receive(cpu_iid);
        if (commit matches tagged Valid .commit_tok)
        begin
            debugLog.record(cpu_iid, fshow(commit_tok) + $format(": Commit"));
            local_state.numInstrsInFlight = local_state.numInstrsInFlight - 1;
        end

        // Process retired instructions from memory write buffer.
        let wb <- writebackFromWB.receive(cpu_iid);
        if (wb matches tagged Valid .wb_tok)
        begin
            debugLog.record(cpu_iid, fshow(wb_tok) + $format(": WriteBuf"));
            local_state.numInstrsInFlight = local_state.numInstrsInFlight - 1;
        end

        if (m_fault matches tagged Valid .tok)
        begin
            
            // A fault occurred.
            debugLog.record(cpu_iid, fshow("2: FAULT: ") + fshow(tok));
            
            rewindToToken.makeReq(initFuncpReqRewindToToken(tok));
            nRewindInFlight.up();

            // Increment the epoch. Don't do anything with the queue. We'll start dropping instructions on the next cycle.
            local_state.epoch.fault = local_state.epoch.fault + 1;
            
            // Don't dequeue the instQ.
            deqToInstQ.send(cpu_iid, tagged Invalid);
            
            // Tell the following stages it's a rewind.
            stage3Ctrl.ready(cpu_iid, tuple2(tagged STAGE3_rewindRsp, local_state));
        
        end
        else if (m_mispred matches tagged Valid {.tok, .fault_epoch} &&& fault_epoch == local_state.epoch.fault)
        begin

            // A mispredict occurred.
            debugLog.record(cpu_iid, fshow("2: MISPREDICT"));

            rewindToToken.makeReq(initFuncpReqRewindToToken(tok));
            nRewindInFlight.up();

            // Increment the epoch. Don't do anything with the queue. We'll start dropping instructions on the next cycle.
            local_state.epoch.branch = local_state.epoch.branch + 1;
        
            // Don't dequeue the instQ.
            deqToInstQ.send(cpu_iid, tagged Invalid);
            
            // Tell the following stages it's a bubble.
            stage3Ctrl.ready(cpu_iid, tuple2(tagged STAGE3_rewindRsp, local_state));

        end
        else if (isValid(local_state.instToIssue))
        begin
        
            // We have an instruction to issue, tell the next stage to just proceed.
            debugLog.record(cpu_iid, fshow("2: Deps ready."));

            // Don't dequeue the instQ.
            deqToInstQ.send(cpu_iid, tagged Invalid);

            stage3Ctrl.ready(cpu_iid, tuple2(tagged STAGE3_depsReady, local_state));
            
        end
        else if (readyDrainAfter(local_state) &&&
                 m_bundle matches tagged Valid .bundle)
        begin
            
            // There's a new instruction, and we're not stalled.
            // Dequeue the instQ.
            deqToInstQ.send(cpu_iid, tagged Valid (?));
            
            //Is it of the correct epoch?
            if (bundle.branchEpoch == local_state.epoch.branch && bundle.faultEpoch == local_state.epoch.fault)
            begin

                // We need to retrieve dependencies from the functional partition.
                debugLog.record(cpu_iid, fshow("2: Request Deps."));
                getDependencies.makeReq(initFuncpReqGetDependencies(getContextId(cpu_iid), bundle.inst, bundle.pc));
                nGetDepInFlight.up();
                
                // Update the bundle in the local state.
                local_state.currentBundle = bundle;

                // Tell the next stage to get the response.
                stage3Ctrl.ready(cpu_iid, tuple2(tagged STAGE3_depsRsp, local_state));

            end
            else
            begin
            
                // The instruction is from an old epoch, and we haven't gotten
                // a token yet. Tell the following stages to drop it.
                debugLog.record(cpu_iid, fshow("2: SILENT DROP"));

                stage3Ctrl.ready(cpu_iid, tuple2(tagged STAGE3_bubble, local_state));
            
            end
        
        end
        else
        begin
        
            // There's a bubble. Just propogate it.
            // Don't dequeue the instQ.
            deqToInstQ.send(cpu_iid, tagged Invalid);
            debugLog.record(cpu_iid, fshow("2: BUBBLE"));
            stage3Ctrl.ready(cpu_iid, tuple2(tagged STAGE3_bubble, local_state));

        end
 
    endrule

    // stage3_dependenciesRsp
    
    // Get the response from the functional partition (if any).
    
    // Ports read:
    // * None
    
    // Ports written:
    // * None

    rule stage3_dependenciesRsp (True);
    
        // Get the next instance id.
        match {.cpu_iid, {.stage3state, .local_state}} <- stage3Ctrl.nextReadyInstance();

        case (stage3state) matches 
            tagged STAGE3_bubble:
            begin

                // Just propogate the bubble to the next stage.
                stage4Ctrl.ready(cpu_iid, tuple2(tagged STAGE4_bubble, local_state));

            end
            tagged STAGE3_rewindRsp:
            begin

                // Get the rewind response.
                let rsp = rewindToToken.getResp();
                rewindToToken.deq();
                nRewindInFlight.down();
                debugLog.record(cpu_iid, $format("3: REWIND RSP"));
                
                // Any tokens we had were rewound, so drop them.
                local_state.instToIssue = tagged Invalid;

                // Propogate the bubble to the next stage.
                stage4Ctrl.ready(cpu_iid, tuple2(tagged STAGE4_bubble, local_state));

            end
            tagged STAGE3_depsReady:
            begin

                // instToIssue is already up to date.

                // Pass it to the next stage to do the checking.
                stage4Ctrl.ready(cpu_iid, tuple2(tagged STAGE4_depsCheck, local_state));

            end
            tagged STAGE3_depsRsp:
            begin

                // Get the response from the functional partition.
                let rsp = getDependencies.getResp();
                getDependencies.deq();
                nGetDepInFlight.down();

                // Update dependencies.
                local_state.instToIssue = tagged Valid rsp; 

                // Pass it to the next stage to do the checking.
                stage4Ctrl.ready(cpu_iid, tuple2(tagged STAGE4_depsCheck, local_state));

            end
        endcase

    endrule

    // stage4_attemptIssue
    
    // Check to see if we can actually issue the youngest instruction (if any).

    // Ports read:
    // * creditFromSB
    
    // Ports written:
    // * bundleToIssueQ
    // * deqToInstQ
    // * allocToSB

    // Register writebacks are consumed by this stage.
    let writebacksFinished = wbExeCtrl.consumerCanStart() &&
                             wbHitCtrl.consumerCanStart() &&
                             wbMissCtrl.consumerCanStart() &&
                             wbStoreCtrl.consumerCanStart();

    rule stage4_attemptIssue (writebacksFinished);
    
        // Extract the next active instance.
        match {.cpu_iid, {.stage4state, .local_state}} <- stage4Ctrl.nextReadyInstance();

        let model_cycle = modelCycle;
        if (cpu_iid == 0)
        begin
            model_cycle = model_cycle + 1;
            modelCycle <= model_cycle;
        end

        // Registers ready flags are consumed here by readyToGo()
        wbExeCtrl.consumerStart();
        wbHitCtrl.consumerStart();
        wbMissCtrl.consumerStart();
        wbStoreCtrl.consumerStart();

        // Get the store buffer credit in case we're dealing with a store...
        let m_credit <- creditFromSB.receive(cpu_iid);
        let sb_has_credit = isValid(m_credit);
        
        // See if the issueQ has any room.
        let can_enq <- bundleToIssueQ.canEnq(cpu_iid);

        Maybe#(Tuple2#(TOKEN, ISA_DST_MAPPING)) stage5_msg = tagged Invalid;

        if (stage4state matches tagged STAGE4_bubble)
        begin

            // No allocation to the store buffer.
            allocToSB.send(cpu_iid, tagged Invalid);

            // Don't enqueue anything to the IssueQ.
            eventDec.recordEvent(cpu_iid, tagged Invalid);
            bundleToIssueQ.noEnq(cpu_iid);
                
        end
        else if (stage4state matches tagged STAGE4_depsCheck)
        begin

            // assert isValid(instToIssue)
            let deps = validValue(local_state.instToIssue);
            let fetchbundle = local_state.currentBundle;
            let tok = deps.token;
            
            if (can_enq)
            begin
            
                // Check if we can issue.
                let data_ready <- readyToGo(tok, deps.srcMap);

                // If it's a store, we need to be able to allocate a slot in the store buffer.
                let inst_is_store = isaIsStore(fetchbundle.inst);
                let store_ready =  inst_is_store ? sb_has_credit : True;

                if (data_ready && store_ready && readyDrainAfter(local_state) && readyDrainBefore(local_state, fetchbundle.inst))
                begin

                    // Yep... we're ready to send it. 
                    let bundle = makeBundle(tok, fetchbundle, deps.dstMap);
                    debugLog.record(cpu_iid, fshow(tok) + fshow(": SEND INST: ") + fshow(fetchbundle.inst) + fshow(" ") + fshow(bundle));

                    Bit#(64) dbg_dst_reg0 = -1;
                    if (deps.dstMap[0] matches tagged Valid {.ar, .pr})
                    begin
                        dbg_dst_reg0 = zeroExtend(pr);
                    end
                    stdio.printf(msgSendInst, list(model_cycle, resize(bundle.pc),
                                                   zeroExtendNP(tok.index.context_id),
                                                   zeroExtendNP(tok.index.token_id),
                                                   dbg_dst_reg0));

                    eventDec.recordEvent(cpu_iid, tagged Valid zeroExtend(pack(tok.index)));

                    // If it's a store, reserve a slot in the store buffer.
                    if (inst_is_store)
                    begin
                        allocToSB.send(cpu_iid, tagged Valid tok);
                    end
                    else
                    begin
                        allocToSB.send(cpu_iid, tagged Invalid);
                    end

                    // The scoreboard can now be updated.  This is delayed to
                    // the next stage for FPGA timing.
                    stage5_msg = tagged Valid tuple2(tok, deps.dstMap);

                    // Update the number of instructions in flight.
                    local_state.numInstrsInFlight = local_state.numInstrsInFlight + 1;

                    local_state.instToIssue = tagged Invalid;
                    local_state.drainingAfter = isaDrainAfter(fetchbundle.inst);

                    // Enqueue the decoded instruction in the IssueQ.
                    bundleToIssueQ.doEnq(cpu_iid, bundle);

                end
                else
                begin

                    // Nope, we're waiting on an older instruction to write its results.
                    String stall_reason = "unknown";
                    if (! data_ready)
                        stall_reason = "data";
                    else if (! store_ready)
                        stall_reason = "store";
                    else if (! readyDrainAfter(local_state))
                        stall_reason = "drain after";
                    else if (! readyDrainBefore(local_state, fetchbundle.inst))
                        stall_reason = "drain before";

                    debugLog.record(cpu_iid, fshow(tok) + $format(": STALL ON DEPENDENCY (%s)", stall_reason));
                    eventDec.recordEvent(cpu_iid, tagged Invalid);

                    // Propogate the bubble.
                    bundleToIssueQ.noEnq(cpu_iid);
                    allocToSB.send(cpu_iid, tagged Invalid);

                end
            end
            else
            begin
            
                // Nope, the issueQ is full.
                debugLog.record(cpu_iid, fshow(tok) + fshow(": STALL ON ISSUEQ"));
                eventDec.recordEvent(cpu_iid, tagged Invalid);

                // Propogate the bubble.
                bundleToIssueQ.noEnq(cpu_iid);
                allocToSB.send(cpu_iid, tagged Invalid);

            end

        end
        
        statePool.insertState(cpu_iid, local_state);
        stage5Ctrl.ready(cpu_iid, stage5_msg);

    endrule
        

    //
    // stage5_completeIssue --
    //   Logic continuation of previous stage, extracted merely for FPGA
    //   timing.
    //

    // Specify a rule urgency so that if the Scoreboard has less
    // parallelism so there are no compiler warnings.
    (* conservative_implicit_conditions, descending_urgency = "stage1a_writebackMemMiss, stage1a_writebackMemHit, stage1a_writebackExe, stage5_completeIssue" *)
    rule stage5_completeIssue (True);
        // Extract the next active instance.
        match {.cpu_iid, .cmd} <- stage5Ctrl.nextReadyInstance();

        // Done with register flag updates for the new instruction.  Allow
        // the next producer cycle to begin.
        wbExeCtrl.consumerDone();
        wbHitCtrl.consumerDone();
        wbMissCtrl.consumerDone();
        wbStoreCtrl.consumerDone();

        if (cmd matches tagged Valid {.tok, .dst_map})
        begin
            // Update the scoreboard to reflect this instruction issuing.
            // Mark its destination registers as unready until it commits.
            for (Integer x = 0; x < valueof(ISA_MAX_DSTS); x = x + 1)
            begin
                if (dst_map[x] matches tagged Valid { .ar, .pr })
                begin
                    prfScoreboard.issue[x].unready(pr);
                    debugLog.record(cpu_iid, fshow(tok) + $format(": PR %0d (AR %0d) locked", pr, ar));
                end
            end
        end

        localCtrl.endModelCycle(cpu_iid, 1);
        debugLog.nextModelCycle(cpu_iid);
    endrule

endmodule
