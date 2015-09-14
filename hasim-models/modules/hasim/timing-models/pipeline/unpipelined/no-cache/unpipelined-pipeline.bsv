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

import Vector::*;
import FIFO::*;
import FShow::*;

//HASim library imports
`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/soft_services.bsh"
`include "asim/provides/soft_services_lib.bsh"
`include "asim/provides/soft_services_deps.bsh"
`include "asim/provides/common_services.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/fpga_components.bsh"

//Model-specific imports
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/chip_base_types.bsh"

`include "asim/dict/EVENTS_CPU.bsh"

`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/funcp_simulated_memory.bsh"

//************************* Simple Timing Partition ***********************//
//                                                                         //
// This is about the simplest timing partition you can conceive of. It     //
// simply fetches one instruction at a time, executes it, then moves to    //
// the next instruction. This can serve as a good mechanism to verify      //
// the functional partition and can serve as a "golden model" for more     //
// complex timing partitions.                                              //
//                                                                         //
//*************************************************************************//

module [HASIM_MODULE] mkPipeline
    //interface:
        ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_cpu.out");

    // Debugging output stream, useful for getting a stream of status messages
    // when running on an FPGA.
    STDIO#(Bit#(64)) stdio <- mkStdIO_Debug();
    let stdio1 <- mkStdIO_CondPrintf(ioMask_TIMEP_START, stdio);
    let stdio2 <- mkStdIO_CondPrintf(ioMask_TIMEP_START, stdio);
    let stdio3 <- mkStdIO_CondPrintf(ioMask_TIMEP_START, stdio);
    let stdio6 <- mkStdIO_CondPrintf(ioMask_TIMEP_START, stdio);
    let stdio7 <- mkStdIO_CondPrintf(ioMask_TIMEP_START, stdio);
    let stdio8 <- mkStdIO_CondPrintf(ioMask_TIMEP_START, stdio);
    let stdio9 <- mkStdIO_CondPrintf(ioMask_TIMEP_START, stdio);

    let msgPC_VTOA_REQ <- getGlobalStringUID("Timing: ITranslate VA 0x%016llx\n");
    let msgFET         <- getGlobalStringUID("Timing: Fetch PA 0x%016llx\n");
    let msgDEC         <- getGlobalStringUID("Timing: Decode 0x%x\n");
    let msgLOAD_REQ    <- getGlobalStringUID("Timing: TOKEN (%d, %d) load req\n");
    let msgLOAD_REQ_P  <- getGlobalStringUID("Timing: TOKEN (%d, %d) load req [POISON]\n");
    let msgLOAD_RSP    <- getGlobalStringUID("Timing: TOKEN (%d, %d) load rsp\n");
    let msgSTORE_REQ   <- getGlobalStringUID("Timing: TOKEN (%d, %d) store req\n");
    let msgSTORE_REQ_P <- getGlobalStringUID("Timing: TOKEN (%d, %d) store req [POISON]\n");
    let msgSTORE_RSP   <- getGlobalStringUID("Timing: TOKEN (%d, %d) store rsp\n");
    let msgCOMMIT      <- getGlobalStringUID("Timing: TOKEN (%d, %d) local commit\n");
    let msgPOISON      <- getGlobalStringUID("Timing: TOKEN (%d, %d) is poisoned!\n");
    let msgFAULT_REDIR <- getGlobalStringUID("Timing: Fault redirect TOKEN (%d, %d) to 0x%016llx\n");


    //********* State Elements *********//

    // Program counters
    MULTIPLEXED#(MAX_NUM_CPUS, Reg#(ISA_ADDRESS)) pcPool <- mkMultiplexed(mkReg(`PROGRAM_START_ADDR));
    
    //********* UnModel State *******//
    
    Reg#(Maybe#(INSTANCE_ID#(MAX_NUM_CPUS))) dTransStall <- mkReg(tagged Invalid);

    //********* Connections *********//

    Connection_Send#(CONTROL_MODEL_CYCLE_MSG)         linkModelCycle <- mkConnection_Send("model_cycle");

    Connection_Send#(CONTROL_MODEL_COMMIT_MSG)        linkModelCommit <- mkConnection_Send("model_commits");

    Connection_Client#(Maybe#(FUNCP_REQ_DO_ITRANSLATE),
                       FUNCP_RSP_DO_ITRANSLATE)       linkToITR <- mkConnection_Client("funcp_doITranslate");

    Connection_Client#(FUNCP_REQ_GET_INSTRUCTION,
                       FUNCP_RSP_GET_INSTRUCTION)     linkToFET <- mkConnection_Client("funcp_getInstruction");

    Connection_Client#(FUNCP_REQ_GET_DEPENDENCIES,
                       FUNCP_RSP_GET_DEPENDENCIES)    linkToDEC <- mkConnection_Client("funcp_getDependencies");

    Connection_Client#(FUNCP_REQ_GET_RESULTS,
                       FUNCP_RSP_GET_RESULTS)         linkToEXE <- mkConnection_Client("funcp_getResults");

    Connection_Client#(Maybe#(FUNCP_REQ_DO_DTRANSLATE),
                       FUNCP_RSP_DO_DTRANSLATE)       linkToDTR <- mkConnection_Client("funcp_doDTranslate");

    Connection_Client#(FUNCP_REQ_DO_LOADS,
                       FUNCP_RSP_DO_LOADS)            linkToLOA <- mkConnection_Client("funcp_doLoads");

    Connection_Client#(FUNCP_REQ_DO_STORES,
                       FUNCP_RSP_DO_STORES)           linkToSTO <- mkConnection_Client("funcp_doSpeculativeStores");

    Connection_Client#(FUNCP_REQ_COMMIT_RESULTS,
                       FUNCP_RSP_COMMIT_RESULTS)      linkToLCO <- mkConnection_Client("funcp_commitResults");

    Connection_Client#(FUNCP_REQ_COMMIT_STORES,
                       FUNCP_RSP_COMMIT_STORES)       linkToGCO <- mkConnection_Client("funcp_commitStores");

    // For killing. Used in this model only for poisoned instructions
    // (functional model faults).
    Connection_Client#(FUNCP_REQ_REWIND_TO_TOKEN, 
                       FUNCP_RSP_REWIND_TO_TOKEN)     linkToRewindToToken <- mkConnection_Client("funcp_rewindToToken");


    // Events
    EVENT_RECORDER_MULTIPLEXED#(MAX_NUM_CPUS) eventCom <- mkEventRecorder_Multiplexed(`EVENTS_CPU_INSTRUCTION_COMMIT);

    // Stats
    STAT_VECTOR#(MAX_NUM_CPUS) statCom <-
        mkStatCounter_Multiplexed(statName("MODEL_CPU_INSTRUCTION_COMMIT",
                                           "Committed Instructions"));

    Vector#(0, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) inports = newVector();
    Vector#(0, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outports = newVector();

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkLocalController(inports, outports);
    
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(Bool, Bool)) stage4Ctrl <- mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, TOKEN) stage6Ctrl <- mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, TOKEN) stage7Ctrl <- mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, TOKEN) stage8Ctrl <- mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(Bool, Bool)) stage10Ctrl <- mkBufferedStageController();

    //********* Rules *********//

    // Mapping from cpu id to context ids and back.
    function CPU_INSTANCE_ID getCpuInstanceId(CONTEXT_ID ctx_id) = ctx_id;
    function CPU_INSTANCE_ID tokCpuInstanceId(TOKEN tok) = tokContextId(tok);
    function CPU_INSTANCE_ID storeTokCpuInstanceId(STORE_TOKEN st_tok) = storeTokContextId(st_tok);
    function CONTEXT_ID getContextId(CPU_INSTANCE_ID cpu_iid) = cpu_iid;


    //
    // endModelCycle --
    //     Invoked by any rule that is completely done with a token.
    //
    function Action endModelCycle(CPU_INSTANCE_ID cpu_iid);
    action

        debugLog.record(cpu_iid, $format("Model cycle complete."));

        // Sample event & statistic (commit)
        Reg#(ISA_ADDRESS) pc = pcPool[cpu_iid];
        eventCom.recordEvent(cpu_iid, tagged Valid truncate(pc));
        statCom.incr(cpu_iid);

        // Commit counter for heartbeat
        linkModelCommit.send(tuple2(cpu_iid, 1));
        
        // End the model cycle.
        localCtrl.endModelCycle(cpu_iid, 1);
        debugLog.nextModelCycle(cpu_iid);
    endaction
    endfunction


    //
    // Whether an instruction is a load or store is stored in token scratchpad
    // memory.  These are accessor functions...
    //
    function Bool tokIsLoad(TOKEN tok) = unpack(tok.timep_info.scratchpad[0]);
    function Bool tokIsStore(TOKEN tok) = unpack(tok.timep_info.scratchpad[1]);
    function Bool tokIsLoadOrStore(TOKEN tok) = tokIsLoad(tok) || tokIsStore(tok);

    rule stage1_itrReq (True);

        // Begin a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        linkModelCycle.send(cpu_iid);

        // Translate next pc.
        Reg#(ISA_ADDRESS) pc = pcPool[cpu_iid];
        let ctx_id = getContextId(cpu_iid);
        linkToITR.makeReq(tagged Valid initFuncpReqDoITranslate(ctx_id, pc));
        debugLog.record(cpu_iid, $format("Translating virtual address: 0x%h", pc));
        stdio1.printf(msgPC_VTOA_REQ, list1(resize(pc)));

    endrule

    rule stage2_itrRsp_fetReq (True);

        // Get the ITrans response started by stage1_itrReq
        let rsp = linkToITR.getResp();
        linkToITR.deq();

        let cpu_iid = getCpuInstanceId(rsp.contextId);
        Reg#(ISA_ADDRESS) pc = pcPool[cpu_iid];

        debugLog.record(cpu_iid, $format("ITR Responded, hasMore: %0d", rsp.hasMore));

        if (! rsp.hasMore)
        begin

            // Fetch the next instruction
            linkToFET.makeReq(initFuncpReqGetInstruction(rsp.contextId, rsp.physicalAddress, rsp.offset));
            debugLog.record(cpu_iid, $format("Fetching physical address: 0x%h, offset: 0x%h", rsp.physicalAddress, rsp.offset));

            Bit#(64) fetch_pa = zeroExtend(rsp.physicalAddress + zeroExtend(rsp.offset));
            stdio2.printf(msgFET, list1(resize(fetch_pa)));
        end

    endrule

    rule stage3_fetRsp_decReq (True);

        // Get the instruction response
        let rsp = linkToFET.getResp();
        linkToFET.deq();

        let cpu_iid = getCpuInstanceId(rsp.contextId);
        Reg#(ISA_ADDRESS) pc = pcPool[cpu_iid];

        debugLog.record(cpu_iid, $format("FET Responded"));

        // Tell the functional partition to decode the current instruction and place it in flight.
        linkToDEC.makeReq(initFuncpReqGetDependencies(rsp.contextId, rsp.instruction, pc));
        debugLog.record(cpu_iid, $format("Decoding instruction: 0x%h", rsp.instruction));
        stdio3.printf(msgDEC, list1(resize(rsp.instruction)));

        stage4Ctrl.ready(cpu_iid, tuple2(isaIsLoad(rsp.instruction), isaIsStore(rsp.instruction)));

    endrule

    rule stage4_decRsp_exeReq (True);

        match {.*, .is_load, .is_store} <- stage4Ctrl.nextReadyInstance();

        // Get the decode response
        let rsp = linkToDEC.getResp();
        linkToDEC.deq();

        // Get the token the functional partition assigned to this instruction.
        let tok = rsp.token;
        let cpu_iid = tokCpuInstanceId(tok);

        debugLog.record(cpu_iid, $format("DEC Responded with token: %0d", tokTokenId(tok)));

        // Record load and store properties for the instruction in the token
        tok.timep_info.scratchpad[0] = pack(is_load);
        tok.timep_info.scratchpad[1] = pack(is_store);

        // In a more complex processor we would use the dependencies 
        // to determine if we can issue the instruction.

        // Execute the instruction
        linkToEXE.makeReq(initFuncpReqGetResults(tok));
        debugLog.record(cpu_iid, $format("Executing token: %0d", tokTokenId(tok)));

    endrule

    rule stage5_exeRsp (True);

        // Get the execution result
        let exe_resp = linkToEXE.getResp();
        linkToEXE.deq();

        let tok = exe_resp.token;
        let res = exe_resp.result;

        let cpu_iid = tokCpuInstanceId(tok);
        Reg#(ISA_ADDRESS) pc = pcPool[cpu_iid];

        debugLog.record(cpu_iid, $format("EXE Responded with token: %0d", tokTokenId(tok)));

        // If it was a branch we must update the PC.
        case (res) matches

            tagged RBranchTaken .addr:
            begin

                debugLog.record(cpu_iid, $format("Branch taken to address: 0x%h", addr));
                pc <= addr;

            end

            tagged RBranchNotTaken .addr:
            begin

                debugLog.record(cpu_iid, $format("Branch not taken"));
                pc <= pc + 4;

            end

            tagged RTerminate .pf:
            begin

                debugLog.record(cpu_iid, $format("Terminating Execution. PassFail: %0b", pf));
                localCtrl.instanceDone(cpu_iid, pf);

            end

            default:
            begin

                pc <= pc + 4;

            end

        endcase

        let is_load = tokIsLoad(tok);
        let is_store = tokIsStore(tok);

        if (is_load || is_store)
        begin
            // Memory ops require more work.
            debugLog.record(cpu_iid, $format("DTranslate request for token: %0d", tokTokenId(tok)));

            // Get the physical address(es) of the memory access.
            linkToDTR.makeReq(tagged Valid initFuncpReqDoDTranslate(tok));
        end
        else
        begin
            // Unlike most functional methods, doITranslate requires
            // a no-message.  There will be no response.
            linkToDTR.makeReq(tagged Invalid);
        end

        // Everything else is a no-op in the next stage.
        stage6Ctrl.ready(cpu_iid, tok);

    endrule

    rule stage6_dtrRsp_memReq (!isValid(dTransStall));
    
        match {.cpu_iid, .tok} <- stage6Ctrl.nextReadyInstance();
    
        if (tokIsLoad(tok) || tokIsStore(tok))
        begin
    
            // Get the response from dTranslate
            let rsp = linkToDTR.getResp();
            linkToDTR.deq();

            tok = rsp.token;

            debugLog.record(cpu_iid, $format("DTR Responded for token: %0d, hasMore: %0d", tokTokenId(tok), rsp.hasMore));

            if (! rsp.hasMore)
            begin

                if (tokIsLoad(tok))
                begin

                    // Request the load(s).
                    linkToLOA.makeReq(initFuncpReqDoLoads(tok));
                    debugLog.record(cpu_iid, fshow(tok.index) + $format(": Do loads"));
                    stdio6.printf(! tokIsPoisoned(tok) ? msgLOAD_REQ :
                                                         msgLOAD_REQ_P,
                                 list2(zeroExtend(tokContextId(tok)),
                                       zeroExtend(tokTokenId(tok))));

                end
                else if (tokIsStore(tok))
                begin

                    // Request the store(s)
                    linkToSTO.makeReq(initFuncpReqDoStores(tok));
                    debugLog.record(cpu_iid, fshow(tok.index) + $format(": Do stores"));
                    stdio6.printf(! tokIsPoisoned(tok) ? msgSTORE_REQ :
                                                         msgSTORE_REQ_P,
                                 list2(zeroExtend(tokContextId(tok)),
                                       zeroExtend(tokTokenId(tok))));

                end

                stage7Ctrl.ready(cpu_iid, tok);

            end
            else
            begin
            
                dTransStall <= tagged Valid cpu_iid;
            
            end

        end
        else
        begin
            stage7Ctrl.ready(cpu_iid, tok);
        end
        
    endrule
    
    rule stage6_dtrRsp_unaligned (dTransStall matches tagged Valid .cpu_iid);
    
        // Get the response from dTranslate
        let rsp = linkToDTR.getResp();
        linkToDTR.deq();

        let tok = rsp.token;

        debugLog.record(cpu_iid, $format("DTR Responded (unaligned) for token: %0d, hasMore: %0d", tokTokenId(tok), rsp.hasMore));

        if (! rsp.hasMore)
        begin

            if (tokIsLoad(tok))
            begin

                // Request the load(s).
                linkToLOA.makeReq(initFuncpReqDoLoads(tok));
                debugLog.record(cpu_iid, fshow(tok.index) + $format(": Do loads"));
                stdio6.printf(! tokIsPoisoned(tok) ? msgLOAD_REQ :
                                                     msgLOAD_REQ_P,
                             list2(zeroExtend(tokContextId(tok)),
                                   zeroExtend(tokTokenId(tok))));

            end
            else if (tokIsStore(tok))
            begin

                // Request the store(s)
                linkToSTO.makeReq(initFuncpReqDoStores(tok));
                debugLog.record(cpu_iid, fshow(tok.index) + $format(": Do stores"));
                stdio6.printf(! tokIsPoisoned(tok) ? msgSTORE_REQ :
                                                     msgSTORE_REQ_P,
                             list2(zeroExtend(tokContextId(tok)),
                                   zeroExtend(tokTokenId(tok))));

            end

            stage7Ctrl.ready(cpu_iid, tok);
            dTransStall <= tagged Invalid;

        end
        
    endrule

    rule stage7_memRsp (True);

        match {.cpu_iid, .tok} <- stage7Ctrl.nextReadyInstance();

        if (tokIsLoad(tok))
        begin

            // Get the load response
            let rsp = linkToLOA.getResp();
            linkToLOA.deq();

            tok = rsp.token;

            debugLog.record(cpu_iid, $format("Load ops responded for token: %0d", tokTokenId(tok)));
            stdio7.printf(msgLOAD_RSP, list2(zeroExtend(tokContextId(tok)),
                                             zeroExtend(tokTokenId(tok))));

        end
        else if (tokIsStore(tok))
        begin

            // Get the store response
            let rsp = linkToSTO.getResp();
            linkToSTO.deq();

            tok = rsp.token;

            debugLog.record(cpu_iid, $format(": Store ops responded for token: %0d", tokTokenId(tok)));
            stdio7.printf(msgSTORE_RSP, list2(zeroExtend(tokContextId(tok)),
                                              zeroExtend(tokTokenId(tok))));

        end
        
        stage8Ctrl.ready(cpu_iid, tok);
        
    endrule

    rule stage8_lcoReq (True);

        match {.*, .tok} <- stage8Ctrl.nextReadyInstance();

        // Get the current token from previous rule
        let cpu_iid = tokCpuInstanceId(tok);

        // Locally commit the token.
        linkToLCO.makeReq(initFuncpReqCommitResults(tok));
        if (! tokIsPoisoned(tok))
        begin
            debugLog.record(cpu_iid, $format("Locally committing token: %0d", tokTokenId(tok)));
            stdio8.printf(msgCOMMIT, list2(zeroExtend(tokContextId(tok)),
                                           zeroExtend(tokTokenId(tok))));
        end
        else
        begin
            debugLog.record(cpu_iid, $format("Locally committing POISONED token: %0d", tokTokenId(tok)));
            stdio8.printf(msgPOISON, list2(zeroExtend(tokContextId(tok)),
                                           zeroExtend(tokTokenId(tok))));
        end

    endrule

    rule stage9_lcoRsp_gcoReq (True);

        let rsp = linkToLCO.getResp();
        linkToLCO.deq();

        let tok = rsp.token;
        let cpu_iid = tokCpuInstanceId(tok);

        debugLog.record(cpu_iid, $format("LCO responded for token: %0d", tokTokenId(tok)));

        Reg#(ISA_ADDRESS) pc = pcPool[cpu_iid];

        if (rsp.faultRedirect matches tagged Valid .new_addr)
        begin

            debugLog.record(cpu_iid, $format("LCO responded with fault for token: %0d, new PC: 0x%h", tokTokenId(tok), new_addr));
            stdio9.printf(msgFAULT_REDIR, list3(zeroExtend(tokContextId(tok)),
                                                zeroExtend(tokTokenId(tok)),
                                                resize(new_addr)));

            // Next PC following fault
            pc <= new_addr;

            // Trigger a rewind.  For the unpipelined model this isn't strictly
            // necessary, but it is a useful test of the functional model.
            linkToRewindToToken.makeReq(initFuncpReqRewindToToken(tok));

            stage10Ctrl.ready(cpu_iid, tuple2(False, True));

        end
        else if (rsp.storeToken matches tagged Valid .st_tok)
        begin

            // Request global commit of stores.
            linkToGCO.makeReq(initFuncpReqCommitStores(st_tok));
            debugLog.record(cpu_iid, $format("Globally committing stores for token: %0d (store token: %0d)", tokTokenId(tok), storeTokTokenId(st_tok)));
            stage10Ctrl.ready(cpu_iid, tuple2(True, False));

        end
        else
        begin

            stage10Ctrl.ready(cpu_iid, tuple2(False, False));

        end

    endrule
  
    (* descending_urgency = "stage10_gcoRsp, stage9_lcoRsp_gcoReq" *)
    rule stage10_gcoRsp (True);

        match {.cpu_iid, .is_store, .is_rewind} <- stage10Ctrl.nextReadyInstance();

        if (is_store)
        begin

            // Get the global commit response
            let rsp = linkToGCO.getResp();
            linkToGCO.deq();

            let st_tok = rsp.storeToken;

            debugLog.record(cpu_iid, fshow(st_tok.index) + $format("GCO Responded for store token: %0d", storeTokTokenId(st_tok)));

        end
        
        if (is_rewind)
        begin

            // Get the rewind response
            let rsp = linkToRewindToToken.getResp();
            linkToRewindToToken.deq();

            debugLog.record(cpu_iid, $format("Fault rewind complete for token: %0d", tokTokenId(rsp.token)));

        end
        
        endModelCycle(cpu_iid);

    endrule


endmodule

