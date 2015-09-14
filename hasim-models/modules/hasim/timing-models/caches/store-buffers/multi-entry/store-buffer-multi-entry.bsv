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

// ****** Bluespec imports ******

import Vector::*;
import FShow::*;
import FIFO::*;
import FIFOF::*;


// ****** Project imports ******

`include "awb/provides/hasim_common.bsh"
`include "awb/provides/soft_connections.bsh"
`include "awb/provides/hasim_isa.bsh"
`include "awb/provides/funcp_simulated_memory.bsh"
`include "awb/provides/funcp_interface.bsh"

// ****** Timing Model Imports ******

`include "awb/provides/hasim_modellib.bsh"
`include "awb/provides/hasim_model_services.bsh"
`include "awb/provides/l1_cache_base_types.bsh"
`include "awb/provides/chip_base_types.bsh"
`include "awb/provides/pipeline_base_types.bsh"
`include "awb/provides/write_buffer.bsh"
`include "awb/provides/funcp_memstate_base_types.bsh"



typedef Bit#(TLog#(`SB_NUM_ENTRIES)) SB_INDEX;

// mkStoreBuffer

// A simple head/tail circular buffer store buffer.

// This uses an associative memory. Therefore it is best for small sizes. 
// Larger sizes would want to use BRAM or LUTRAM and sequentially search the RAMs.

// This module is pipelined across instances. Stages:
// Stage 1 -> Stage 2
// These stages will never stall.

// There is only one way that a model cycle can end.

typedef struct
{
    Vector#(`SB_NUM_ENTRIES, Maybe#(TOKEN)) tokID;
    Vector#(`SB_NUM_ENTRIES, STORE_TOKEN) storeToken;
    Vector#(`SB_NUM_ENTRIES, Maybe#(MEM_ADDRESS)) physAddress;
    SB_INDEX oldestCommitted;
    SB_INDEX numToCommit;
    SB_INDEX nextFreeSlot;
    WB_INDEX wbCredits;
}
STORE_BUFF_STATE deriving (Eq, Bits);

STORE_BUFF_STATE initStoreBuffState = 
    STORE_BUFF_STATE
    {
        tokID: replicate(Invalid),
        storeToken: newVector(),
        physAddress: replicate(Invalid),
        oldestCommitted: 0,
        numToCommit: 0,
        nextFreeSlot: 0,
        wbCredits: 0
    };

module [HASIM_MODULE] mkStoreBuffer ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("pipe_storebuffer.out");


    // ****** Model State (per Context) ******
    
    MULTIPLEXED_STATE_POOL#(MAX_NUM_CPUS, STORE_BUFF_STATE) statePool <- mkMultiplexedStatePool(initStoreBuffState);
    
    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, TOKEN)             allocFromDec    <- mkPortRecv_Multiplexed("Dec_to_SB_alloc", 1);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, SB_INPUT)          reqFromDMem     <- mkPortRecv_Multiplexed("DMem_to_SB_req", 0);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, SB_DEALLOC_INPUT)  deallocFromCom  <- mkPortRecv_Multiplexed("Com_to_SB_dealloc", 1);
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, WB_INDEX)          creditFromWriteQ  <- mkPortRecv_Multiplexed("WB_to_SB_credit", 1);

    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, SB_OUTPUT)      rspToDMem     <- mkPortSend_Multiplexed("SB_to_DMem_rsp");
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, VOID)          creditToDecode <- mkPortSend_Multiplexed("SB_to_Dec_credit");
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, WB_ENTRY)       storeToWriteQ <- mkPortSend_Multiplexed("SB_to_WB_enq");

    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, BUS_MESSAGE) writebackToDec <- mkPortSend_Multiplexed("SB_to_Dec_writeback");

    // ****** Soft Connections ******
    
    Connection_Client#(FUNCP_REQ_DO_STORES, FUNCP_RSP_DO_STORES) doStores <- mkConnection_Client("funcp_doSpeculativeStores");

    // ****** Local Controller ******

    Vector#(5, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) inports  = newVector();
    Vector#(4, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outports = newVector();
    inports[0]  = reqFromDMem.ctrl;
    inports[1]  = allocFromDec.ctrl;
    inports[2]  = deallocFromCom.ctrl;
    inports[3]  = creditFromWriteQ.ctrl;
    inports[4]  = statePool.ctrl;
    outports[0] = rspToDMem.ctrl;
    outports[1] = creditToDecode.ctrl;
    outports[2] = storeToWriteQ.ctrl;
    outports[3] = writebackToDec.ctrl;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkNamedLocalController("Store Buffer", inports, outports);
    
    STAGE_CONTROLLER#(MAX_NUM_CPUS, STORE_BUFF_STATE) stage2Ctrl <- mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(STORE_BUFF_STATE, Bool)) stage3Ctrl <- mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, STORE_BUFF_STATE) stage4Ctrl <- mkBufferedStageController();



    // ****** Rules ******

    (* conservative_implicit_conditions *)
    rule stage1_alloc (True);
    
        // Start a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();
        
        let local_state <- statePool.extractState(cpu_iid);

        // Check if the decode is allocating a new slot.
        let m_alloc <- allocFromDec.receive(cpu_iid);
        
        if (m_alloc matches tagged Valid .tok)
        begin
        
            // Allocate a new slot.
            // assert !full(cpu_iid)
            debugLog.record(cpu_iid, fshow("ALLOC ") + fshow(tok) + $format(" SLOT: %0d", local_state.nextFreeSlot));
            local_state.tokID[local_state.nextFreeSlot] = tagged Valid tok;

            // We don't know its effective address yet.
            local_state.physAddress[local_state.nextFreeSlot] = tagged Invalid;

            local_state.nextFreeSlot = local_state.nextFreeSlot + 1;
        
        end
        
        // Calculate the credit for decode.
        if (((local_state.nextFreeSlot + 2) == local_state.oldestCommitted) || ((local_state.nextFreeSlot + 1) == local_state.oldestCommitted)) // Plus 2 because we assume it takes a cycle for the credit to arrive. Should really be +L+1 where L is latency of credit.
        begin

            // Tell decode we're full.
            debugLog.record(cpu_iid, fshow("NO CREDIT"));
            creditToDecode.send(cpu_iid, tagged Invalid);

        end
        else
        begin

            // Tell decode still have room.
            debugLog.record(cpu_iid, fshow("SEND CREDIT"));
            creditToDecode.send(cpu_iid, tagged Valid (?));
        
        end

        // Continue to the next stage.
        stage2Ctrl.ready(cpu_iid, local_state);

    endrule


    // stage2_search
    
    // Helper function for stage2 store buffer entry CAM
    function Bool matchCompletedStore(TOKEN storeTok,
                                      Tuple2#(Maybe#(TOKEN), Maybe#(MEM_ADDRESS)) sb);
        match {.sb_tok, .sb_pa} = sb;
        // To match the token must be the same AND there must not yet be a PA
        // assigned.  A valid PA means the store is already completed.  Two store
        // buffer entries may share the same token if the token space has wrapped
        // with a store outstanding.  Tokens aren't used by the store buffer for
        // precisely this reason.  Requiring a non-valid PA forces a match of the
        // active instance of a token.
        return (isValid(sb_tok) &&
                tokTokenId(validValue(sb_tok)) == tokTokenId(storeTok) &&
                ! isValid(sb_pa));
    endfunction

    (* conservative_implicit_conditions *)
    rule stage2_search (True);

        match {.cpu_iid, .local_state} <- stage2Ctrl.nextReadyInstance();

        // See if the DMem is completing or searching.
        let m_req <- reqFromDMem.receive(cpu_iid);

        if (m_req matches tagged Valid .req)
        begin

            case (req.reqType) matches
                tagged SB_search:
                begin

                    let target_addr = req.bundle.physicalAddress;

                    // Luckily, since we're a simulation, we don't actually 
                    // need to retrieve the value, which makes the hardware a LOT simpler
                    // as we don't need to get the "youngest store older than this load"
                    // Instead, just tell the DMem module that we have the value.
                
                    Bool hit = False;
                
                    for (Integer x = 0; x < `SB_NUM_ENTRIES; x = x + 1)
                    begin

                        // It's a hit if it's a store to the same address which is older than the load.
                        let addr_match = case (local_state.physAddress[x]) matches 
                                            tagged Valid .addr: return addr == target_addr;
                                            tagged Invalid: return False;
                                         endcase;

                        let older_store = case (local_state.tokID[x]) matches 
                                                tagged Valid .tok: return tokenIsOlderOrEq(tok.index.token_id, req.bundle.token.index.token_id);
                                                tagged Invalid: return False;
                                            endcase;

                        hit = hit || (addr_match && older_store);
                    end

                    if (hit)
                    begin

                        // We've got that address in the store buffer.
                        debugLog.record(cpu_iid, fshow("LOAD HIT ") + fshow(req.bundle.token));

                        rspToDMem.send(cpu_iid, tagged Valid initSBHit(req.bundle));
                        writebackToDec.send(cpu_iid, tagged Invalid);

                    end
                    else
                    begin

                        // We don't have it.
                        debugLog.record(cpu_iid, fshow("LOAD MISS ") + fshow(req.bundle.token));
                        rspToDMem.send(cpu_iid, tagged Valid initSBMiss(req.bundle));
                        writebackToDec.send(cpu_iid, tagged Invalid);

                    end
                    
                    // Continue in the next stage.
                    stage3Ctrl.ready(cpu_iid, tuple2(local_state, False));

                end
                tagged SB_complete:
                begin

                    // Update with the actual physical address.
                    // (A real store buffer would also record the value.)
                    
                    // We find the index for this token using a CAM Write.  There
                    // must be a match (could add an assertion), so no check
                    // is used for actually finding the entry.                    
                    let idx = findIndex(matchCompletedStore(req.bundle.token),
                                        zip(local_state.tokID, local_state.physAddress));
                    SB_INDEX sb_idx = pack(validValue(idx));
                    
                    // A completion of a previously allocated store.
                    debugLog.record(cpu_iid, fshow("COMPLETE STORE ") + fshow(req.bundle.token) + $format(" SLOT: %0d", sb_idx));

                    local_state.physAddress[sb_idx] = tagged Valid req.bundle.physicalAddress;

                    // Tell the functional partition to make the store locally visible.
                    doStores.makeReq(initFuncpReqDoStores(req.bundle.token));

                    // No need for a response.
                    rspToDMem.send(cpu_iid, tagged Invalid);

                    // In some architectures, store may update a register (e.g. Alpha stq_c)
                    // The branch epoch is irrelevant at this late stage.
                    let epoch = initEpoch(?, req.bundle.faultEpoch);
                    writebackToDec.send(cpu_iid, tagged Valid genBusMessage(req.bundle.token,
                                                                            epoch,
                                                                            req.bundle.dests));

                    // Get the store respone in the next stage.
                    stage3Ctrl.ready(cpu_iid, tuple2(local_state, True));

                end

            endcase
        end
        else
        begin

            // Propogate the bubble.
            debugLog.record(cpu_iid, fshow("NO SEARCH"));
            rspToDMem.send(cpu_iid, Invalid);
            writebackToDec.send(cpu_iid, tagged Invalid);
            stage3Ctrl.ready(cpu_iid, tuple2(local_state, False));

        end

    endrule

    rule stage3_dealloc (True);
    
        // Get our context from the previous stage.
        match {.cpu_iid, {.local_state, .get_rsp}} <- stage3Ctrl.nextReadyInstance();

        // See if we need to get a store response.
        if (get_rsp)
        begin
            let rsp = doStores.getResp();
            doStores.deq();
        end

        // See if we're getting a deallocation request.
        let m_dealloc <- deallocFromCom.receive(cpu_iid);
        
        if (m_dealloc matches tagged Valid .req &&& req.reqType == SB_drop)
        begin

            // Invalidate the requested entry. We assume drop/dealloc requests come in allocation order.
            debugLog.record(cpu_iid, fshow("DROP REQ ") + fshow(req.token));
            local_state.tokID[local_state.oldestCommitted + local_state.numToCommit] = tagged Invalid;

            // Record that the commit path has work to do.
            local_state.numToCommit = local_state.numToCommit + 1;
        
        end
        else if (m_dealloc matches tagged Valid .req &&& req.reqType == SB_writeback)
        begin

            // Update the token with the latest value.
            debugLog.record(cpu_iid, fshow("DEALLOC REQ ") + fshow(req.token));
            local_state.tokID[local_state.oldestCommitted + local_state.numToCommit] = tagged Valid req.token;
            local_state.storeToken[local_state.oldestCommitted + local_state.numToCommit] = req.storeToken;

            // Record that the commit path has work to do.
            local_state.numToCommit = local_state.numToCommit + 1;
        
        end
        
        // Finish up in the next stage.
        stage4Ctrl.ready(cpu_iid, local_state);
        
    endrule
    
    (* conservative_implicit_conditions *)
    rule stage4_commit (True);
    
        // Get our context from the previous stage.
        match {.cpu_iid, .local_state} <- stage4Ctrl.nextReadyInstance();

        // See if the Write Buffer has room.
        let m_credit <- creditFromWriteQ.receive(cpu_iid);
        if (m_credit matches tagged Valid .c)
        begin
            local_state.wbCredits = local_state.wbCredits + 1;
            debugLog.record(cpu_iid, $format("%0d new WB credits, now %0d", c, local_state.wbCredits));
        end
        let write_buff_has_credit = (local_state.wbCredits != 0);

        // We need to dealloc if we have pending commmits.
        if (local_state.numToCommit != 0)
        begin
            case (local_state.tokID[local_state.oldestCommitted]) matches
                tagged Invalid:
                begin
                
                    // If the oldest committed token is invalid then it was dropped. Just move over it.
                    debugLog.record(cpu_iid, $format("JUNK DROPPED, SLOT %0d", local_state.oldestCommitted));
                    local_state.oldestCommitted = local_state.oldestCommitted + 1;
                    local_state.numToCommit = local_state.numToCommit - 1;
                    
                    // No guys to commit.
                    storeToWriteQ.send(cpu_iid, tagged Invalid);

                end
                tagged Valid .tok:
                begin
                    // The oldest token has been committed. Let's see if we can send it to the write buffer.
                    if (local_state.physAddress[local_state.oldestCommitted] matches tagged Valid .phys_addr &&&
                        write_buff_has_credit)
                    begin

                        // It's got room. Let's send the oldest store.
                        debugLog.record(cpu_iid, fshow("DEALLOC ") + fshow(tok) + $format(" SLOT: %0d", local_state.oldestCommitted));

                        // Send it to the writeBuffer.
                        let store_tok = local_state.storeToken[local_state.oldestCommitted];
                        storeToWriteQ.send(cpu_iid, tagged Valid tuple2(store_tok, phys_addr));

                        // Dequeue the old entry.
                        local_state.tokID[local_state.oldestCommitted] = tagged Invalid;
                        local_state.oldestCommitted = local_state.oldestCommitted + 1;
                        local_state.numToCommit = local_state.numToCommit - 1;
                        local_state.wbCredits = local_state.wbCredits - 1;

                    end
                    else
                    begin
                    
                        // No room to commit this guy.
                        debugLog.record(cpu_iid, fshow("DEALLOC STALL ") + fshow(tok) + $format(" SLOT %0d", local_state.oldestCommitted));
                        storeToWriteQ.send(cpu_iid, tagged Invalid);
                    
                    end
                    
                end
            
            endcase
        
        end
        else
        begin

            // No guys to commit.
            debugLog.record(cpu_iid, fshow("NO DEALLOC"));
            storeToWriteQ.send(cpu_iid, tagged Invalid);
        
        end
        
        localCtrl.endModelCycle(cpu_iid, 1);
        debugLog.nextModelCycle(cpu_iid);
        statePool.insertState(cpu_iid, local_state);

    endrule

endmodule
