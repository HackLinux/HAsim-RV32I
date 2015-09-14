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

// ******* Library Imports *******

import Vector::*;
import DefaultValue::*;


// ******* Application Imports *******

`include "awb/provides/soft_connections.bsh"
`include "awb/provides/common_services.bsh"
`include "awb/provides/fpga_components.bsh"

// ******* HAsim Imports *******

`include "awb/provides/hasim_common.bsh"
`include "awb/provides/hasim_model_services.bsh"
`include "awb/provides/hasim_modellib.bsh"

`include "awb/provides/funcp_base_types.bsh"
`include "awb/provides/funcp_memstate_base_types.bsh"
`include "awb/provides/funcp_interface.bsh"

// ******* Timing Model Imports *******

`include "awb/provides/memory_base_types.bsh"
`include "awb/provides/chip_base_types.bsh"
`include "awb/provides/hasim_cache_protocol.bsh"
`include "awb/provides/l1_cache_base_types.bsh"
`include "awb/provides/hasim_cache_algorithms.bsh"
`include "awb/provides/hasim_l1_icache_alg.bsh"
`include "awb/provides/hasim_miss_tracker.bsh"

// ****** Local Definitions *******

//
// States associated with entries.  Invalid isn't listed because it is provided
// as part of the cache algorithm.
//
typedef enum
{
    // Shared
    L1I_STATE_S,
    // Fill in progress.  No data yet.
    L1I_STATE_FILL
}
L1I_ENTRY_STATE
    deriving (Eq, Bits);

instance FShow#(L1I_ENTRY_STATE);
    function Fmt fshow(L1I_ENTRY_STATE state);
        let str =
            case (state) matches
                L1I_STATE_S: return "S";
                L1I_STATE_FILL: return "FILL";
                default: return "UNDEFINED";
            endcase;

        return $format(str);
    endfunction
endinstance


typedef union tagged
{
    L1_ICACHE_MISS_TOKEN MULTI_LOAD;
    CACHE_PROTOCOL_MSG   CACHE_MSG;
    ICACHE_INPUT         LOAD_REQ;
    void Invalid;
}
L1I_OPER
    deriving (Eq, Bits);

// newL1ICOper --
//   The compiler sometimes fails to infer the type when initialized.  This
//   function makes types clear.
function L1I_OPER newL1ICOper(L1I_OPER oper) = oper;


// IC_LOCAL_STATE
//
// Local State to pass between pipeline stages.

typedef struct
{
    Maybe#(L1_ICACHE_MISS_TOKEN) missTokToFree;

    Vector#(2, Bool) toL2QNotFull;
    Vector#(2, Bool) toL2QUsed;
    Vector#(2, CACHE_PROTOCOL_MSG) toL2QData;

    Bool cacheUpdUsed;
    Bool cacheUpdInval;
    L1_ICACHE_IDX cacheUpdIdx;
    L1I_ENTRY_STATE cacheUpdState;
    LINE_ADDRESS cacheUpdPAddr;

    Maybe#(ICACHE_OUTPUT_IMMEDIATE) loadRspImm;
    Maybe#(ICACHE_OUTPUT_DELAYED) loadRsp;
}
IC_LOCAL_STATE
    deriving (Eq, Bits);

instance DefaultValue#(IC_LOCAL_STATE);
    defaultValue = IC_LOCAL_STATE { 
        missTokToFree: tagged Invalid,
        toL2QNotFull: ?,
        toL2QUsed: replicate(False),
        toL2QData: ?,
        cacheUpdUsed: False,
        cacheUpdInval: False,
        cacheUpdIdx: ?,
        cacheUpdState: ?,
        cacheUpdPAddr: ?,
        loadRspImm: tagged Invalid,
        loadRsp: tagged Invalid
        };
endinstance



// mkL11Cache

// A model of a straightforward L1 ICache.
// There is no victim buffer.
//
// Note that the module itself is implmented as a pipeline, though the target
// model carries out all actions in one model cycle.

module [HASIM_MODULE] mkL1ICache ();

    TIMEP_DEBUG_FILE_MULTIPLEXED#(MAX_NUM_CPUS) debugLog <- mkTIMEPDebugFile_Multiplexed("cache_l1_instruction.out");

 
    // ****** Submodules ******

    //
    // The cache algorithm which determines hits, misses, and evictions.
    // The mayEvict function is used inside the algorithm when returning
    // a candidate for replacement.  mayEvict indicates when an entry
    // may not be victimized.
    //
    function Bool mayEvict(L1I_ENTRY_STATE state) = (state != L1I_STATE_FILL);

    L1_ICACHE_ALG#(MAX_NUM_CPUS, L1I_ENTRY_STATE) iCacheAlg <-
        mkL1ICacheAlg(mayEvict);

    // Track the next Miss ID to give out.
    CACHE_MISS_TRACKER#(MAX_NUM_CPUS, ICACHE_MISS_ID_SIZE) outstandingMisses <- mkCoalescingCacheMissTracker();


    // ****** Ports ******

    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, ICACHE_INPUT) loadReqFromCPU <- mkPortRecv_Multiplexed("CPU_to_ICache_load", 0);

    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, ICACHE_OUTPUT_IMMEDIATE) loadRspImmToCPU <- mkPortSend_Multiplexed("ICache_to_CPU_load_immediate");

    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, ICACHE_OUTPUT_DELAYED) loadRspDelToCPU <- mkPortSend_Multiplexed("ICache_to_CPU_load_delayed");

    // Queues to and from the memory hierarchy, encapsulated as StallPorts.
    Vector#(2, PORT_STALL_SEND_MULTIPLEXED#(MAX_NUM_CPUS,
                                            CACHE_PROTOCOL_MSG)) portToL2 = newVector();
    portToL2[0] <- mkPortStallSend_Multiplexed("L1_ICache_OutQ_0");
    portToL2[1] <- mkPortStallSend_Multiplexed("L1_ICache_OutQ_1");

    PORT_STALL_RECV_MULTIPLEXED#(MAX_NUM_CPUS, CACHE_PROTOCOL_MSG) portFromL2 <- mkPortStallRecv_Multiplexed("L1_ICache_InQ_0");


    // ****** Local Controller ******

    Vector#(4, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) inports = newVector();
    Vector#(5, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outports = newVector();
    
    inports[0] = loadReqFromCPU.ctrl;
    inports[1] = portFromL2.ctrl.in;
    inports[2] = portToL2[0].ctrl.in;
    inports[3] = portToL2[1].ctrl.in;
    outports[0] = loadRspImmToCPU.ctrl;
    outports[1] = loadRspDelToCPU.ctrl;
    outports[2] = portFromL2.ctrl.out;
    outports[3] = portToL2[0].ctrl.out;
    outports[4] = portToL2[1].ctrl.out;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkNamedLocalController("L1 ICache", inports, outports);

    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(L1I_OPER,
                                            IC_LOCAL_STATE)) stage2Ctrl <-
        mkStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(L1I_OPER,
                                            IC_LOCAL_STATE)) stage3Ctrl <-
        mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple3#(L1I_OPER,
                                            IC_LOCAL_STATE,
                                            Bool)) stage4Ctrl <-
        mkBufferedStageController();
    STAGE_CONTROLLER#(MAX_NUM_CPUS, Tuple2#(L1I_OPER,
                                            IC_LOCAL_STATE)) stage5Ctrl <-
        mkStageController();


    // A RAM to record the future homes of outstanding fills.
    MEMORY_IFC_MULTIPLEXED#(MAX_NUM_CPUS,
                            L1_ICACHE_MISS_ID,
                            L1_ICACHE_IDX) fillIdxPool <-
        mkMemory_Multiplexed(mkBRAM);

    // ****** Stats ******

    STAT_VECTOR#(MAX_NUM_CPUS) statHit <-
        mkStatCounter_Multiplexed(statName("MODEL_L1_ICACHE_HIT",
                                           "L1 ICache Controller Read Hits"));
    STAT_VECTOR#(MAX_NUM_CPUS) statMiss <-
        mkStatCounter_Multiplexed(statName("MODEL_L1_ICACHE_MISS",
                                           "L1 ICache Controller Read Misses"));
    STAT_VECTOR#(MAX_NUM_CPUS) statRetry <-
        mkStatCounter_Multiplexed(statName("MODEL_L1_ICACHE_RETRY",
                                           "L1 ICache Controller Read Retries"));

    // ****** Assertions ******

    Vector#(2, ASSERTION_STR_CLIENT) assertNode <- mkAssertionStrClientVec();

    let assertInFill <-
        mkAssertionStrCheckerWithMsg("l1-icache-controller-MESI.bsv: Entry not in FILL state",
                                     ASSERT_ERROR,
                                     assertNode[0]);

    let assertL2MsgOk <-
        mkAssertionStrCheckerWithMsg("l1-icache-controller-MESI.bsv: Unexpected message from L2",
                                     ASSERT_ERROR,
                                     assertNode[1]);


    // ****** Functions ******

    //
    // getOper --
    //   Pick out the L1I_OPER argument from a stage controller payload.
    //   Assumes the operation is always the first element in the payload
    //   tuple.
    //
    function L1I_OPER getOper(STAGE_CONTROLLER#(MAX_NUM_CPUS, t_ARGS) ctrl)
        provisos (Has_tpl_1#(t_ARGS, L1I_OPER));

        match {.cpu_iid, .payload} = ctrl.peekReadyInstance();
        return tpl_1(payload);
    endfunction


    // ****** Rules ******

    // ====================================================================
    //
    //  Stage 1:  Consume inputs, populate local_state, and pick an
    //  operation to perform this target machine cycle.
    //
    // ====================================================================

    (* conservative_implicit_conditions *)
    rule stage1_pickOperation (True);
        // Start a new model cycle
        let cpu_iid <- localCtrl.startModelCycle();

        // Make a conglomeration of local information to pass from stage to stage.
        IC_LOCAL_STATE local_state = defaultValue;

        // Check if the toL2Q has room for any new requests.
        local_state.toL2QNotFull[0] <- portToL2[0].canEnq(cpu_iid);
        local_state.toL2QNotFull[1] <- portToL2[1].canEnq(cpu_iid);

        // Consume incoming messages
        let m_fromL2 <- portFromL2.receive(cpu_iid);
        let m_cpu_req_load <- loadReqFromCPU.receive(cpu_iid);

        //
        // Pick an action for the model cycle.
        //

        L1I_OPER oper = tagged Invalid;

        // Are any previously returned fills going to multiple loads?
        if (outstandingMisses.fillToDeliver(cpu_iid) matches tagged Valid .miss_tok)
        begin
            oper = tagged MULTI_LOAD miss_tok;
            debugLog.record(cpu_iid, $format("1: FILL MULTIPLE RSP: %0d", miss_tok.index));
        end
        else if (m_fromL2 matches tagged Valid .fromL2)
        begin
            oper = tagged CACHE_MSG fromL2;

            // Request/response from lower in the cache hierarchy.
            if (fromL2.kind matches tagged RSP_LOAD .fill_meta)
            begin
                // Fill response.
                L1_ICACHE_MISS_TOKEN miss_tok = fromMemOpaque(fromL2.opaque);
                debugLog.record(cpu_iid, $format("1: ") + fshow(fromL2) + $format(", idx %0d", miss_tok.index));
            end
            else if (fromL2.kind matches tagged FORCE_INVAL .inval_meta)
            begin
                // Writeback or invalidation request.
                debugLog.record(cpu_iid, $format("1: ") + fshow(fromL2));
            end
            else
            begin
                // Unexpected message!
                Fmt msg = $format("1: UNEXPECTED: ") + fshow(fromL2);
                debugLog.record(cpu_iid, msg);
                assertL2MsgOk(False,
                              msg + $format(", cpu %0d", cpu_iid));
            end
        end
        else if (m_cpu_req_load matches tagged Valid .req)
        begin
            oper = tagged LOAD_REQ req;

            let line_addr = toLineAddress(req.physicalAddress);
            debugLog.record(cpu_iid, $format("1: LOAD REQ: PA 0x%h, line 0x%h", req.physicalAddress, line_addr));
        end
        else
        begin
            debugLog.record(cpu_iid, $format("1: Bubble"));
        end

        // If there is a load request then some immediate response
        // is required.  Initialize a retry response by default.
        if (m_cpu_req_load matches tagged Valid .req)
        begin
            local_state.loadRspImm = tagged Valid initICacheRetry(req);
        end

        stage2Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule


    // ====================================================================
    //
    //  Stage 2:  Exactly one stage2 rule must fire for proper pipelining.
    //
    // ====================================================================

    rule stage2_MULTI_LOAD (getOper(stage2Ctrl) matches tagged MULTI_LOAD .miss_tok);
        match {.cpu_iid, {.oper, .local_state}} <- stage2Ctrl.nextReadyInstance();

        // A fill that came in previously is going to multiple miss tokens.
        local_state.missTokToFree = tagged Valid miss_tok;

        // Now send the fill to CPU
        debugLog.record(cpu_iid, $format("2: FILL MULTIPLE RSP LOAD: %0d", miss_tok.index));
        local_state.loadRsp = tagged Valid initICacheMissRsp(miss_tok.index);

        stage3Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule


    rule stage2_FILL_RSP (getOper(stage2Ctrl) matches tagged CACHE_MSG .fill &&&
                          fill.kind matches tagged RSP_LOAD .fill_meta);

        match {.cpu_iid, {.oper, .local_state}} <- stage2Ctrl.nextReadyInstance();

        // Record fill meta data that will be written to the cache.
        local_state.cacheUpdUsed = True;
        local_state.cacheUpdPAddr = fill.linePAddr;
        local_state.cacheUpdState = L1I_STATE_S;

        // Deallocate the Miss ID.
        L1_ICACHE_MISS_TOKEN miss_tok = fromMemOpaque(fill.opaque);

        // Free the token in the next stage, in case we had to retry.
        local_state.missTokToFree = tagged Valid miss_tok;

        // Now send the fill to the CPU
        debugLog.record(cpu_iid, $format("2: MEM RSP LOAD: %0d, line 0x%h", miss_tok.index, fill.linePAddr));
        local_state.loadRsp = tagged Valid initICacheMissRsp(miss_tok.index);

        // Pick the victim
        iCacheAlg.lookupByAddrReq(cpu_iid,
                                  fill.linePAddr,
                                  False,
                                  True);

        stage3Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule


    rule stage2_INVAL_REQ (getOper(stage2Ctrl) matches tagged CACHE_MSG .inval &&&
                           inval.kind matches tagged FORCE_INVAL .inval_meta);

        match {.cpu_iid, {.oper, .local_state}} <- stage2Ctrl.nextReadyInstance();

        debugLog.record(cpu_iid, $format("2: INVAL: line 0x%h", inval.linePAddr));

        // Look up the line
        iCacheAlg.lookupByAddrReq(cpu_iid,
                                  inval.linePAddr,
                                  False,
                                  True);

        stage3Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule


    rule stage2_LOAD_REQ (getOper(stage2Ctrl) matches tagged LOAD_REQ .req);
        match {.cpu_iid, {.oper, .local_state}} <- stage2Ctrl.nextReadyInstance();

        let line_addr = toLineAddress(req.physicalAddress);

        debugLog.record(cpu_iid, $format("2: LOAD REQ: PA 0x%h, line 0x%h", req.physicalAddress, line_addr));

        // Look up the address in the cache.
        iCacheAlg.lookupByAddrReq(cpu_iid, line_addr, True, True);

        stage3Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule


    rule stage2_Invalid (getOper(stage2Ctrl) matches tagged Invalid);
        match {.cpu_iid, {.oper, .local_state}} <- stage2Ctrl.nextReadyInstance();

        debugLog.record(cpu_iid, $format("2: Bubble"));

        stage3Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule


    // ====================================================================
    //
    //  Stage 3:  Exactly one stage3 rule must fire for proper pipelining.
    //
    // ====================================================================

    rule stage3_MULTI_LOAD (getOper(stage3Ctrl) matches tagged MULTI_LOAD .miss_tok);
        match {.cpu_iid, {.oper, .local_state}} <- stage3Ctrl.nextReadyInstance();
        Bool new_miss_tok_req = False;

        debugLog.record(cpu_iid, $format("3: Bubble MULTI_LOAD"));

        stage4Ctrl.ready(cpu_iid, tuple3(oper, local_state, new_miss_tok_req));
    endrule


    rule stage3_FILL_RSP (getOper(stage3Ctrl) matches tagged CACHE_MSG .fill &&&
                          fill.kind matches tagged RSP_LOAD .fill_meta);

        match {.cpu_iid, {.oper, .local_state}} <- stage3Ctrl.nextReadyInstance();
        Bool new_miss_tok_req = False;

        //
        // This protocol evicts at request.  The entry should already
        // be present in the FILL state.
        //
        let entry <- iCacheAlg.lookupByAddrRsp(cpu_iid);

        local_state.cacheUpdIdx = entry.idx;
        outstandingMisses.reportLoadDone(cpu_iid, fill.linePAddr);

        if (entry.state matches tagged Valid .state)
        begin
            Fmt msg = $format("3: FILL: line 0x%h, state ", fill.linePAddr) + fshow(state) + $format(", ") + fshow(entry.idx);
            debugLog.record(cpu_iid, msg);
            assertInFill(state.opaque == L1I_STATE_FILL,
                         msg + $format(", cpu %0d", cpu_iid));
        end
        else
        begin
            // Error:  line should be in the cache already in FILL state.
            Fmt msg = $format("3: ERROR: Filled line not present, line 0x%h", fill.linePAddr);
            debugLog.record(cpu_iid, msg);
            assertInFill(False,
                         msg + $format(", cpu %0d", cpu_iid));
        end

        stage4Ctrl.ready(cpu_iid, tuple3(oper, local_state, new_miss_tok_req));
    endrule


    rule stage3_INVAL_REQ (getOper(stage3Ctrl) matches tagged CACHE_MSG .inval &&&
                           inval.kind matches tagged FORCE_INVAL .inval_meta);

        match {.cpu_iid, {.oper, .local_state}} <- stage3Ctrl.nextReadyInstance();
        Bool new_miss_tok_req = False;

        let entry <- iCacheAlg.lookupByAddrRsp(cpu_iid);

        local_state.cacheUpdIdx = entry.idx;

        if (entry.state matches tagged Valid .state &&&
            state.opaque == L1I_STATE_S)
        begin
            //
            // Line is present and in use.  Invalidate it.
            //
            // Note that the this cache never sends a response to the
            // invalidation request.  This is a simplification of the
            // protocol.  The L1 dcache will send a response and the
            // timing will be almost identical.  Sending a response
            // from the icache would require bookkeeping by the L2
            // to merge icache and dcache responses.  Instead, we use
            // the dcache response in the model as a proxy for both
            // L1 caches.
            //
            local_state.cacheUpdInval = True;
            debugLog.record(cpu_iid, $format("3: INVAL ENTRY: line 0x%h, ", inval.linePAddr) + fshow(entry.idx));
        end
        else
        begin
            //
            // Either the line is not present or it is in the cache
            // but tagged in FILL state.  FILL state is a proxy for
            // a miss address handler.  The fill remains outstanding
            // and will be serviced with the updated value later.
            //
            debugLog.record(cpu_iid, $format("3: INVAL NOT PRESENT: line 0x%h", inval.linePAddr));
        end

        stage4Ctrl.ready(cpu_iid, tuple3(oper, local_state, new_miss_tok_req));
    endrule


    rule stage3_LOAD_REQ (getOper(stage3Ctrl) matches tagged LOAD_REQ .req);
        match {.cpu_iid, {.oper, .local_state}} <- stage3Ctrl.nextReadyInstance();
        Bool new_miss_tok_req = False;

        //
        // Did the load hit in the cache?
        //
        let entry <- iCacheAlg.lookupByAddrRsp(cpu_iid);

        let line_addr = toLineAddress(req.physicalAddress);

        // Is there space in the miss tracker for a new L2 request?
        let can_allocate = outstandingMisses.canAllocateLoad(cpu_iid);

        if (entry.state matches tagged Valid .state)
        begin
            if (state.opaque == L1I_STATE_S)
            begin
                // A hit, so give the data back.
                local_state.loadRspImm = tagged Valid initICacheHit(req);
                statHit.incr(cpu_iid);
                debugLog.record(cpu_iid, $format("3: LOAD HIT: line 0x%h", state.linePAddr));
            end
            else
            begin
                //
                // Fill for this address is already outstanding.  One of
                // two things will now happen:
                //   (1) The miss tracker will merge the new request
                //       with the previous outstanding one.
                //   (2) Failing that, tell the core to retry later.
                //
                if (outstandingMisses.loadOutstanding(cpu_iid, line_addr) &&
                    can_allocate)
                begin
                    // Option 1: Merge with previous fill token.
                    new_miss_tok_req = True;
                    outstandingMisses.allocateLoadReq(cpu_iid, line_addr);

                    statMiss.incr(cpu_iid);
                    debugLog.record(cpu_iid, $format("3: LOAD MISS ALREADY IN FILL (merged): line 0x%h", line_addr));
                end
                else
                begin
                    // Option 2: Retry later.
                    debugLog.record(cpu_iid, $format("3: LOAD HIT IN FILL: line 0x%h (blocking)", state.linePAddr));
                end
            end
        end
        else
        begin
            // A miss.
            if (outstandingMisses.loadOutstanding(cpu_iid, line_addr) &&
                can_allocate)
            begin
                // A fill of this address is already in flight.  Latch
                // on to it and don't generate a new request.
                new_miss_tok_req = True;
                outstandingMisses.allocateLoadReq(cpu_iid, line_addr);

                statMiss.incr(cpu_iid);
                debugLog.record(cpu_iid, $format("3: LOAD MISS (ALREADY OUTSTANDING): line 0x%h", line_addr));
            end
            else if (entry.state matches tagged Blocked)
            begin
                // A miss and no eviction candidate.  Replay.
                debugLog.record(cpu_iid, $format("3: BLOCKED: line 0x%h (blocking)", line_addr));
            end
            else if (can_allocate && local_state.toL2QNotFull[0])
            begin
                // Allocate the next miss ID
                new_miss_tok_req = True;
                outstandingMisses.allocateLoadReq(cpu_iid, line_addr);

                // Record that we are using the memory queue.
                local_state.toL2QUsed[0] = True;

                // Evict now and add the new entry in FILL state.
                // This design embeds miss tracking in the cache itself.
                // (See comment in the MESI protocol README file.)
                local_state.cacheUpdUsed = True;
                local_state.cacheUpdIdx = entry.idx;
                local_state.cacheUpdPAddr = line_addr;
                local_state.cacheUpdState = L1I_STATE_FILL;

                //
                // Send a writeback message on capacity eviction so the L2
                // can add the entry.  The L2 acts as a victim cache in this
                // protocol.
                //
                if (entry.state matches tagged MustEvict .state)
                begin
                    let l2_req = cacheMsg_WBInval(state.linePAddr,
                                                  ?,
                                                  defaultValue);
                    local_state.toL2QData[1] = l2_req;
                    local_state.toL2QUsed[1] = True;

                    debugLog.record(cpu_iid, $format("3: WB: line 0x%h", state.linePAddr));
                end

                statMiss.incr(cpu_iid);
                debugLog.record(cpu_iid, $format("3: LOAD MISS: line 0x%h", line_addr));
            end
            else
            begin
                // Miss, but miss tracker is full or L2 is busy.
                debugLog.record(cpu_iid, $format("3: LOAD RETRY: line 0x%h, can alloc %0d, l2 notFull %0d", line_addr, can_allocate, local_state.toL2QNotFull[0]));
            end
        end

        stage4Ctrl.ready(cpu_iid, tuple3(oper, local_state, new_miss_tok_req));
    endrule


    rule stage3_Invalid (getOper(stage3Ctrl) matches tagged Invalid);
        match {.cpu_iid, {.oper, .local_state}} <- stage3Ctrl.nextReadyInstance();
        Bool new_miss_tok_req = False;

        debugLog.record(cpu_iid, $format("3: Bubble"));

        stage4Ctrl.ready(cpu_iid, tuple3(oper, local_state, new_miss_tok_req));
    endrule


    // ====================================================================
    //
    //  Stage 4:  Not much left to do.  All operations trigger the same
    //            rule.
    //
    // ====================================================================

    rule stage4 (True);
        match {.cpu_iid, {.oper,
                          .local_state,
                          .new_miss_tok_req}} <- stage4Ctrl.nextReadyInstance();

        case (oper) matches
            tagged MULTI_LOAD .miss_tok:
            begin
                debugLog.record(cpu_iid, $format("4: Bubble MULTI_LOAD"));
            end

            tagged CACHE_MSG .msg:
            begin
                debugLog.record(cpu_iid, $format("4: Bubble CACHE_MSG"));
            end

            tagged LOAD_REQ .req:
            begin
                if (new_miss_tok_req)
                begin
                    let miss_tok <- outstandingMisses.allocateLoadRsp(cpu_iid);
                    local_state.loadRspImm = tagged Valid initICacheMiss(req, miss_tok.index);

                    if (! local_state.toL2QUsed[0])
                    begin
                        debugLog.record(cpu_iid, $format("4: LOAD MISS (ALREADY OUTSTANDING): %0d", miss_tok.index));
                    end
                    else
                    begin
                        // Use the opaque bits to store the miss token.
                        let line_addr = toLineAddress(req.physicalAddress);
                        let l2_req = cacheMsg_ReqLoad(line_addr,
                                                      toMemOpaque(miss_tok),
                                                      defaultValue);
                        local_state.toL2QData[0] = l2_req;

                        debugLog.record(cpu_iid, $format("4: LOAD MISS: line 0x%h, tok %0d", line_addr, miss_tok.index));
                    end
                end
                else
                begin
                    debugLog.record(cpu_iid, $format("4: Bubble LOAD_REQ"));
                end
            end

            tagged Invalid:
            begin
                debugLog.record(cpu_iid, $format("4: Bubble"));
            end
        endcase


        // Send immediate response
        loadRspImmToCPU.send(cpu_iid, local_state.loadRspImm);

        // Update retry statistics.
        if (local_state.loadRspImm matches tagged Valid .rsp &&&
            rsp.rspType matches tagged ICACHE_retry)
        begin
            statRetry.incr(cpu_iid);
        end


        stage5Ctrl.ready(cpu_iid, tuple2(oper, local_state));
    endrule


    (* conservative_implicit_conditions *)
    rule stage5 (True);
        match {.cpu_iid, {.oper, .local_state}} <- stage5Ctrl.nextReadyInstance();

        loadRspDelToCPU.send(cpu_iid, local_state.loadRsp);

        if (oper matches tagged CACHE_MSG .msg)
        begin
            portFromL2.doDeq(cpu_iid);
        end
        else
        begin
            portFromL2.noDeq(cpu_iid);
        end

        // Take care of the memory queue.
        for (Integer i = 0; i < 2; i = i + 1)
        begin
            if (local_state.toL2QUsed[i])
            begin
                portToL2[i].doEnq(cpu_iid, local_state.toL2QData[i]);
            end
            else
            begin
                portToL2[i].noEnq(cpu_iid);
            end
        end

        // Take care of the cache update.
        if (local_state.cacheUpdInval)
        begin
            iCacheAlg.invalidate(cpu_iid, local_state.cacheUpdIdx);

            debugLog.record(cpu_iid, $format("5: INVAL: ") + fshow(local_state.cacheUpdIdx));
        end
        else if (local_state.cacheUpdUsed)
        begin
            let state = initCacheEntryState(local_state.cacheUpdPAddr,
                                            False,
                                            local_state.cacheUpdState);

            // Fill allocates a new line.
            if (local_state.cacheUpdState == L1I_STATE_FILL)
            begin
                iCacheAlg.allocate(cpu_iid,
                                   local_state.cacheUpdIdx,
                                   state,
                                   CACHE_ALLOC_FILL);

                debugLog.record(cpu_iid, $format("5: ALLOC: ") + fshow(local_state.cacheUpdIdx) + $format(", ") + fshow(state));
            end
            else
            begin
                iCacheAlg.update(cpu_iid,
                                 local_state.cacheUpdIdx,
                                 state);

                debugLog.record(cpu_iid, $format("5: UPDATE: ") + fshow(local_state.cacheUpdIdx) + $format(", ") + fshow(state));
            end
        end
        
        // Free at the end so we don't reuse token accidentally.
        if (local_state.missTokToFree matches tagged Valid .miss_tok)
        begin
            outstandingMisses.free(cpu_iid, miss_tok);
        end

        debugLog.record(cpu_iid, $format("5: Done"));

        // End of model cycle. (Path 1)
        localCtrl.endModelCycle(cpu_iid, 1); 
        debugLog.nextModelCycle(cpu_iid);
    endrule

endmodule
