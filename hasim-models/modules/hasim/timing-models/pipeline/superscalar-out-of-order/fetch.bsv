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

import FShow::*;
import Vector::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/hasim_model_services.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/model_structures_base_types.bsh"
`include "asim/provides/hasim_branch_pred_alg.bsh"
`include "asim/provides/funcp_simulated_memory.bsh"

`include "asim/provides/pipeline_base_types.bsh"

typedef enum
{
    FETCH_STATE_PREDICT_UPDATE,
    FETCH_STATE_FAULT_RESP,
    FETCH_STATE_REWIND_RESP,
    FETCH_STATE_I_TRANSLATE_REQ,
    FETCH_STATE_INST_REQ,
    FETCH_STATE_INST_RESP,
    FETCH_STATE_BRANCH_IMM,
    FETCH_STATE_JUMP_IMM
}
FETCH_STATE
    deriving (Bits, Eq);

module [HASIM_MODULE] mkFetch();
    TIMEP_DEBUG_FILE                                                               debugLog <- mkTIMEPDebugFile("pipe_fet.out");

    PORT_CREDIT_SEND#(FETCH_BUNDLE, `FETCH_NUM, LOG_FETCH_CREDITS)                fetchPort <- mkPortCreditSend("fetch");
    PORT_NO_STALL_RECEIVE#(PREDICT_UPDATE_BUNDLE, `ALU_NUM)               predictUpdatePort <- mkPortNoStallReceive("predictUpdate");
    PORT_NO_STALL_RECEIVE#(REWIND_BUNDLE, 1)                                    resteerPort <- mkPortNoStallReceive("resteer");
    PORT_NO_STALL_RECEIVE#(FAULT_BUNDLE, 1)                                       faultPort <- mkPortNoStallReceive("fault");

    Connection_Client#(Maybe#(FUNCP_REQ_DO_ITRANSLATE), FUNCP_RSP_DO_ITRANSLATE) iTranslate <- mkConnection_Client("funcp_doITranslate");
    Connection_Client#(FUNCP_REQ_GET_INSTRUCTION, FUNCP_RSP_GET_INSTRUCTION) getInstruction <- mkConnection_Client("funcp_getInstruction");
    Connection_Client#(FUNCP_REQ_REWIND_TO_TOKEN, FUNCP_RSP_REWIND_TO_TOKEN)  rewindToToken <- mkConnection_Client("funcp_rewindToToken");

    Reg#(ISA_ADDRESS)                                                                    pc <- mkReg(`PROGRAM_START_ADDR);
    Reg#(FETCH_STATE)                                                                 state <- mkReg(FETCH_STATE_PREDICT_UPDATE);
    Reg#(ROB_INDEX)                                                                epochRob <- mkReg(0);
    Reg#(Bool)                                                                 afterResteer <- mkReg(False);

    BRANCH_PREDICTOR_ALG                                                         branchPred <- mkBranchPredAlg;

    //
    // Local Controller
    //
    // FIXME -- need to enumerate ports so balancing works for events
    Vector#(0, INSTANCE_CONTROL_IN#(1)) inports  = newVector();
    Vector#(0, INSTANCE_CONTROL_OUT#(1)) outports = newVector();

    LOCAL_CONTROLLER#(1) localCtrl <- mkLocalController(inports, outports);


    function Action makeFetchBundle(ISA_INSTRUCTION inst, ISA_ADDRESS _pc, PRED_TYPE predType, Bool prediction, ISA_ADDRESS predPc);
    action
        getInstruction.deq;
        let bundle = FETCH_BUNDLE{inst: inst, pc: _pc, predType: predType, prediction: prediction, predPc: predPc, epochRob: epochRob, afterResteer: afterResteer};
        fetchPort.enq(bundle);
        debugLog.record($format("instResp ") + fshow(bundle));
        afterResteer <= False;
        pc <= predPc;
        state <= FETCH_STATE_I_TRANSLATE_REQ;
    endaction
    endfunction

    rule predictUpdate(state == FETCH_STATE_PREDICT_UPDATE);
        let dummy <- localCtrl.startModelCycle();
        localCtrl.endModelCycle(dummy, 1);

        if(predictUpdatePort.canReceive)
        begin
            let bundle <- predictUpdatePort.pop;
            debugLog.record($format("predict update received") + fshow(bundle));
            if(bundle.predType == PRED_TYPE_BRANCH_IMM)
            begin
                debugLog.record($format("Branch Imm upd ") + fshow(bundle));
                branchPred.upd(0,
                               bundle.pc,
                               bundle.pred == bundle.actual,
                               bundle.actual);
            end
        end
        else
        begin
            //
            // End of cycle.  Read fault and resteer from ROB.
            //
            predictUpdatePort.done;
            let fault <- faultPort.receive();
            let resteer <- resteerPort.receive();
            
            if (fault.fault)
            begin
                // Fault.  Invoke the functional fault handler.  The fault handler
                // rewinds instructions and returns the next PC.
                debugLog.record($format("faultReq ") + fshow(fault) + $format(" resteer to 0x%0x", fault.nextInstructionAddress));
                epochRob <= fault.robIndex;
                afterResteer <= True;
                pc <= fault.nextInstructionAddress;
                rewindToToken.makeReq(initFuncpReqRewindToToken(fault.token));
                state <= FETCH_STATE_REWIND_RESP;
                
            end
            else if (resteer.mispredict)
            begin
                // Branch misprediction.  Rewind and resteer.
                debugLog.record($format("rewindReq ") + fshow(resteer));
                pc <= resteer.addr;
                epochRob <= resteer.robIndex;
                afterResteer <= True;
                rewindToToken.makeReq(initFuncpReqRewindToToken(resteer.token));
                state <= FETCH_STATE_REWIND_RESP;
            end
            else
            begin
                debugLog.record($format("no predict update or resteer"));
                state <= FETCH_STATE_I_TRANSLATE_REQ;
            end
        end
    endrule

    rule rewindResp(state == FETCH_STATE_REWIND_RESP);
        debugLog.record($format("rewindResp "));
        rewindToToken.deq();
        state <= FETCH_STATE_I_TRANSLATE_REQ;
    endrule

    rule iTranslateReq(state == FETCH_STATE_I_TRANSLATE_REQ);
        if(fetchPort.canSend())
        begin
            debugLog.record($format("iTranslate req"));
            iTranslate.makeReq(tagged Valid FUNCP_REQ_DO_ITRANSLATE{contextId: 0, virtualAddress: pc});
            state <= FETCH_STATE_INST_REQ;
        end
        else
        begin
            debugLog.record($format("end cycle"));
            debugLog.nextModelCycle();
            state <= FETCH_STATE_PREDICT_UPDATE;
            fetchPort.done();
        end
    endrule

    rule instReq(state == FETCH_STATE_INST_REQ);
        debugLog.record($format("iTranslate resp"));
        let resp = iTranslate.getResp();
        iTranslate.deq();

        // iTranslate may return multiple responses for unaligned references.
        // Don't act until the last one is received.
        if (! resp.hasMore)
        begin
            debugLog.record($format("inst req"));
            getInstruction.makeReq(FUNCP_REQ_GET_INSTRUCTION{contextId: 0, physicalAddress: resp.physicalAddress, offset: resp.offset, hasMore: False});
            state <= FETCH_STATE_INST_RESP;
        end
    endrule

    rule instResp(state == FETCH_STATE_INST_RESP);
        let resp = getInstruction.getResp;
        debugLog.record($format("inst resp"));
        if(isBranchImm(resp.instruction))
        begin
            branchPred.getPredReq(0, pc);
            debugLog.record($format("Branch Imm"));
            state <= FETCH_STATE_BRANCH_IMM;
        end
        else if(isJumpImm(resp.instruction))
        begin
            debugLog.record($format("Jump Imm"));
            state <= FETCH_STATE_JUMP_IMM;
        end
        else
            makeFetchBundle(resp.instruction, pc, PRED_TYPE_NONE, False, pc + 4);
    endrule

    rule branchImm(state == FETCH_STATE_BRANCH_IMM);
        debugLog.record($format("branch imm resp"));
        let resp = getInstruction.getResp;
        let pred <- branchPred.getPredRsp(0);
        let predPc = pred? predPcBranchImm(pc, resp.instruction): pc + 4;
        makeFetchBundle(resp.instruction, pc, PRED_TYPE_BRANCH_IMM, pred, predPc);
    endrule

    rule jumpImm(state == FETCH_STATE_JUMP_IMM);
        debugLog.record($format("jump imm resp"));
        let resp = getInstruction.getResp;
        let predPc = predPcJumpImm(pc, resp.instruction);
        makeFetchBundle(resp.instruction, pc, PRED_TYPE_JUMP_IMM, False, predPc);
    endrule
endmodule
