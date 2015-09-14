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
import Counter::*;
import FShow::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/fpga_components.bsh"
`include "asim/provides/hasim_model_services.bsh"

`include "asim/provides/pipeline_base_types.bsh"
`include "asim/provides/hasim_issue.bsh"

typedef union tagged 
{
    void ROB_STATE_WRITEBACK_ALU; 
    void ROB_STATE_WRITEBACK_MEM;
    void ROB_STATE_ADD;
    FETCH_BUNDLE ROB_STATE_RESP_DEPENDENCIES;
    void ROB_STATE_COMMIT_REQ;
    void ROB_STATE_COMMIT_RESP;
    void ROB_STATE_ISSUE_REQ;
    void ROB_STATE_ISSUE_RESP;
} 
    ROB_STATE deriving (Bits, Eq);

typedef Bit#(3) TIME_STAMP;
typedef enum {ALU, MEM} WRITE_TYPE deriving (Bits, Eq);

typedef DECODE_BUNDLE ROB_ENTRY;

REWIND_BUNDLE nullRewindBundle = REWIND_BUNDLE{robIndex: 0, mispredict: False, addr: 0, token: ?};

module [HASIM_MODULE] mkIssue();
    TIMEP_DEBUG_FILE                                                              debugLog <- mkTIMEPDebugFile("pipe_rob.out");

    PORT_CREDIT_SEND#(COMMIT_BUNDLE, `COMMIT_NUM, LOG_COMMIT_NUM)               commitPort <- mkPortCreditSend("commit");
    PORT_CREDIT_RECEIVE#(ALU_WRITEBACK_BUNDLE, `ALU_NUM, LOG_ALU_CREDITS) aluWritebackPort <- mkPortCreditReceive("aluWriteback");
    PORT_CREDIT_RECEIVE#(MEM_WRITEBACK_BUNDLE, `MEM_NUM, LOG_MEM_CREDITS) memWritebackPort <- mkPortCreditReceive("memWriteback");
    PORT_CREDIT_RECEIVE#(FETCH_BUNDLE, `DECODE_NUM, LOG_DECODE_CREDITS)        decodePort <- mkPortCreditReceive("decode");
    PORT_CREDIT_SEND#(MEM_BUNDLE, `MEM_NUM, LOG_MEM_CREDITS)                       memPort <- mkPortCreditSend("mem");
    PORT_CREDIT_SEND#(ALU_BUNDLE, `ALU_NUM, LOG_ALU_CREDITS)                       aluPort <- mkPortCreditSend("alu");
    PORT_CREDIT_SEND#(PREDICT_UPDATE_BUNDLE, `ALU_NUM, LOG_ALU_NUM)      predictUpdatePort <- mkPortCreditSend("predictUpdate");

    PORT_CREDIT_SEND#(REWIND_BUNDLE, 1, 1)                                     resteerPort <- mkPortCreditSend("resteer");

    Connection_Send#(CONTROL_MODEL_CYCLE_MSG)                                   modelCycle <- mkConnection_Send("model_cycle");
    Connection_Client#(FUNCP_REQ_GET_DEPENDENCIES, FUNCP_RSP_GET_DEPENDENCIES) getDependencies <- mkConnection_Client("funcp_getDependencies");

    BRAM#(ROB_INDEX, ROB_ENTRY)                                                        rob <- mkBRAM();

    function prfInit(i) = (i < valueOf(TExp#(SizeOf#(ISA_REG_INDEX))));
    Reg#(Vector#(TExp#(SizeOf#(FUNCP_PHYSICAL_REG_INDEX)), Bool))                prfValids <- mkReg(genWith(prfInit));
    Reg#(Vector#(TExp#(SizeOf#(FUNCP_PHYSICAL_REG_INDEX)), TIME_STAMP))     writeTimeStamp <- mkReg(replicate(1));
    Reg#(Vector#(TExp#(SizeOf#(FUNCP_PHYSICAL_REG_INDEX)), Maybe#(WRITE_TYPE)))  writeType <- mkReg(replicate(tagged Valid ALU));
    Reg#(TIME_STAMP)                                                             timeStamp <- mkReg(0);

    Reg#(ROB_STATE)                                                                  state <- mkReg(ROB_STATE_WRITEBACK_ALU);

    Reg#(ROB_PTR)                                                                commitPtr <- mkReg(0);
    Reg#(ROB_PTR)                                                                   addPtr <- mkReg(0);
    Reg#(ROB_PTR)                                                                 issuePtr <- mkReg(0);

    Reg#(Vector#(DECODE_CREDITS, Bool))                                           robValid <- mkReg(replicate(True));
    Reg#(Vector#(DECODE_CREDITS, Bool))                                           robEpoch <- mkReg(replicate(False));
    Reg#(Bool)                                                                 resteerWait <- mkReg(False);

    LUTRAM#(ROB_INDEX, Bool)                                                       robDone <- mkLUTRAMU();
    LUTRAM#(ROB_INDEX, Bool)                                                     robPoison <- mkLUTRAMU();
    LUTRAM#(ROB_INDEX, Bool)                                                     robIssued <- mkLUTRAMU();
    LUTRAM#(ROB_INDEX, Bool)                                                     terminate <- mkLUTRAMU();
    LUTRAM#(ROB_INDEX, Bool)                                                      passFail <- mkLUTRAMU();

    Reg#(REWIND_BUNDLE)                                                       rewindBundle <- mkReg(nullRewindBundle);

    Reg#(Bool)                                                               allPrevIssued <- mkReg(False);
    Reg#(Bool)                                                            allPrevMemIssued <- mkReg(False);

    Vector#(0, INSTANCE_CONTROL_IN#(1)) inports  = newVector();
    Vector#(0, INSTANCE_CONTROL_OUT#(1)) outports = newVector();
    LOCAL_CONTROLLER#(1)                                                       localController <- mkLocalController(inports, outports);

    ROB_INDEX commitIndex = truncate(commitPtr);
    ROB_INDEX addIndex = truncate(addPtr);
    ROB_INDEX issueIndex = truncate(issuePtr);

    ROB_PTR credits = fromInteger(valueOf(DECODE_CREDITS)) - (addPtr - commitPtr);

    function Vector#(DECODE_CREDITS, Bool) markValidOrInvalid(Bool which, Vector#(DECODE_CREDITS, Bool) inVec, ROB_INDEX startIndex);
        Bit#(DECODE_CREDITS) commitMask = (~0) << commitIndex;
        Bit#(DECODE_CREDITS) startMask = (~0) << startIndex;
        Bit#(DECODE_CREDITS) mask = (startIndex < commitIndex)? ((~commitMask) & startMask): ((~commitMask) | startMask);
        Vector#(DECODE_CREDITS, Bool) outVec = newVector();
        for(Integer i = 0; i < valueOf(DECODE_CREDITS); i = i + 1)
            outVec[i] = (mask[i] == 1)? which: inVec[i];
        return outVec;
    endfunction

    function Bool isReady(ROB_ENTRY entry);
        Vector#(ISA_MAX_SRCS, Bool) readys = newVector();
        for(Integer i = 0; i < valueOf(ISA_MAX_SRCS); i = i + 1)
        begin
            if(entry.srcs[i] matches tagged Valid .src)
                readys[i] = (isValid(writeType[src]) && (validValue(writeType[src]) == ALU && (timeStamp - writeTimeStamp[src]) > 1) || prfValids[src]);
            else
                readys[i] = True;
        end
        return fold(\&& , readys) && (!entry.drainBefore || issuePtr == commitPtr);
    endfunction

    function Vector#(TExp#(SizeOf#(FUNCP_PHYSICAL_REG_INDEX)), Bool) markPrfValidOrInvalid(Bool which, Vector#(ISA_MAX_DSTS, Maybe#(FUNCP_PHYSICAL_REG_INDEX)) regs);
        Vector#(TExp#(SizeOf#(FUNCP_PHYSICAL_REG_INDEX)), Bool) newPrfValids = prfValids;
        for(Integer i = 0; i < valueOf(ISA_MAX_DSTS); i = i + 1)
        begin
            if(regs[i] matches tagged Valid .idx)
                newPrfValids[idx] = which;
        end
        return newPrfValids;
    endfunction

    function Action markWriteType(Maybe#(WRITE_TYPE) _writeType, Vector#(ISA_MAX_DSTS, Maybe#(FUNCP_PHYSICAL_REG_INDEX)) regs);
    action
        Vector#(TExp#(SizeOf#(FUNCP_PHYSICAL_REG_INDEX)), Maybe#(WRITE_TYPE)) newWriteType = writeType;
        Vector#(TExp#(SizeOf#(FUNCP_PHYSICAL_REG_INDEX)), TIME_STAMP)    newWriteTimeStamp = writeTimeStamp;
        for(Integer i = 0; i < valueOf(ISA_MAX_DSTS); i = i + 1)
        begin
            if(regs[i] matches tagged Valid .idx)
            begin
                newWriteType[idx] = _writeType;
                newWriteTimeStamp[idx] = timeStamp;
            end
        end
        writeType <= newWriteType;
        writeTimeStamp <= newWriteTimeStamp;
    endaction
    endfunction

    rule writebackAlu(state == ROB_STATE_WRITEBACK_ALU);
        if(aluWritebackPort.canReceive())
        begin
            let bundle <- aluWritebackPort.pop();
            debugLog.record($format("writebackAlu ") + fshow(bundle) + $format(" robValid[%d]: %b", bundle.robIndex, robValid[bundle.robIndex]));
            predictUpdatePort.enq(makePredictUpdateBundle(bundle));
            if(robValid[bundle.robIndex])
            begin
                debugLog.record($format("robValid"));
                prfValids <= markPrfValidOrInvalid(True, bundle.dsts);
                if(bundle.mispredict)
                begin
                    debugLog.record($format("mispredict"));
                    let newRobValid = markValidOrInvalid(False, robValid, bundle.robIndex);
                    newRobValid[bundle.robIndex] = True;
                    let newRobEpoch = markValidOrInvalid(False, robEpoch, bundle.robIndex);
                    newRobEpoch[bundle.robIndex] = True;
                    robValid <= newRobValid;
                    robEpoch <= newRobEpoch;
                    rewindBundle <= makeRewindBundle(bundle);
                    resteerWait <= True;
                end
            end

            terminate.upd(bundle.robIndex, bundle.terminate);
            passFail.upd(bundle.robIndex, bundle.passFail);
            robPoison.upd(bundle.robIndex, tokIsPoisoned(bundle.token));
            robDone.upd(bundle.robIndex, True);
        end
        else
        begin
            predictUpdatePort.done;
            aluWritebackPort.done(`ALU_CREDITS);
            state <= ROB_STATE_WRITEBACK_MEM;
            debugLog.record($format("writebackAlu resteer ") + fshow(rewindBundle));
            resteerPort.send(rewindBundle);
            rewindBundle <= nullRewindBundle;
        end
    endrule

    rule writebackMem(state == ROB_STATE_WRITEBACK_MEM);
        if(memWritebackPort.canReceive())
        begin
            let bundle <- memWritebackPort.pop();
            debugLog.record($format("writebackMem ") + fshow(bundle) + $format(" robValid[%d]: %b", bundle.robIndex, robValid[bundle.robIndex]));
            if(robValid[bundle.robIndex])
                prfValids <= markPrfValidOrInvalid(True, bundle.dsts);

            robPoison.upd(bundle.robIndex, tokIsPoisoned(bundle.token));
            robDone.upd(bundle.robIndex, True);
        end
        else
        begin
            debugLog.record($format("writeback mem done"));
            memWritebackPort.done(`MEM_CREDITS);
            state <= ROB_STATE_ADD;
        end
    endrule

    rule add(state == ROB_STATE_ADD);
        if(decodePort.canReceive())
        begin
            debugLog.record($format("decode port dequeued"));
            let fetch_bundle <- decodePort.pop();
            if(robValid[addIndex] || fetch_bundle.afterResteer && robEpoch[fetch_bundle.epochRob])
            begin
               debugLog.record($format("add dep req: 0x%h", fetch_bundle.pc));
               getDependencies.makeReq(initFuncpReqGetDependencies(0, fetch_bundle.inst, fetch_bundle.pc));
               state <= tagged ROB_STATE_RESP_DEPENDENCIES fetch_bundle;
            end
            else
            begin
                debugLog.record($format("add epoch drop: 0x%h", fetch_bundle.pc));
            end
        end
        else
        begin
            debugLog.record($format("add done"));
            state <= ROB_STATE_COMMIT_REQ;
            decodePort.done(credits);
        end
    endrule

    rule respDependencies(state matches tagged ROB_STATE_RESP_DEPENDENCIES .fetch_bundle);
        let resp = getDependencies.getResp();
        getDependencies.deq();
        let decode_bundle = makeDecodeBundle(resp.token, fetch_bundle, extractPhysReg(resp.srcMap), extractPhysReg(resp.dstMap));
        resteerWait <= False;
        debugLog.record($format("add ") + fshow(decode_bundle) + $format(" addPtr: %d commitPtr: %d credits: %d", addPtr, commitPtr, credits));
        robValid <= markValidOrInvalid(True, robValid, addIndex);
        robIssued.upd(addIndex, False);
        robDone.upd(addIndex, False);
        robPoison.upd(addIndex, False);
        terminate.upd(addIndex, False);
        prfValids <= markPrfValidOrInvalid(False, decode_bundle.dsts);
        rob.write(addIndex, decode_bundle);
        markWriteType(Invalid, decode_bundle.dsts);
        addPtr <= addPtr + 1;
        state <= ROB_STATE_ADD;
    endrule

    rule commitReq(state == ROB_STATE_COMMIT_REQ);
        debugLog.record($format("commitReq: commitPtr: %d robValid: %b robDone: %b robValid[]: %b addPtr: %d credits: %d", commitPtr, robValid, robDone.sub(commitIndex), robValid[commitIndex], addPtr, credits));
        if(!commitPort.canSend() || commitPtr == addPtr || !robDone.sub(commitIndex))
        begin
            debugLog.record($format("commit done"));
            commitPort.done();
            state <= ROB_STATE_ISSUE_REQ;
            issuePtr <= commitPtr;
            allPrevIssued <= True;
            allPrevMemIssued <= True;
        end
        else if(!robValid[commitIndex])
        begin
            debugLog.record($format("robInValid %d", commitIndex));
            commitPtr <= commitPtr + 1;
        end
        else
        begin
            debugLog.record($format("commit req -> resp"));
            rob.readReq(commitIndex);
            state <= ROB_STATE_COMMIT_RESP;
        end
    endrule

    rule commitResp(state == ROB_STATE_COMMIT_RESP);
        let entry <- rob.readRsp();
        debugLog.record($format("commitResp ") + fshow(entry) + $format(" commitPtr: %d robValid: %b addPtr: %d credits: %d", commitPtr, robValid, addPtr, credits));

        if(terminate.sub(commitIndex))
        begin
            debugLog.record($format("ending run"));
            localController.instanceDone(0, passFail.sub(commitIndex));
        end

        if (robPoison.sub(commitIndex))
        begin
            debugLog.record($format("robFault %d", commitIndex));

            let newRobValid = markValidOrInvalid(False, robValid, commitIndex);
            robValid <= newRobValid;

            let newRobEpoch = markValidOrInvalid(False, robEpoch, commitIndex);
            newRobEpoch[commitIndex] = True;
            robEpoch <= newRobEpoch;

            // Set the poison bit in the token sent back to commit.
            // It was never updated in the ROB itself.
            entry.token.poison = True;
            // Send the faulting inst onto commit, 
            // who will handle the fault as a side effecct.
            commitPort.enq(makeCommitBundle(entry, commitIndex));
            resteerWait <= True;
        end
        else
        begin
            commitPort.enq(makeCommitBundle(entry, commitIndex));
            robValid[commitIndex] <= !resteerWait;
        end

        commitPtr <= commitPtr + 1;
        state <= ROB_STATE_COMMIT_REQ;
    endrule

    rule issueReq(state == ROB_STATE_ISSUE_REQ);
        if((!memPort.canSend() && !aluPort.canSend()) || issuePtr == addPtr || !allPrevIssued)
        begin
            debugLog.record($format("end cycle"));
            modelCycle.send(0);
            debugLog.nextModelCycle();
            timeStamp <= timeStamp + 1;
            memPort.done();
            aluPort.done();
            state <= ROB_STATE_WRITEBACK_ALU;
        end
        else if(robIssued.sub(issueIndex))
        begin
            debugLog.record($format("issued"));
            issuePtr <= issuePtr + 1;
        end
        else if(!robValid[issueIndex])
        begin
            debugLog.record($format("issue robInvalid %d", issueIndex));
            robIssued.upd(issueIndex, True);
            robDone.upd(issueIndex, True);
            issuePtr <= issuePtr + 1;
        end
        else
        begin
            debugLog.record($format("issue req -> issue resp"));
            rob.readReq(issueIndex);
            state <= ROB_STATE_ISSUE_RESP;
        end
    endrule

    rule issueResp(state == ROB_STATE_ISSUE_RESP);
        let entry <- rob.readRsp();
        Bool isMem = isaIsStore(entry.inst) || isaIsLoad(entry.inst);
        Bool newAllPrevMemIssued = allPrevMemIssued;
        Bool newAllPrevIssued = True;
        debugLog.record($format("issueResp check issuePtr: %d", issuePtr) + fshow(entry));
        if(isReady(entry))
        begin
            if(isMem)
            begin
                if(allPrevMemIssued && memPort.canSend())
                begin
                    debugLog.record($format("issueResp mem") + fshow(entry));
                    memPort.enq(makeMemBundle(entry, issuePtr));
                    markWriteType(tagged Valid MEM, entry.dsts);
                    robIssued.upd(issueIndex, True);
                end
                else
                begin
                    debugLog.record($format("mem not issued"));
                    newAllPrevMemIssued = False;
                    if(`INORDER_ISSUE == 1)
                        newAllPrevIssued = False;
                end
            end
            else
            begin
                if(aluPort.canSend())
                begin
                    debugLog.record($format("issueResp alu") + fshow(entry));
                    aluPort.enq(makeAluBundle(entry, issuePtr));
                    markWriteType(tagged Valid ALU, entry.dsts);
                    robIssued.upd(issueIndex, True);
                end
                else
                begin
                    debugLog.record($format("alu not issued"));
                    if(`INORDER_ISSUE == 1)
                        newAllPrevIssued = False;
                end
            end
        end
        else
        begin
            if(isMem)
                newAllPrevMemIssued = False;
            if(`INORDER_ISSUE == 1)
                newAllPrevIssued = False;
            debugLog.record($format("entry not ready for issue ") + fshow(entry));
        end
        issuePtr <= issuePtr + 1;
        state <= ROB_STATE_ISSUE_REQ;
        allPrevMemIssued <= newAllPrevMemIssued;
        allPrevIssued <= newAllPrevIssued;
    endrule
endmodule
