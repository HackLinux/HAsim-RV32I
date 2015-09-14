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
import DefaultValue::*;
import FIFO::*;
import FIFOF::*;
import SpecialFIFOs::*;

`include "awb/provides/librl_bsv.bsh"

`include "awb/provides/soft_connections.bsh"
`include "awb/provides/soft_services.bsh"
`include "awb/provides/soft_services_lib.bsh"
`include "awb/provides/soft_services_deps.bsh"
`include "awb/provides/umf.bsh"
`include "awb/provides/mem_services.bsh"
`include "awb/provides/common_services.bsh"
`include "awb/provides/scratchpad_memory_common.bsh"
`include "awb/provides/matrix_multiply_common_params.bsh"
`include "awb/provides/matrix_multiply_common.bsh"

`include "awb/dict/VDEV_SCRATCH.bsh"

typedef enum
{
    STATE_IDLE,
    STATE_TEST,
    STATE_CHECK
}
STATE
    deriving (Bits, Eq);

//
// Implement a heat transfer test
//
module [CONNECTED_MODULE] mkMatrixMultiplyLocal ()
    provisos (Bits#(MEM_ADDRESS, t_MEM_ADDR_SZ),
              Bits#(TEST_DATA, t_MEM_DATA_SZ),
              NumAlias#(TExp#(TMin#(TLog#(BLOCK_SIZE), TLog#(N_LOCAL_MULTIPLIERS))), n),
              Bits#(SCRATCHPAD_MEM_VALUE, t_SCRATCHPAD_MEM_VALUE_SZ),
              //NumAlias#(TMin#(TMul#(t_MEM_DATA_SZ, n), t_SCRATCHPAD_MEM_VALUE_SZ), t_MATRIX_C_DATA_SZ),
              NumAlias#(TExp#(TLog#(TDiv#(TMin#(TMul#(t_MEM_DATA_SZ, n), t_SCRATCHPAD_MEM_VALUE_SZ), t_MEM_DATA_SZ))), n_MATRIX_C_WORD),
              NumAlias#(TMul#(n_MATRIX_C_WORD, t_MEM_DATA_SZ), t_MATRIX_C_DATA_SZ),
              Alias#(Bit#(TSub#(t_MEM_ADDR_SZ, TLog#(n_MATRIX_C_WORD))), t_MATRIX_C_ADDR),
              Alias#(Bit#(t_MATRIX_C_DATA_SZ), t_MATRIX_C_DATA));

    Connection_Receive#(UMF_CHUNK) linkFromHost <- mkConnection_Receive("CPUTOFPGA");
    Connection_Send#(UMF_CHUNK) linkToHost <- mkConnection_Send("FPGATOCPU");    
    DEBUG_FILE debugLog <- mkDebugFile("matrix_multiply_local.out");

    // Output
    STDIO#(Bit#(64)) stdio <- mkStdIO();
    Reg#(STATE) state <- mkReg(STATE_IDLE);
    let msgError <- getGlobalStringUID("matrix_multiply: error: addr: 0x%x, val: 0x%016llx, expected: 0x%016llx\n");

    // Matrix memories
    let matrixAInitFileName <- getGlobalStringUID("matrixA.dat");
    SCRATCHPAD_CONFIG sconfA = defaultValue;
    sconfA.cacheMode = (`MATRIX_MULTIPLY_PVT_CACHE_ENABLE == 1)? SCRATCHPAD_CACHED : SCRATCHPAD_NO_PVT_CACHE;
    sconfA.cacheEntries = 512;
    sconfA.requestMerging = (`MATRIX_MULTIPLY_REQ_MERGE_ENABLE == 1);
    sconfA.debugLogPath = tagged Valid "matrix_A_memory.out";
    sconfA.initFilePath = tagged Valid matrixAInitFileName;
    sconfA.enableStatistics = tagged Valid ("matrix_A_memory_");
    MATRIX_MEMORY_READ_ONLY_IFC#(N_ENGINES, MEM_ADDRESS, TEST_DATA) memoryA <- mkReadOnlyMemWithPrivScratchpad(`VDEV_SCRATCH_MATRIX_A, sconfA);

    let matrixBInitFileName <- getGlobalStringUID("matrixB.dat");
    SCRATCHPAD_CONFIG sconfB = defaultValue;
    sconfB.cacheMode = (`MATRIX_MULTIPLY_PVT_CACHE_ENABLE == 1)? SCRATCHPAD_CACHED : SCRATCHPAD_NO_PVT_CACHE;
    sconfB.requestMerging = (`MATRIX_MULTIPLY_REQ_MERGE_ENABLE == 1);
    sconfB.debugLogPath = tagged Valid "matrix_B_memory.out";
    sconfB.initFilePath = tagged Valid matrixBInitFileName;
    sconfB.enableStatistics = tagged Valid ("matrix_B_memory_");
    MATRIX_MEMORY_READ_ONLY_IFC#(N_ENGINES, MEM_ADDRESS, TEST_DATA) memoryB <- mkReadOnlyMemWithPrivScratchpad(`VDEV_SCRATCH_MATRIX_B, sconfB);
    
    SCRATCHPAD_CONFIG sconfC = defaultValue;
    sconfC.cacheMode = SCRATCHPAD_UNCACHED;
    sconfC.debugLogPath = tagged Valid "matrix_C_memory.out";
    //sconfC.requestMerging = (`MATRIX_MULTIPLY_REQ_MERGE_ENABLE == 1);
    sconfC.requestMerging = False;
    
`ifndef MATRIX_MULTIPLY_RESULT_CHECK_Z
    MATRIX_MEMORY_ONE_READER_MULTI_WRITER_IFC#(N_ENGINES, t_MATRIX_C_ADDR, t_MATRIX_C_DATA) memoryC <- mkMultiWriterMemWithPrivScratchpad(`VDEV_SCRATCH_MATRIX_C, sconfC);
    Reg#(Bool) checkReqIssueDone  <- mkReg(False);
    Reg#(t_MATRIX_C_ADDR) maxAddr <- mkReg(0);
    let matrixGoldenCInitFileName <- getGlobalStringUID("matrixC.dat");
    SCRATCHPAD_CONFIG sconfGoldC = defaultValue;
    sconfGoldC.initFilePath = tagged Valid matrixGoldenCInitFileName;
    sconfGoldC.debugLogPath = tagged Valid "matrix_C_memory_golden.out";
    MEMORY_IFC#(t_MATRIX_C_ADDR, t_MATRIX_C_DATA) memoryGoldenC <- mkScratchpad(`VDEV_SCRATCH_MATRIX_GOLD, sconfGoldC);
`else
    MATRIX_MEMORY_WRITE_ONLY_IFC#(N_ENGINES, t_MATRIX_C_ADDR, t_MATRIX_C_DATA) memoryC <- mkWriteOnlyMemWithPrivScratchpad(`VDEV_SCRATCH_MATRIX_C, sconfC);
`endif

    // Processing Engines and command fifos
    Vector#(N_ENGINES, MATRIX_MULTIPLY_ENGINE_IFC) engines = newVector();
    for (Integer p = 0; p < valueOf(N_ENGINES); p = p + 1)
    begin
        engines[p] <- mkMatrixMultiplyEngine(memoryA.readPorts[p], memoryB.readPorts[p], memoryC.writePorts[p], p);
    end
    
    Reg#(CYCLE_COUNTER) cycleCnt       <- mkReg(0);
    Reg#(CYCLE_COUNTER) initCycleCnt   <- mkReg(0);
    Reg#(Bool)      receivedLastInst   <- mkReg(False);

    (* fire_when_enabled *)
    rule countCycle(True);
        cycleCnt <= cycleCnt + 1;
    endrule

    rule receiveHostInitReq (state == STATE_IDLE);
        MATRIX_MULTIPLY_INSTRUCTION m = unpack(truncate(pack(linkFromHost.receive())));
        if (m matches tagged INIT_INST .inst)
        begin
            debugLog.record($format("receiveReqFromHost: Init Instruction: matrixA row size: %d, matrix B row size: %d, blockSize: %d, engineNum: %d", 
                            pack(1)<<(inst.matrixSizeA), pack(1)<<(inst.matrixSizeB), pack(1)<<(inst.blockSize), inst.engineNum));
            state <= STATE_TEST;
            initCycleCnt <= cycleCnt;
            for (Integer p = 0; p < valueOf(N_ENGINES); p = p + 1)
            begin
                if (fromInteger(p) < inst.engineNum)
                begin
                    engines[p].setMatrixRowSize(truncate(inst.matrixSizeA), truncate(inst.matrixSizeB));
                    engines[p].setBlockSize(truncate(inst.blockSize));
                end
            end
        end
`ifndef MATRIX_MULTIPLY_RESULT_CHECK_Z
        else if (m matches tagged CHECK_INST .e)
        begin
            state <= STATE_CHECK;
            MEM_ADDRESS max_addr = resize((1 << (e.logElement-fromInteger(valueOf(TLog#(n_MATRIX_C_WORD)))))-1);
            debugLog.record($format("receiveReqFromHost: Check Instruction: # elements=0x%x", max_addr));
            maxAddr <= truncate(max_addr);
            checkReqIssueDone <= False;
        end
`endif        
        linkFromHost.deq();
    endrule

    rule receiveHostComputeReq (state == STATE_TEST && !receivedLastInst);
        MATRIX_MULTIPLY_INSTRUCTION m = unpack(truncate(pack(linkFromHost.receive())));
        if (m matches tagged COMPUTE_INST .inst)
        begin
            debugLog.record($format("receiveReqFromHost: Compute Instruction: engine ID: %d", inst.engineID));
            debugLog.record($format("receiveReqFromHost: matrixA: start x: 0x%x, start y: 0x%x,  matrixB: start x: 0x%x, start y: 0x%x", 
                            inst.startAx, inst.startAy, inst.startBx, inst.startBy));
            MATRIX_MULTIPLY_BLOCK_INFO info = unpack(truncate(inst.blockInfo));
            debugLog.record($format("receiveReqFromHost: isFirstBlock=%s, isLastBlock=%s, needWriteBack=%s, newBlock=%s, lastInst=%s", 
                            info.isFirstBlock? "True" : "False", info.isLastBlock? "True" : "False", 
                            info.needWriteBack? "True" : "False", info.newBlock? "True" : "False", info.isLastInst? "True" : "False"));
            engines[inst.engineID].setStartAddr(truncate(inst.startAx), truncate(inst.startAy), truncate(inst.startBx), truncate(inst.startBy),
                                                          info.isFirstBlock, info.isLastBlock, info.needWriteBack, info.newBlock);
            if (info.isLastInst)
            begin
                receivedLastInst <= True;
            end
        end
        linkFromHost.deq();
    endrule

    function Bool checkDone (x);
        return x.done();
    endfunction

    rule waitForAllEngineDone (state == STATE_TEST && receivedLastInst);
        Vector#(N_ENGINES, Bool) doneSignals = map(checkDone, engines);
        if (fold(\&& ,doneSignals)) // all ones
        begin
            state <= STATE_IDLE;
            receivedLastInst <= False;
            let test_cycle = cycleCnt-initCycleCnt;
            linkToHost.send(zeroExtend(test_cycle));
            debugLog.record($format("waitForAllEngineDone: test done, cycle=%0d", test_cycle));
        end
    endrule

`ifndef MATRIX_MULTIPLY_RESULT_CHECK_Z
    Reg#(t_MATRIX_C_ADDR)  errorCnt  <- mkReg(0);
    Reg#(t_MATRIX_C_ADDR)  checkCnt  <- mkReg(0);
    FIFO#(t_MATRIX_C_ADDR) checkReqQ <- mkSizedFIFO(32);
    rule issueCheckReq (state == STATE_CHECK && !checkReqIssueDone);
        memoryC.readPort.readReq(checkCnt);
        memoryGoldenC.readReq(checkCnt);
        checkReqQ.enq(checkCnt);
        if (checkCnt == maxAddr)
        begin
           checkCnt <= 0;
           checkReqIssueDone <= True;
        end
        else
        begin
           checkCnt <= checkCnt + 1;
        end
    endrule
    rule compareMatrixC (state == STATE_CHECK);
        let r1 <- memoryC.readPort.readRsp();
        let r2 <- memoryGoldenC.readRsp();
        let addr = checkReqQ.first();
        checkReqQ.deq();
        let new_error_cnt = errorCnt;
        if (pack(r1) == pack(r2))
        begin
            debugLog.record($format("compareMatrixC: correct: addr=0x%x, data=0x%x", addr, r1)); 
        end
        else
        begin
            new_error_cnt = errorCnt + 1;
            debugLog.record($format("compareMatrixC: error: addr=0x%x, data=0x%x, expected=0x%x", addr, r1, r2)); 
            stdio.printf(msgError, list3(zeroExtend(addr), zeroExtend(r1), zeroExtend(r2)));
        end
        if (addr == maxAddr)
        begin
            state <= STATE_IDLE;
            errorCnt <= 0;
            debugLog.record($format("compareMatrixC: Done, errorCnt=%0d", new_error_cnt));
            linkToHost.send(zeroExtend(new_error_cnt));
        end
        else
        begin
            errorCnt <= new_error_cnt;
        end
    endrule
`endif

endmodule

