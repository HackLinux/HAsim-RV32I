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
import FIFO::*;
import FIFOF::*;
import FIFOLevel::*;
import SpecialFIFOs::*;
import Vector::*;
import DefaultValue::*;
import ConfigReg::* ;

`include "awb/provides/librl_bsv.bsh"
`include "awb/provides/fpga_components.bsh"
`include "awb/provides/soft_connections.bsh"
`include "awb/provides/soft_services.bsh"
`include "awb/provides/soft_services_lib.bsh"
`include "awb/provides/soft_services_deps.bsh"
`include "awb/provides/mem_services.bsh"
`include "awb/provides/common_services.bsh"
`include "awb/provides/matrix_multiply_common_params.bsh"
`include "awb/provides/matrix_multiply_common.bsh"


interface MATRIX_MULTIPLY_ENGINE_IFC;
    method Action setMatrixRowSize(MATRIX_SIZE_BITS matrixAsizeBits, MATRIX_SIZE_BITS  matrixBsizeBits);
    method Action setBlockSize(BLOCK_SIZE_BITS sizeBits);
    method Action setStartAddr(MATRIX_ADDR_X_MAX matrixAx, MATRIX_ADDR_Y_MAX matrixAy, MATRIX_ADDR_X_MAX matrixBx, MATRIX_ADDR_Y_MAX matrixBy, Bool isFirstBlock, Bool isLastBlock, Bool needWriteBack, Bool newBlockC);
    method Bool notBusy();
    method Bool done();
endinterface

interface MATRIX_LOAD_MANAGER#(type t_BUFFER_DATA);
    method Action setMatrixRowSize(MATRIX_SIZE_BITS sizeBits);
    method Action setBlockSize(BLOCK_SIZE_BITS sizeBits);
    method Action setStartAddr(MATRIX_ADDR_X_MAX addrX, MATRIX_ADDR_Y_MAX addrY, Bool isLastBlock);
    method ActionValue#(t_BUFFER_DATA) getDataFromMem();
    method t_BUFFER_DATA peekDataFromMem();
    method Action putDataIntoBuffer(t_BUFFER_DATA data);
    method ActionValue#(t_BUFFER_DATA) getDataFromBuffer();
endinterface

interface MATRIX_STORE_MANAGER#(type t_BUFFER_DATA);
    method Action setMatrixRowSize(MATRIX_SIZE_BITS sizeBits);
    method Action setBlockSize(BLOCK_SIZE_BITS sizeBits);
    method Action setStartAddr(MATRIX_ADDR_X_MAX addrX, MATRIX_ADDR_Y_MAX addrY, Bool isLastBlock);
    method Action readBankReq(Bit#(TMul#(TLog#(BLOCK_SIZE),2)) addr);
    method Action writeBankReq(Bit#(TMul#(TLog#(BLOCK_SIZE),2)) addr, t_BUFFER_DATA data);
    method Action bankCompleted(Bit#(TMul#(TLog#(BLOCK_SIZE),2)) addr);
    method ActionValue#(t_BUFFER_DATA) readBankResp();
    method Bool notBusy();
endinterface

typedef enum
{
    STATE_IDLE,
    STATE_READ,
    STATE_READ_DONE,
    STATE_READ_WAIT
}
MATRIX_LOAD_BUFFER_MANAGER_STATE
    deriving (Bits, Eq);

module [CONNECTED_MODULE] mkBufferManagerA#(MEMORY_READER_IFC#(t_ADDR, t_DATA) mem,
                                            NumTypeParam#(n_DEPTH) bufferDepth, 
                                            DEBUG_FILE debugLog)
    // interface:
    (MATRIX_LOAD_MANAGER#(t_DATA))
    provisos (Bits#(t_ADDR, t_ADDR_SZ),
              Bits#(t_DATA, t_DATA_SZ),
              NumAlias#(TLog#(MATRIX_X_MAX), t_ADDR_MAX_X_SZ),
              NumAlias#(TLog#(MATRIX_Y_MAX), t_ADDR_MAX_Y_SZ),
              Add#(TAdd#(t_ADDR_MAX_X_SZ, t_ADDR_MAX_Y_SZ), extraBits, t_ADDR_SZ),
              Alias#(Bit#(TMul#(TLog#(BLOCK_SIZE), 2)), t_BLOCK_ADDR),
              Alias#(Bit#(t_ADDR_MAX_X_SZ), t_ADDR_MAX_X),
              Alias#(Bit#(t_ADDR_MAX_Y_SZ), t_ADDR_MAX_Y));

    //Vector#(2, FIFOF#(t_DATA)) dataBuffers <- replicateM(mkSizedAutoMemFIFOF(valueOf(n_DEPTH), defaultValue));
`ifndef MATRIX_MULTIPLY_MATRIX_A_Z_SHAPE_ACCESS_Z
    Vector#(2, FIFOF#(t_DATA)) dataBuffers <- replicateM(mkSizedBRAMFIFOF(valueOf(n_DEPTH)));
`else
    FIFOF#(t_DATA) dataBuffer <- mkSizedBRAMFIFOF(valueOf(n_DEPTH));
`endif


    Reg#(MATRIX_LOAD_BUFFER_MANAGER_STATE) state <- mkReg(STATE_IDLE);

    Reg#(t_ADDR_MAX_X) matrixStartX    <- mkReg(0);
    Reg#(t_ADDR_MAX_Y) matrixStartY    <- mkReg(0);
    Reg#(t_ADDR_MAX_X) matrixEndX      <- mkReg(0);
    Reg#(t_ADDR_MAX_Y) matrixEndY      <- mkReg(0);
    Reg#(Bool)         readLastBlock   <- mkReg(False);

    Reg#(MATRIX_SIZE_BITS) rowSizeBits    <- mkReg(0);
    Reg#(t_BLOCK_ADDR)     maxElementIdx  <- mkReg(0);
    Reg#(t_BLOCK_ADDR)     readRespIdx    <- mkReg(0);
    Wire#(BLOCK_SIZE_BITS) blockInitCmdW  <- mkWire();
    Wire#(Tuple3#(t_ADDR_MAX_X, t_ADDR_MAX_Y, Bool)) startCmdW <- mkWire();

    Reg#(Bit#(TLog#(BLOCK_SIZE))) maxRowIdx       <- mkReg(0); 
    
`ifndef MATRIX_MULTIPLY_MATRIX_A_Z_SHAPE_ACCESS_Z    
    Reg#(Bit#(TLog#(BLOCK_SIZE))) bufferDeqRowIdx <- mkReg(0); 
    Reg#(Bit#(1)) bufferBankIdx                   <- mkReg(0);
`endif

    function t_ADDR calMemAddr(t_ADDR_MAX_X rx, t_ADDR_MAX_Y ry);
        Bit#(TAdd#(TAdd#(t_ADDR_MAX_X_SZ, 1), t_ADDR_MAX_Y_SZ)) addr = (zeroExtend(ry) << rowSizeBits) + zeroExtend(rx); 
        return unpack(resize(addr));
    endfunction
    
    Reg#(Bool) readReqPhase        <- mkReg(False);
    Reg#(t_ADDR_MAX_X) matrixAddrX <- mkReg(0);
    Reg#(t_ADDR_MAX_Y) matrixAddrY <- mkReg(0);
    
    PulseWire readMatrixReqDoneW  <- mkPulseWire();
    PulseWire readMatrixRespDoneW <- mkPulseWire();
    PulseWire readStartW          <- mkPulseWire();

    (* fire_when_enabled*)
    rule changeReadState (readMatrixReqDoneW || readMatrixRespDoneW || readStartW);
        if (readStartW)
        begin
            state <= STATE_READ;
        end
        else if (readMatrixRespDoneW && state == STATE_READ_WAIT) // response done with the last block
        begin
            state <= STATE_IDLE;
        end
        else if (readMatrixReqDoneW) // read request done with the current block
        begin
            state <= (readLastBlock)? STATE_READ_WAIT: STATE_READ_DONE;
        end
    endrule
        
    (* fire_when_enabled*)
    rule startNewBlock (readStartW);
        match {.addr_x, .addr_y, .is_last} = startCmdW;
        matrixStartX <= addr_x;
        matrixStartY <= addr_y;
        matrixAddrX  <= addr_x;
        matrixAddrY  <= addr_y;
        
        // end addresses
        let end_x = pack(addr_x) + zeroExtendNP(maxRowIdx); 
        let end_y = pack(addr_y) + zeroExtendNP(maxRowIdx); 
        matrixEndX <= end_x; 
        matrixEndY <= end_y;
        readLastBlock <= is_last;
        
        debugLog.record($format("setStartAddr: matrix A: x=0x%x, y=0x%x, end addr: x=0x%x, y=0x%x", 
                        addr_x, addr_y, end_x, end_y));
    endrule
    
    (* fire_when_enabled*)
    rule initBLockSize (True);
        let size_bits = blockInitCmdW;
        Bit#(TAdd#(TLog#(TMul#(TLog#(BLOCK_SIZE),2)),1)) element_bits = resize(pack(size_bits)) << 1;
        Bit#(TAdd#(TMul#(TLog#(BLOCK_SIZE),2),1)) total_element = 1 << element_bits;
        Bit#(TAdd#(TLog#(BLOCK_SIZE),1)) total_row = 1 << size_bits;
        maxElementIdx <= truncate(total_element-1);
        maxRowIdx <= truncate(total_row -1);
        debugLog.record($format("setBlockSize: matrix A: size bits=0x%x, total_row=0x%x, total_element=0x%x", 
                        size_bits, total_row, total_element));
    endrule

`ifndef MATRIX_MULTIPLY_MATRIX_A_Z_SHAPE_ACCESS_Z
    (* mutually_exclusive = "startNewBlock, readMatrixPhase1, readMatrixPhase2" *)
    rule readMatrixPhase1 (state == STATE_READ && !readReqPhase);
        let addr = calMemAddr(matrixAddrX, matrixAddrY);
        mem.readReq(addr);
        readReqPhase <= !readReqPhase;
        debugLog.record($format("readMatrixA: phase1: x=0x%x, y=0x%x, addr=0x%x", 
                        matrixAddrX, matrixAddrY, addr));
    endrule

    // optimization (underlying scratchpad data size is twice the data size)
    rule readMatrixPhase2 (state == STATE_READ && readReqPhase);
        let addr = calMemAddr(matrixAddrX+1, matrixAddrY);
        mem.readReq(addr);
        if (matrixAddrY == matrixEndY)
        begin
            if (matrixAddrX == (matrixEndX - 1))
            begin
                readMatrixReqDoneW.send();
                debugLog.record($format("readMatrixA: memRead issue done"));
            end
            else
            begin
                matrixAddrY <= matrixStartY;
                matrixAddrX <= matrixAddrX + 2;
            end
        end
        else 
        begin
            matrixAddrY <= matrixAddrY + 1;
        end
        readReqPhase <= !readReqPhase;
        debugLog.record($format("readMatrixA: phase2: x=0x%x, y=0x%x, addr=0x%x", 
                        matrixAddrX+1, matrixAddrY, addr));
    endrule
`else
    (* mutually_exclusive = "startNewBlock, readMatrixPhase" *)
    rule readMatrixPhase (state == STATE_READ);
        let addr = calMemAddr(matrixAddrX, matrixAddrY);
        mem.readReq(addr);
        debugLog.record($format("readMatrixA: x=0x%x, y=0x%x, addr=0x%x", matrixAddrX, matrixAddrY, addr));
        if (matrixAddrY == matrixEndY)
        begin
            if (matrixAddrX == matrixEndX)
            begin
                readMatrixReqDoneW.send();
                debugLog.record($format("readMatrixA: memRead issue done"));
            end
            else
            begin
                matrixAddrY <= matrixStartY;
                matrixAddrX <= matrixAddrX + 1;
            end
        end
        else 
        begin
            matrixAddrY <= matrixAddrY + 1;
        end
    endrule
`endif


    rule recvDataFromMem (state != STATE_IDLE);
        let resp <- mem.readRsp();
`ifndef MATRIX_MULTIPLY_MATRIX_A_Z_SHAPE_ACCESS_Z        
        dataBuffers[readRespIdx[0]].enq(resp);
        debugLog.record($format("recvDataFromMatrixA: data=0x%x, bank_idx=%d", resp, readRespIdx[0]));
`else
        dataBuffer.enq(resp);
        debugLog.record($format("recvDataFromMatrixA: data=0x%x", resp));
`endif
        if (readRespIdx == maxElementIdx)
        begin
            readRespIdx <= 0;
            readMatrixRespDoneW.send();
            debugLog.record($format("readMatrixA: memRead resp done"));
        end
        else
        begin
            readRespIdx <= readRespIdx + 1;
        end
    endrule
    
    // =======================================================================
    //
    // Methods
    //
    // =======================================================================
    
    method Action setMatrixRowSize(MATRIX_SIZE_BITS sizeBits) if (state == STATE_IDLE);
        rowSizeBits <= sizeBits;
        debugLog.record($format("setMatrixRowSize: matrix A: row size bits=0x%x", sizeBits));
    endmethod
   
    method Action setBlockSize(BLOCK_SIZE_BITS sizeBits) if (state == STATE_IDLE);
        blockInitCmdW <= sizeBits;
    endmethod

    method Action setStartAddr(t_ADDR_MAX_X addrX, t_ADDR_MAX_Y addrY, Bool isLastBlock) if (state == STATE_IDLE || state == STATE_READ_DONE);
        readStartW.send();
        startCmdW <= tuple3(addrX, addrY, isLastBlock); 
    endmethod
    
    method ActionValue#(t_DATA) getDataFromMem();
`ifndef MATRIX_MULTIPLY_MATRIX_A_Z_SHAPE_ACCESS_Z    
        let r = dataBuffers[bufferBankIdx].first();
        dataBuffers[bufferBankIdx].deq();
        if (bufferDeqRowIdx == maxRowIdx)
        begin
            bufferBankIdx <= bufferBankIdx + 1;
            bufferDeqRowIdx <= 0;
        end
        else
        begin
            bufferDeqRowIdx <= bufferDeqRowIdx + 1;
        end
        debugLog.record($format("getData from memory buffer A: bank_idx=0x%x, row_idx=0x%x, data=0x%x", 
                       bufferBankIdx, bufferDeqRowIdx, r));
`else
        let r = dataBuffer.first();
        dataBuffer.deq();
        debugLog.record($format("getData from memory buffer A: data=0x%x", r));
`endif
        return r;  
    endmethod

    method t_DATA peekDataFromMem();
`ifndef MATRIX_MULTIPLY_MATRIX_A_Z_SHAPE_ACCESS_Z    
        return dataBuffers[bufferBankIdx].first();
`else
        return dataBuffer.first();
`endif
    endmethod

    method Action putDataIntoBuffer(t_DATA data);
        noAction;
    endmethod
    
    method ActionValue#(t_DATA) getDataFromBuffer();
        return ?;
    endmethod

endmodule


module [CONNECTED_MODULE] mkBufferManagerB#(MEMORY_READER_IFC#(t_ADDR, t_MEM_DATA) mem,
                                            NumTypeParam#(n_DEPTH) bufferDepth, 
                                            DEBUG_FILE debugLog)
    // interface:
    (MATRIX_LOAD_MANAGER#(t_BUFFER_DATA))
    provisos (Bits#(t_ADDR, t_ADDR_SZ),
              Bits#(t_MEM_DATA, t_MEM_DATA_SZ),
              Bits#(t_BUFFER_DATA, t_BUFFER_DATA_SZ),
              NumAlias#(TDiv#(t_BUFFER_DATA_SZ, t_MEM_DATA_SZ), n_BANKS),
              NumAlias#(TDiv#(BLOCK_SIZE, n_BANKS), n_LOCAL_BUF_DEPTH),
              NumAlias#(TLog#(MATRIX_X_MAX), t_ADDR_MAX_X_SZ),
              NumAlias#(TLog#(MATRIX_Y_MAX), t_ADDR_MAX_Y_SZ),
              NumAlias#(TMul#(TLog#(BLOCK_SIZE), 2), t_BLOCK_ADDR_SZ),
              NumAlias#(TSub#(t_BLOCK_ADDR_SZ, TLog#(n_BANKS)), t_BANK_ADDR_SZ),
              Add#(TAdd#(t_ADDR_MAX_X_SZ, t_ADDR_MAX_Y_SZ), extraBits, t_ADDR_SZ),
              Alias#(Bit#(t_ADDR_MAX_X_SZ), t_ADDR_MAX_X),
              Alias#(Bit#(t_ADDR_MAX_Y_SZ), t_ADDR_MAX_Y),
              Alias#(Bit#(t_BLOCK_ADDR_SZ), t_BLOCK_ADDR),
              Alias#(Bit#(TLog#(n_BANKS)), t_BANK_IDX),
              Alias#(Bit#(t_BANK_ADDR_SZ), t_BANK_ADDR));

    //FIFOF#(t_BUFFER_DATA) dataBuffer <- mkSizedAutoMemFIFOF(valueOf(n_DEPTH), defaultValue);
    //FIFOF#(t_BUFFER_DATA) localBuffer <- mkSizedAutoMemFIFOF(valueOf(n_LOCAL_BUF_DEPTH)+1, defaultValue);
    
    FIFOF#(t_BUFFER_DATA) dataBuffer <- mkSizedBRAMFIFOF(valueOf(n_DEPTH));
    FIFOF#(t_BUFFER_DATA) localBuffer <- mkSizedBRAMFIFOF(valueOf(n_LOCAL_BUF_DEPTH)+1);

    Reg#(MATRIX_LOAD_BUFFER_MANAGER_STATE) state <- mkReg(STATE_IDLE);
    
    Reg#(t_ADDR_MAX_X) matrixStartX            <- mkReg(0);
    Reg#(t_ADDR_MAX_Y) matrixStartY            <- mkReg(0);
    Reg#(t_ADDR_MAX_X) matrixEndX              <- mkReg(0);
    Reg#(t_ADDR_MAX_Y) matrixEndY              <- mkReg(0);
    Reg#(Bit#(TLog#(BLOCK_SIZE))) maxBlockAddr <- mkReg(0);
    Reg#(Bool)       readLastBlock             <- mkReg(False);
    
    Wire#(BLOCK_SIZE_BITS) blockInitCmdW                       <- mkWire();
    Wire#(Tuple3#(t_ADDR_MAX_X, t_ADDR_MAX_Y, Bool)) startCmdW <- mkWire();

    Reg#(MATRIX_SIZE_BITS) rowSizeBits <- mkReg(0);

    function t_ADDR calMemAddr(t_ADDR_MAX_X rx, t_ADDR_MAX_Y ry);
        Bit#(TAdd#(TAdd#(t_ADDR_MAX_X_SZ, 1), t_ADDR_MAX_Y_SZ)) addr = (zeroExtend(ry) << rowSizeBits) + zeroExtend(rx); 
        return unpack(resize(addr));
    endfunction
    
    Reg#(t_ADDR_MAX_X) matrixAddrX <- mkReg(0);
    Reg#(t_ADDR_MAX_Y) matrixAddrY <- mkReg(0);

    Reg#(t_BANK_ADDR) respBankAddrPtr <- mkReg(0);
    Reg#(t_BANK_ADDR) maxBankAddr     <- mkReg(0);
    Reg#(t_BANK_IDX)  respBankIdxPtr  <- mkReg(0);
    Reg#(Vector#(n_BANKS, t_MEM_DATA)) respBuffer <- mkReg(unpack(0));
    PulseWire readMatrixReqDoneW  <- mkPulseWire();
    PulseWire readMatrixRespDoneW <- mkPulseWire();
    PulseWire readStartW          <- mkPulseWire();

    (* fire_when_enabled*)
    rule changeReadState (readMatrixReqDoneW || readMatrixRespDoneW || readStartW);
        if (readStartW)
        begin
            state <= STATE_READ;
        end
        else if (readMatrixRespDoneW && state == STATE_READ_WAIT) // response done with the last block
        begin
            state <= STATE_IDLE;
        end
        else if (readMatrixReqDoneW) // read request done with the current block
        begin
            state <= (readLastBlock)? STATE_READ_WAIT: STATE_READ_DONE;
        end
    endrule
    
    (* fire_when_enabled*)
    rule startNewBlock (readStartW);
        match {.addr_x, .addr_y, .is_last} = startCmdW;
        matrixStartX <= addr_x;
        matrixStartY <= addr_y;
        matrixAddrX  <= addr_x;
        matrixAddrY  <= addr_y;
        
        // end addresses
        let end_x = pack(addr_x) + zeroExtendNP(maxBlockAddr); 
        let end_y = pack(addr_y) + zeroExtendNP(maxBlockAddr); 
        matrixEndX <= end_x; 
        matrixEndY <= end_y;
        readLastBlock <= is_last;
        
        debugLog.record($format("setStartAddr: matrix B: x=0x%x, y=0x%x, end addr: x=0x%x, y=0x%x", 
                        addr_x, addr_y, end_x, end_y));
    endrule
    
    (* fire_when_enabled*)
    rule initBLockSize (True);
        let size_bits = blockInitCmdW;
        Bit#(TAdd#(TLog#(TMul#(TLog#(BLOCK_SIZE),2)),1)) element_bits = resize(pack(size_bits)) << 1;
        Bit#(TAdd#(TMul#(TLog#(BLOCK_SIZE),2),1)) total_element = 1 << element_bits;
        Bit#(TAdd#(TLog#(BLOCK_SIZE),1)) total_row = 1 << size_bits;
        Bit#(TAdd#(TMul#(TLog#(BLOCK_SIZE),2),1)) total_bank = total_element >> fromInteger(valueOf(TLog#(n_BANKS)));
        maxBankAddr  <= resize(total_bank -1);
        maxBlockAddr <= truncate(total_row-1);
        debugLog.record($format("setBlockSize: matrix B: size bits=0x%x, total_row=0x%x, total_bank=0x%x", 
                        size_bits, total_row, total_bank));
    endrule

    (* mutually_exclusive = "startNewBlock, readMatrix" *)
    rule readMatrix (state == STATE_READ);
        let addr = calMemAddr(matrixAddrX, matrixAddrY);
        mem.readReq(addr);
        debugLog.record($format("readMatrixB: x=0x%x, y=0x%x, addr=0x%x", matrixAddrX, matrixAddrY, addr));
        if (matrixAddrX == matrixEndX)
        begin
            if (matrixAddrY == matrixEndY)
            begin
                readMatrixReqDoneW.send();
                debugLog.record($format("readMatrixB: memRead issue done"));
            end
            else
            begin
                matrixAddrX <= matrixStartX;
                matrixAddrY <= matrixAddrY + 1;
            end
        end
        else 
        begin
            matrixAddrX <= matrixAddrX + 1;
        end
    endrule

    rule recvDataFromMem (state != STATE_IDLE);
        let resp <- mem.readRsp();
        debugLog.record($format("recvDataFromMatrixB: data=0x%x", resp));
        
        let data_vec = respBuffer;
        data_vec[respBankIdxPtr] = resp;

        if (respBankIdxPtr == fromInteger(valueOf(n_BANKS)-1)) //last bank
        begin
            dataBuffer.enq(unpack(resize(pack(data_vec))));
            debugLog.record($format("recvDataFromMatrixB: dataBuffer enq: 0x%x", pack(data_vec)));
            respBuffer <= unpack(0);
            respBankIdxPtr <= 0;
            if (respBankAddrPtr == maxBankAddr)
            begin
                respBankAddrPtr <= 0;
                readMatrixRespDoneW.send();
                debugLog.record($format("readMatrixB: memRead resp done"));
            end
            else
            begin
                respBankAddrPtr <= respBankAddrPtr + 1;
            end
        end
        else
        begin
            respBuffer <= data_vec;
            respBankIdxPtr <= respBankIdxPtr + 1;
            debugLog.record($format("recvDataFromMatrixB: respBuffer update: 0x%x", pack(data_vec)));
        end
    endrule
    
    // =======================================================================
    //
    // Methods
    //
    // =======================================================================
    
    method Action setMatrixRowSize(MATRIX_SIZE_BITS sizeBits) if (state == STATE_IDLE);
        rowSizeBits <= sizeBits;
        debugLog.record($format("setMatrixRowSize: matrix B: row size bits=0x%x", sizeBits));
    endmethod
   
    method Action setBlockSize(BLOCK_SIZE_BITS sizeBits) if (state == STATE_IDLE);
        blockInitCmdW <= sizeBits;
    endmethod

    method Action setStartAddr(t_ADDR_MAX_X addrX, t_ADDR_MAX_Y addrY, Bool isLastBlock) if (state == STATE_IDLE || state == STATE_READ_DONE);
        readStartW.send();
        startCmdW <= tuple3(addrX, addrY, isLastBlock); 
    endmethod
    
    method ActionValue#(t_BUFFER_DATA) getDataFromMem();
        let r = dataBuffer.first();
        dataBuffer.deq();
        debugLog.record($format("getData from memory buffer B: data=0x%x", r));
        return r;  
    endmethod
    
    method t_BUFFER_DATA peekDataFromMem();
       return dataBuffer.first();
    endmethod

    method Action putDataIntoBuffer(t_BUFFER_DATA data);
        localBuffer.enq(data);
        debugLog.record($format("putData to local buffer B: data=0x%x", data));
    endmethod
    
    method ActionValue#(t_BUFFER_DATA) getDataFromBuffer();
        let r = localBuffer.first();
        localBuffer.deq();
        debugLog.record($format("getData from local buffer B: data=0x%x", r));
        return r;
    endmethod
    
endmodule

typedef enum
{
    STATE_IDLE,
    STATE_WRITE,
    STATE_WRITE_WAIT
}
MATRIX_STORE_BUFFER_MANAGER_STATE
    deriving (Bits, Eq);

module [CONNECTED_MODULE] mkBufferManagerC#(MEMORY_WRITER_IFC#(t_ADDR, t_MEM_DATA) mem,
                                            NumTypeParam#(n_WORDS) wordNum, 
                                            DEBUG_FILE debugLog)
    // interface:
    (MATRIX_STORE_MANAGER#(t_BUFFER_DATA))
    provisos (Bits#(t_ADDR, t_ADDR_SZ),
              Bits#(t_MEM_DATA, t_MEM_DATA_SZ),
              Bits#(t_BUFFER_DATA, t_BUFFER_DATA_SZ),
              NumAlias#(TDiv#(t_BUFFER_DATA_SZ, t_MEM_DATA_SZ), n_BANKS),
              NumAlias#(TSub#(TLog#(MATRIX_X_MAX), TLog#(n_WORDS)), t_ADDR_MAX_X_SZ),
              NumAlias#(TLog#(MATRIX_Y_MAX), t_ADDR_MAX_Y_SZ),
              NumAlias#(TSub#(TMul#(TLog#(BLOCK_SIZE), 2), TLog#(n_WORDS)), t_BLOCK_ADDR_SZ),
              NumAlias#(TSub#(t_BLOCK_ADDR_SZ, TLog#(n_BANKS)), t_BANK_ADDR_SZ),
              Add#(TAdd#(t_ADDR_MAX_X_SZ, t_ADDR_MAX_Y_SZ), extraBits, t_ADDR_SZ),
              Alias#(Bit#(t_ADDR_MAX_X_SZ), t_ADDR_MAX_X),
              Alias#(Bit#(t_ADDR_MAX_Y_SZ), t_ADDR_MAX_Y),
              Alias#(Bit#(t_BLOCK_ADDR_SZ), t_BLOCK_ADDR),
              Alias#(Bit#(TLog#(n_BANKS)), t_BANK_IDX),
              Alias#(Bit#(t_BANK_ADDR_SZ), t_BANK_ADDR));

    //FIFOF#(Tuple2#(t_ADDR, t_MEM_DATA)) writebackBuffer <- mkSizedAutoMemFIFOF(valueOf(MATRIX_C_WRITE_BACK_BUFFER_DEPTH), defaultValue);
    FIFOF#(Tuple2#(t_ADDR, t_MEM_DATA)) writebackBuffer <- mkSizedBRAMFIFOF(valueOf(MATRIX_C_WRITE_BACK_BUFFER_DEPTH));
    BRAM#(t_BANK_ADDR, t_BUFFER_DATA) dataBuffer <- mkBRAM();
    Reg#(MATRIX_STORE_BUFFER_MANAGER_STATE) state <- mkReg(STATE_IDLE);
    
    Reg#(t_ADDR_MAX_X) matrixStartX     <- mkReg(0);
    Reg#(t_ADDR_MAX_Y) matrixStartY     <- mkReg(0);
    Reg#(t_BANK_ADDR)  maxBankAddr      <- mkReg(0);
    Reg#(Bool)         writeLastBlock   <- mkReg(False);

    Reg#(BLOCK_SIZE_BITS)  blockRowSizeBits  <- mkReg(fromInteger(valueOf(TLog#(BLOCK_SIZE))));
    Reg#(MATRIX_SIZE_BITS)      rowSizeBits  <- mkReg(0);

    function t_ADDR calMemAddr(t_BANK_ADDR bank_addr, t_BANK_IDX bank_idx);
        t_BLOCK_ADDR block_addr = pack(tuple2(bank_addr, bank_idx));
        t_BLOCK_ADDR by = block_addr >> blockRowSizeBits;
        t_BLOCK_ADDR bx = block_addr - (by << blockRowSizeBits);
        t_ADDR_MAX_X rx = matrixStartX + resize(bx);
        t_ADDR_MAX_Y ry = matrixStartY + resize(by);
        Bit#(TAdd#(TAdd#(t_ADDR_MAX_X_SZ, 1), t_ADDR_MAX_Y_SZ)) addr = (zeroExtend(ry) << rowSizeBits) + zeroExtend(rx); 
        return unpack(resize(addr));
    endfunction
    
    FIFOF#(Tuple2#(t_BANK_ADDR, Bool)) readBankReqQ <- mkFIFOF();
    FIFOF#(t_BUFFER_DATA) readBankRespQ <- mkSizedBypassFIFOF(2);
    FIFO#(t_BANK_ADDR) engineReadBankReqQ <- mkBypassFIFO();
    FIFOLevelIfc#(Tuple2#(t_ADDR, t_MEM_DATA), MATRIX_C_WRITE_BACK_FIFO_DEPTH) wbReqQ <- mkFIFOLevel();
    //Reg#(Vector#(n_BANKS, t_MEM_DATA)) wbBuffer <- mkRegU;
    Reg#(Vector#(n_BANKS, t_MEM_DATA)) wbBuffer <- mkConfigRegU;
    RWire#(t_MEM_DATA) wbData <- mkRWire();
    Reg#(Maybe#(Tuple2#(t_BANK_ADDR, t_BUFFER_DATA))) bypassWriteReq <- mkReg(tagged Invalid);
    Reg#(Bool) bankWriteReqPending <- mkReg(False);
    Reg#(Bool) checkBankWriteReqPending <- mkReg(False);
    Reg#(Maybe#(t_BANK_ADDR)) wbReadyAddr <- mkReg(tagged Invalid); // address of data ready to write back
    Reg#(Maybe#(t_BANK_ADDR)) wbCompletedAddr <- mkReg(tagged Invalid); // address of data that is already written back to memory
    PulseWire wbReadBankReqW <- mkPulseWire();
    PulseWire wbRespReadyW <- mkPulseWire();

    (* fire_when_enabled*)
    rule engineReadBankReq (!wbReadBankReqW && !readBankRespQ.notEmpty);
        let bank_addr = engineReadBankReqQ.first();
        engineReadBankReqQ.deq();
        readBankReqQ.enq(tuple2(bank_addr, True));
        dataBuffer.readReq(bank_addr);
        debugLog.record($format("readBankReq: dataBuffer C: addr=0x%x", bank_addr)); 
    endrule

    rule recvNormalRespFromBuffer (tpl_2(readBankReqQ.first()));
        let r <- dataBuffer.readRsp();
        let addr = tpl_1(readBankReqQ.first());
        readBankReqQ.deq();
        let resp = r;
        if (bypassWriteReq matches tagged Valid .w_req &&& tpl_1(w_req) == addr)
        begin
            resp = tpl_2(w_req);
        end
        readBankRespQ.enq(resp);
        debugLog.record($format("recvRespFromBufferC: response for engine request: addr=0x%x, data=0x%x", addr, resp)); 
    endrule
    

    (* fire_when_enabled*)
    rule writePendingBankWriteReq (checkBankWriteReqPending && bankWriteReqPending);
        match {.bank_addr, .data} = fromMaybe(?, bypassWriteReq);
        if (state == STATE_IDLE || (state == STATE_WRITE && bank_addr < fromMaybe(0, wbCompletedAddr)))
        begin
            dataBuffer.write(bank_addr, data);
            debugLog.record($format("writePendingBankWriteReq: dataBuffer C: addr=0x%x, data=0x%x", bank_addr, data)); 
            bankWriteReqPending <= False;
        end
    endrule

    // =======================================================================
    //
    // Processing write back requests 
    //
    // =======================================================================
   
    Reg#(Bool)       wbBankReqPending <- mkReg(False);
    Reg#(Bool)            wbRespReady <- mkReg(False);
    Reg#(Bool)         firstWriteback <- mkReg(True);
    Reg#(t_BANK_IDX)     wbBankIdxPtr <- mkReg(0);
    Wire#(t_BANK_ADDR) bankReadyAddrW <- mkWire();
    PulseWire              wbBankEndW <- mkPulseWire();
    PulseWire        readyForNewBankW <- mkPulseWire();
    PulseWire                wbStartW <- mkPulseWire();
    PulseWire                 wbDoneW <- mkPulseWire();
    PulseWire         writeLastBlockW <- mkPulseWire();

    // start writeback 
    (* fire_when_enabled*)
    rule startWriteback (state == STATE_IDLE && wbStartW);
        wbReadyAddr  <= tagged Invalid;
        checkBankWriteReqPending <= False;
        firstWriteback <= True;
        writeLastBlock <= writeLastBlockW? True : False;
        state <= STATE_WRITE;
        debugLog.record($format("startWriteback: start write back buffer C..."));
    endrule

    // end writeback 
    (* fire_when_enabled*)
    rule finishWriteback (state == STATE_WRITE && wbDoneW);
        state <= (writeLastBlock)? STATE_WRITE_WAIT : STATE_IDLE;
        debugLog.record($format("startWriteback: finish write back buffer C..."));
    endrule

    (* fire_when_enabled*)
    rule bankReadyForWriteback (state == STATE_WRITE);
        t_BANK_ADDR bank_addr = bankReadyAddrW;
        wbReadyAddr <= tagged Valid bank_addr;
        if (bank_addr == maxBankAddr)
        begin
            checkBankWriteReqPending <= True;
        end
    endrule

    // issue buffer read request to get writeback bank data
    (* mutually_exclusive = "engineReadBankReq, writebackBankReq" *)
    rule writebackBankReq (state == STATE_WRITE && (fromMaybe(0, wbCompletedAddr) < fromMaybe(0, wbReadyAddr)) && !wbBankReqPending && (firstWriteback || readyForNewBankW));
        wbReadBankReqW.send();
        firstWriteback <= False;
        let complete_addr = fromMaybe(0, wbCompletedAddr);
        let addr = (!firstWriteback)? ((wbBankIdxPtr == 0 || !isValid(wbCompletedAddr))? (complete_addr + 1) : (complete_addr + 2)) : 0;
        readBankReqQ.enq(tuple2(addr, False));
        dataBuffer.readReq(addr);
        debugLog.record($format("writebackBankReq: read dataBuffer C: addr=0x%x", addr));
        wbBankReqPending <= True;
    endrule
    
    // recvWritebackRespFromBuffer
    rule recvWritebackRespFromBuffer (!tpl_2(readBankReqQ.first()));
        let r <- dataBuffer.readRsp();
        let addr = tpl_1(readBankReqQ.first());
        readBankReqQ.deq();
        let resp = r;
        if (bypassWriteReq matches tagged Valid .w_req &&& tpl_1(w_req) == addr)
        begin
            resp = tpl_2(w_req);
        end
        Vector#(n_BANKS, t_MEM_DATA) wb_buffer = unpack(resize(pack(resp)));
        wbData.wset(wb_buffer[0]);
        wbBuffer <= wb_buffer;
        wbRespReadyW.send();
        debugLog.record($format("recvRespFromBufferC: response for write back: addr=0x%x, data=0x%x", addr, resp)); 
    endrule

    // send bank data to wbReqQ
    // (* conflict_free = "sendWritebackData, recvWritebackRespFromBuffer" *)
    (* fire_when_enabled*)
    rule sendWritebackData (state == STATE_WRITE && (wbRespReadyW || wbRespReady));
        let complete_addr = fromMaybe(?, wbCompletedAddr);
        let bank_addr = isValid(wbCompletedAddr)? (complete_addr+1) : 0;
        let data_vec = wbBuffer;
        let data = (wbRespReadyW && wbBankIdxPtr == 0)? fromMaybe(?,wbData.wget()) : data_vec[wbBankIdxPtr];
        let mem_addr = calMemAddr(bank_addr, wbBankIdxPtr);
        wbReqQ.enq(tuple2(mem_addr, data));
        debugLog.record($format("sendWritebackData: bank_addr=0x%x, bank_idx=0x%x, mem_addr=0x%x, data=0x%x", 
                        bank_addr, wbBankIdxPtr, mem_addr, data));
        wbBankIdxPtr <= wbBankIdxPtr + 1;
        if (wbBankIdxPtr == fromInteger(valueOf(n_BANKS)-1))
        begin
            wbBankEndW.send();
            debugLog.record($format("sendWritebackData: done with the current bank, complete_bank_addr=0x%x", bank_addr)); 
            if (isValid(wbCompletedAddr) && bank_addr == maxBankAddr) // last bank to write back
            begin
                wbDoneW.send();
                wbCompletedAddr <= tagged Invalid;
                debugLog.record($format("sendWritebackData: done with the current block")); 
            end
            else
            begin
                wbCompletedAddr <= tagged Valid bank_addr;
            end
        end
    endrule

    // checkReadyForNewBank
    (* fire_when_enabled*)
    rule checkReadyForNewBank (state == STATE_WRITE);
        let complete_addr = fromMaybe(0, wbCompletedAddr);
        if (!wbRespReady)
        begin
            readyForNewBankW.send();
        end
        else if (valueOf(n_BANKS) >=2 && (wbBankIdxPtr == fromInteger(valueOf(n_BANKS)-2)) && wbReqQ.isLessThan(valueOf(MATRIX_C_WRITE_BACK_FIFO_DEPTH)-2) && ((complete_addr+1) < maxBankAddr))
        begin
            readyForNewBankW.send();
        end
        else if ((wbBankIdxPtr == fromInteger(valueOf(n_BANKS)-1)) && wbReqQ.notFull && ((complete_addr+1) < maxBankAddr))
        begin
            readyForNewBankW.send();
        end
    endrule
    
    // checkRespReady
    (* fire_when_enabled, no_implicit_conditions *)
    rule checkRespReady(True);
        if (wbRespReadyW)
        begin
            wbRespReady <= True;
        end
        else if (wbBankEndW)
        begin
            wbRespReady <= False;
        end
    endrule

    (* fire_when_enabled, no_implicit_conditions *)
    rule checkPendingWrite (wbBankReqPending && wbRespReadyW);
        wbBankReqPending <= False;
    endrule

    // forwardWritebackReq
    (* fire_when_enabled*)
    rule forwardWritebackReq (True);
        match {.addr, .data} = wbReqQ.first();
        //mem.write(addr, data);
        writebackBuffer.enq(wbReqQ.first());
        debugLog.record($format("forwardWritebackReq: matrix C: addr=0x%x, data=0x%x", addr, data)); 
        wbReqQ.deq();
    endrule
    
    // forwardWritebackReq
    (* fire_when_enabled*)
    rule forwardWritebackReqFromBuffer (True);
        match {.addr, .data} = writebackBuffer.first();
        mem.write(addr, data);
        debugLog.record($format("forwardWritebackReqFromBuffer: matrix C: addr=0x%x, data=0x%x", addr, data)); 
        writebackBuffer.deq();
    endrule

    (* mutually_exclusive = "startWriteback, finishWriteback, waitForAllWriteDone" *)
    rule waitForAllWriteDone (state == STATE_WRITE_WAIT && !wbReqQ.notEmpty && !writebackBuffer.notEmpty);
        state <= STATE_IDLE;
        debugLog.record($format("waitForAllWriteDone: all write backs are completed..."));
    endrule

    // =======================================================================
    //
    // Methods
    //
    // =======================================================================
    
    method Action setMatrixRowSize(MATRIX_SIZE_BITS sizeBits) if (state == STATE_IDLE);
        let size_bits = sizeBits - fromInteger(valueOf(TLog#(n_WORDS)));
        rowSizeBits <= size_bits;
        debugLog.record($format("setMatrixRowSize: matrix C: row size bits=0x%x", size_bits));
    endmethod
   
    method Action setBlockSize(BLOCK_SIZE_BITS sizeBits) if (state == STATE_IDLE);
        Bit#(TAdd#(TLog#(TMul#(TLog#(BLOCK_SIZE),2)),1)) element_bits = (resize(pack(sizeBits)) << 1) - fromInteger(valueOf(TLog#(n_WORDS)));
        Bit#(TAdd#(TMul#(TLog#(BLOCK_SIZE),2),1)) total_element = 1 << element_bits;
        Bit#(TAdd#(TLog#(BLOCK_SIZE),1)) total_row = 1 << sizeBits;
        Bit#(TAdd#(TMul#(TLog#(BLOCK_SIZE),2),1)) total_bank = total_element >> fromInteger(valueOf(TLog#(n_BANKS)));
        BLOCK_SIZE_BITS row_size_bit = sizeBits - fromInteger(valueOf(TLog#(n_WORDS)));
        blockRowSizeBits <= row_size_bit;
        maxBankAddr      <= resize(total_bank -1);
        debugLog.record($format("setBlockSize: matrix C: block size bits=0x%x, row size bits=0x%x, total_row=0x%x, total_bank=0x%x", 
                        sizeBits, row_size_bit, total_row, total_bank));
    endmethod

    method Action setStartAddr(MATRIX_ADDR_X_MAX addrX, MATRIX_ADDR_Y_MAX addrY, Bool isLastBlock) if (state == STATE_IDLE);
        matrixStartX <= truncate(addrX >> fromInteger(valueOf(TLog#(n_WORDS))));
        matrixStartY <= addrY;
        wbStartW.send();
        if (isLastBlock)
        begin
            writeLastBlockW.send();
        end
        debugLog.record($format("setStartAddr: matrix C: x=0x%x, y=0x%x", addrX, addrY));
    endmethod
    
    method Action readBankReq(Bit#(TMul#(TLog#(BLOCK_SIZE),2)) addr);
        engineReadBankReqQ.enq(truncate(addr));
    endmethod

    method Action writeBankReq(Bit#(TMul#(TLog#(BLOCK_SIZE),2)) addr, t_BUFFER_DATA data) if (!checkBankWriteReqPending || !bankWriteReqPending);
        t_BANK_ADDR bank_addr = truncate(addr);
        bypassWriteReq <= tagged Valid tuple2(bank_addr, data);
        if (state == STATE_IDLE || (state == STATE_WRITE && bank_addr < fromMaybe(0, wbCompletedAddr)) || !checkBankWriteReqPending)
        begin
            dataBuffer.write(bank_addr, data);
            debugLog.record($format("writeBankReq: dataBuffer C: addr=0x%x, data=0x%x", bank_addr, data)); 
        end
        else
        begin
            bankWriteReqPending <= True;
            debugLog.record($format("writeBankReq: dataBuffer C: delay writeBankReq: addr=0x%x, data=0x%x, write_back_complete_addr=0x%x",
                            bank_addr, data, fromMaybe(0, wbCompletedAddr))); 
        end
    endmethod

    method Action bankCompleted(Bit#(TMul#(TLog#(BLOCK_SIZE),2)) addr);
        t_BANK_ADDR bank_addr = truncate(addr);
        bankReadyAddrW <= bank_addr;
        debugLog.record($format("bankCompleted: dataBuffer C: addr=0x%x", bank_addr)); 
    endmethod

    method ActionValue#(t_BUFFER_DATA) readBankResp();
        let resp = readBankRespQ.first();
        readBankRespQ.deq();
        debugLog.record($format("readBankResp: dataBuffer C: data=0x%x", resp)); 
        return resp;
    endmethod

    method Bool notBusy() = (state == STATE_IDLE);
    
endmodule

typedef enum
{
    STATE_IDLE,
    STATE_COMPUTE,
    STATE_WAIT_FOR_WB
}
MATRIX_MULTIPLY_ENGINE_STATE
    deriving (Bits, Eq);

module [CONNECTED_MODULE] mkMatrixMultiplyEngine#(MEMORY_READER_IFC#(t_ADDR, t_DATA) matrixA,
                                                  MEMORY_READER_IFC#(t_ADDR, t_DATA) matrixB,
                                                  MEMORY_WRITER_IFC#(t_MATRIX_C_ADDR, t_MATRIX_C_DATA) matrixC,
                                                  Integer engineID)
    // interface:
    (MATRIX_MULTIPLY_ENGINE_IFC)
    provisos (Bits#(t_ADDR, t_ADDR_SZ),
              Bits#(t_DATA, t_DATA_SZ),
              Bits#(t_MATRIX_C_ADDR, t_MATRIX_C_ADDR_SZ),
              Bits#(t_MATRIX_C_DATA, t_MATRIX_C_DATA_SZ),
              NumAlias#(TDiv#(t_MATRIX_C_DATA_SZ, t_DATA_SZ), n_MATRIX_C_WORD),
              NumAlias#(TLog#(MATRIX_X_MAX), t_ADDR_MAX_X_SZ),
              NumAlias#(TLog#(MATRIX_Y_MAX), t_ADDR_MAX_Y_SZ),
              NumAlias#(TExp#(TMin#(TLog#(BLOCK_SIZE), TLog#(N_LOCAL_MULTIPLIERS))), n),
              NumAlias#(TDiv#(BLOCK_SIZE, n), n_B_GROUPS_PER_ROW),
              NumAlias#(TMul#(TLog#(BLOCK_SIZE), 2), t_BLOCK_ADDR_SZ),
              NumAlias#(TSub#(t_BLOCK_ADDR_SZ, TLog#(n)), t_BANK_ADDR_SZ),
              Add#(TAdd#(t_ADDR_MAX_X_SZ, t_ADDR_MAX_Y_SZ), extraBits, t_ADDR_SZ),
              Add#(TAdd#(TSub#(t_ADDR_MAX_X_SZ, TLog#(n_MATRIX_C_WORD)), t_ADDR_MAX_Y_SZ), extraBits2, t_MATRIX_C_ADDR_SZ),
              Alias#(Bit#(t_ADDR_MAX_X_SZ), t_ADDR_MAX_X),
              Alias#(Bit#(t_ADDR_MAX_Y_SZ), t_ADDR_MAX_Y),
              Alias#(Bit#(t_BANK_ADDR_SZ), t_BANK_ADDR),
              Alias#(Bit#(t_BLOCK_ADDR_SZ), t_BLOCK_ADDR));

    Reg#(MATRIX_MULTIPLY_ENGINE_STATE) state   <- mkReg(STATE_IDLE);
    Reg#(Bool) engineDone <- mkReg(True);
    Reg#(Bit#(TLog#(BLOCK_SIZE))) maxBlockAddr <- mkReg(0);
    Reg#(Bit#(TLog#(n_B_GROUPS_PER_ROW))) maxGroupIdx <- mkReg(0); 
    Reg#(Bit#(TAdd#(1, TLog#(n_B_GROUPS_PER_ROW)))) totalGroupPerRow <- mkReg(0);
    FIFO#(Tuple3#(Bool, Bool, Bool)) blockCmdQ <- mkFIFO();
    
    FIFO#(Tuple7#(t_ADDR_MAX_X, t_ADDR_MAX_Y, t_ADDR_MAX_X, t_ADDR_MAX_Y, Bool, Bool, Bool)) startAddrCmdQ <- mkSizedFIFO(valueOf(MATRIX_MULTIPLY_CMD_FIFO_DEPTH));

    DEBUG_FILE debugLog <- mkDebugFile("matrix_multiply_engine_" + integerToString(engineID) + ".out");
    DEBUG_FILE bufferDebugLog <- mkDebugFile("matrix_multiply_engine_" + integerToString(engineID) + "_buffer.out");

    // =======================================================================
    //
    // Memory Access Engines and Local Buffers
    //
    // =======================================================================
    
    NumTypeParam#(MATRIX_A_BUFFER_DEPTH) bufferAdepth = ?;
    MATRIX_LOAD_MANAGER#(t_DATA) bufferManagerA <- mkBufferManagerA(matrixA, bufferAdepth, bufferDebugLog);
    
    NumTypeParam#(MATRIX_B_BUFFER_DEPTH) bufferBdepth = ?;
    MATRIX_LOAD_MANAGER#(Vector#(n, t_DATA)) bufferManagerB <- mkBufferManagerB(matrixB, bufferBdepth, bufferDebugLog);
   
    NumTypeParam#(n_MATRIX_C_WORD) matrixWordNum = ?; // matrix C data contains how many pixels
    MATRIX_STORE_MANAGER#(Vector#(n, t_DATA)) bufferManagerC <- mkBufferManagerC(matrixC, matrixWordNum, bufferDebugLog);

    // =======================================================================
    //
    // Multipliers
    //
    // =======================================================================
    
    Vector#(n, MULTIPLIER_IFC#(Complex#(Bit#(TDiv#(t_DATA_SZ, 2))))) multipliers <- replicateM(mkComplexMultiplier);
    
    // =======================================================================
    //
    // Computation
    //
    // =======================================================================
    
    Reg#(Bool)                            loadIssueDone <- mkReg(False);
    // row pointer for buffer A
    Reg#(Bit#(TLog#(BLOCK_SIZE)))        loadRowAddrPtr <- mkReg(0); 
    Reg#(Bit#(TLog#(BLOCK_SIZE)))     loadColumnAddrPtr <- mkReg(0);
    // group (n-pixel group) pointer for buffer B
    Reg#(Bit#(TLog#(n_B_GROUPS_PER_ROW)))  loadGroupPtr <- mkReg(0); 
    
    // row pointer for buffer C
    Reg#(Bit#(TLog#(BLOCK_SIZE)))        storeRowAddrPtr <- mkReg(0);
    // group (n-pixel group) pointer for buffer C
    Reg#(Bit#(TLog#(n_B_GROUPS_PER_ROW)))  storeGroupPtr <- mkReg(0);
    Reg#(Bit#(TLog#(BLOCK_SIZE)))          accumulateCnt <- mkReg(0);
    PulseWire                        productComputeDoneW <- mkPulseWire();
    
    function t_BANK_ADDR calBankAddr(Bit#(TLog#(BLOCK_SIZE)) row_addr, Bit#(TLog#(n_B_GROUPS_PER_ROW)) group_idx);
        Bit#(TAdd#(1, TAdd#(TLog#(BLOCK_SIZE), TLog#(n_B_GROUPS_PER_ROW)))) addr = zeroExtend(row_addr) * zeroExtend(totalGroupPerRow) + zeroExtend(group_idx); 
        return unpack(resize(addr));
    endfunction

    function Action doMultiply(multiplier, x);
        action
            multiplier.inputReq(unpack(resize(pack(tpl_1(x)))), unpack(resize(pack(tpl_2(x)))));    
        endaction
    endfunction
   
    function ActionValue#(t_DATA) getProduct(multiplier);
        actionvalue
            let resp <- multiplier.getProductResp();
            return unpack(resize(pack(resp)));
        endactionvalue
    endfunction
    
    function Complex#(Bit#(TDiv#(t_DATA_SZ, 2))) doComplexAdd( Complex#(Bit#(TDiv#(t_DATA_SZ, 2))) x, Complex#(Bit#(TDiv#(t_DATA_SZ, 2))) y);
        return cmplx( x.rel + y.rel, x.img + y.img );
    endfunction

    function t_DATA doAdd( t_DATA x, t_DATA y);
        return unpack(resize(pack(doComplexAdd(unpack(resize(pack(x))), unpack(resize(pack(y)))))));
    endfunction

    rule sendMultiplierInputData (state == STATE_COMPUTE && !loadIssueDone);
        // load data from local buffers
        let data_a = bufferManagerA.peekDataFromMem();
        let data_vec_b = ?;
        if (loadRowAddrPtr == 0)
        begin
            data_vec_b <- bufferManagerB.getDataFromMem();
        end
        else
        begin
            data_vec_b <- bufferManagerB.getDataFromBuffer();
        end
        if (loadGroupPtr == maxGroupIdx) //last pixel group in the row
        begin
            loadGroupPtr <= 0;
            let tmp <- bufferManagerA.getDataFromMem(); // dequeue the buffered pixel
            debugLog.record($format("sendMultiplierInputData: the last pixel group of the current row from matrix B"));
            if (loadRowAddrPtr == maxBlockAddr)
            begin
                loadRowAddrPtr <= 0;
                if (loadColumnAddrPtr == maxBlockAddr) //last pixel of the current block
                begin
                    loadColumnAddrPtr <= 0;
                    loadIssueDone <= tpl_2(blockCmdQ.first()); // compute the last block
                    debugLog.record($format("sendMultiplierInputData: the last pixel of the current block"));
                end
                else
                begin
                    loadColumnAddrPtr <= (loadColumnAddrPtr + 1);
                    debugLog.record($format("sendMultiplierInputData: the last pixel of the current column from matrix A"));
                end
            end
            else
            begin
                loadRowAddrPtr <= loadRowAddrPtr + 1;
            end
        end
        else
        begin
            loadGroupPtr <= loadGroupPtr + 1;
        end
        if (loadRowAddrPtr != maxBlockAddr) //last row from buffer A
        begin
            bufferManagerB.putDataIntoBuffer(data_vec_b); //recycle the pixel groups for buffer B
        end
        debugLog.record($format("sendMultiplierInputData: pixcel value from matrix A: 0x%x, row_addr=0x%x, col_addr=0x%X", data_a, loadRowAddrPtr, loadColumnAddrPtr));
        debugLog.record($format("sendMultiplierInputData: pixcel values from matrix B: 0x%x, group_addr=0x%x", pack(data_vec_b), loadGroupPtr));
        // feed them into multipliers
        let data_vec = zip(replicate(data_a), data_vec_b);
        zipWithM_(doMultiply, multipliers, data_vec);
        // issue read request to buffer C
        bufferManagerC.readBankReq(zeroExtend(pack(calBankAddr(loadRowAddrPtr, loadGroupPtr))));
    endrule

    rule getProductResults (state == STATE_COMPUTE);
        let products <- mapM(getProduct, multipliers);
        let resp_c <- bufferManagerC.readBankResp();
        let data_vec_c = (tpl_3(blockCmdQ.first()) && (accumulateCnt == 0)) ? unpack(0) : resp_c;
        let result_vec = zipWith(doAdd, products, data_vec_c);
        bufferManagerC.writeBankReq(zeroExtend(pack(calBankAddr(storeRowAddrPtr,storeGroupPtr))), result_vec);
        debugLog.record($format("getProductResults: products=0x%X", products));
        debugLog.record($format("getProductResults: update buffer C: old_val=0x%X, new_val=0x%x, row_addr=0x%x, group_addr=0x%x, accu_cnt=0x%x", 
                        data_vec_c, result_vec, storeRowAddrPtr, storeGroupPtr, accumulateCnt));
        if (accumulateCnt == maxBlockAddr && tpl_1(blockCmdQ.first())) //need to write back buffer C to memory
        begin
            bufferManagerC.bankCompleted(zeroExtend(pack(calBankAddr(storeRowAddrPtr, storeGroupPtr))));
            debugLog.record($format("getProductResults: buffer C bank completed")); 
        end
        if (storeGroupPtr == maxGroupIdx)
        begin
            storeGroupPtr <= 0;
            if (storeRowAddrPtr == maxBlockAddr)
            begin
                storeRowAddrPtr <= 0;
                if (accumulateCnt == maxBlockAddr) // last bank of the current block
                begin
                    accumulateCnt <= 0;
                    debugLog.record($format("getProductResults: compute the last bank of the block, isLastBlock=%s", 
                                    tpl_2(blockCmdQ.first())? "True" : "False"));
                    if (tpl_2(blockCmdQ.first())) // compute the last block
                    begin
                        productComputeDoneW.send();
                    end
                    blockCmdQ.deq();
                end
                else 
                begin
                    accumulateCnt <= accumulateCnt + 1;
                end
            end
            else
            begin
                storeRowAddrPtr <= storeRowAddrPtr + 1;
            end
        end
        else
        begin
            storeGroupPtr <= storeGroupPtr + 1;
        end
    endrule

    // =======================================================================
    //
    // Block and State Transition
    //
    // =======================================================================

    PulseWire recvInitCmdW <- mkPulseWire();
    PulseWire firstBlockStartW <- mkPulseWire();

    (* fire_when_enabled*)
    rule engineStart (state == STATE_IDLE && firstBlockStartW);
        state <= STATE_COMPUTE;
        debugLog.record($format("engineStart: engine start..."));
    endrule
    
    (* fire_when_enabled*)
    rule computeDone (state == STATE_COMPUTE && productComputeDoneW);
        state <= STATE_WAIT_FOR_WB;
        debugLog.record($format("computeDone: computation done..."));
    endrule

    (* mutually_exclusive = "engineStart, computeDone, waitForWriteBack" *)
    (* fire_when_enabled*)
    rule waitForWriteBack (state == STATE_WAIT_FOR_WB && bufferManagerC.notBusy());
        state <= STATE_IDLE;
        engineDone <= True;
        debugLog.record($format("waitForWriteBack: engine done with final block"));
    endrule

    (* mutually_exclusive = "waitForWriteBack, resetStateRegs" *)
    (* fire_when_enabled*)
    rule resetStateRegs (state == STATE_IDLE && recvInitCmdW);
        engineDone <= False;
        loadIssueDone <= False;
    endrule
    
    (* descending_urgency = "processNewBlockCmd, getProductResults" *)
    rule processNewBlockCmd (state == STATE_COMPUTE && !firstBlockStartW);
        match {.ax, .ay, .bx, .by, .is_last, .need_write_back, .zero_buffer_c} = startAddrCmdQ.first();
        startAddrCmdQ.deq();
        bufferManagerA.setStartAddr(ax, ay, is_last);
        bufferManagerB.setStartAddr(bx, by, is_last);
        debugLog.record($format("setStartAddr: matrix A: x=0x%x, y=0x%x", ax, ay));
        debugLog.record($format("setStartAddr: matrix B: x=0x%x, y=0x%x", bx, by));
        if (need_write_back)
        begin
            bufferManagerC.setStartAddr(bx, ay, is_last);
            debugLog.record($format("setStartAddr: matrix C: x=0x%x, y=0x%x, isLastBlock=%s, needWriteBack=%s", 
                            bx, ay, is_last? "True" : "False", need_write_back? "True" : "False"));
        end
        blockCmdQ.enq(tuple3(need_write_back, is_last, zero_buffer_c));
    endrule

    // =======================================================================
    //
    // Methods
    //
    // =======================================================================

    method Action setMatrixRowSize(MATRIX_SIZE_BITS matrixAsizeBits, MATRIX_SIZE_BITS matrixBsizeBits) if (state == STATE_IDLE);
        bufferManagerA.setMatrixRowSize(matrixAsizeBits);
        bufferManagerB.setMatrixRowSize(matrixBsizeBits);
        bufferManagerC.setMatrixRowSize(matrixBsizeBits);
        recvInitCmdW.send();
        debugLog.record($format("setMatrixRowSize: matrix A size bits=%0d, matrix B size bits=%0d", matrixAsizeBits, matrixBsizeBits));
    endmethod
   
    method Action setBlockSize(BLOCK_SIZE_BITS sizeBits) if (state == STATE_IDLE);
        Bit#(TAdd#(TLog#(BLOCK_SIZE),1)) total_row = 1 << sizeBits;
        Bit#(TAdd#(TLog#(BLOCK_SIZE),1)) total_group_per_row = total_row >> fromInteger(valueOf(TLog#(n)));
        maxBlockAddr <= truncate(total_row-1);
        maxGroupIdx <= truncate(total_group_per_row-1);
        totalGroupPerRow <= truncate(total_group_per_row);
        bufferManagerA.setBlockSize(sizeBits);
        bufferManagerB.setBlockSize(sizeBits);
        bufferManagerC.setBlockSize(sizeBits);
        debugLog.record($format("setBlockSize: maxBlockAddr=0x%x, maxGroupIdx=0x%x", total_row-1, total_group_per_row-1));
    endmethod

    method Action setStartAddr(t_ADDR_MAX_X matrixAx, t_ADDR_MAX_Y matrixAy, t_ADDR_MAX_X matrixBx, t_ADDR_MAX_Y matrixBy, Bool isFirstBlock, Bool isLastBlock, Bool needWriteBack, Bool newBlockC);
        if (isFirstBlock)
        begin
            bufferManagerA.setStartAddr(matrixAx, matrixAy, isLastBlock);
            bufferManagerB.setStartAddr(matrixBx, matrixBy, isLastBlock);
            debugLog.record($format("setStartAddr: matrix A: x=0x%x, y=0x%x", matrixAx, matrixAy));
            debugLog.record($format("setStartAddr: matrix B: x=0x%x, y=0x%x", matrixBx, matrixBy));
            if (needWriteBack)
            begin
                bufferManagerC.setStartAddr(matrixBx, matrixAy, isLastBlock);
                debugLog.record($format("setStartAddr: matrix C: x=0x%x, y=0x%x", matrixBx, matrixAy));
            end
            blockCmdQ.enq(tuple3(needWriteBack, isLastBlock, newBlockC));
            firstBlockStartW.send();
        end
        else
        begin
            startAddrCmdQ.enq(tuple7(matrixAx, matrixAy, matrixBx, matrixBy, isLastBlock, needWriteBack, newBlockC));
        end
    endmethod
    
    method Bool notBusy() = (state == STATE_IDLE);
    method Bool done() = engineDone;
 
endmodule

