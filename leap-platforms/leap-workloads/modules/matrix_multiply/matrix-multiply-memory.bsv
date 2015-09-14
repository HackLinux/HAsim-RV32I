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
import SpecialFIFOs::*;
import Vector::*;
import DefaultValue::*;

`include "awb/provides/librl_bsv.bsh"

`include "awb/provides/mem_services.bsh"
`include "awb/provides/common_services.bsh"
`include "awb/provides/scratchpad_memory_common.bsh"
`include "awb/provides/matrix_multiply_common_params.bsh"
`include "awb/provides/matrix_multiply_common.bsh"

interface MATRIX_MEMORY_IFC#(numeric type n_PORTS, type t_ADDR, type t_DATA);
    interface Vector#(n_PORTS, MEMORY_IFC#(t_ADDR, t_DATA)) memoryPorts;
endinterface

interface MATRIX_MEMORY_READ_ONLY_IFC#(numeric type n_PORTS, type t_ADDR, type t_DATA);
    interface Vector#(n_PORTS, MEMORY_READER_IFC#(t_ADDR, t_DATA)) readPorts;
endinterface

interface MATRIX_MEMORY_WRITE_ONLY_IFC#(numeric type n_PORTS, type t_ADDR, type t_DATA);
    interface Vector#(n_PORTS, MEMORY_WRITER_IFC#(t_ADDR, t_DATA)) writePorts;
endinterface

interface MATRIX_MEMORY_ONE_READER_MULTI_WRITER_IFC#(numeric type n_PORTS, type t_ADDR, type t_DATA);
    interface MEMORY_READER_IFC#(t_ADDR, t_DATA) readPort;
    interface Vector#(n_PORTS, MEMORY_WRITER_IFC#(t_ADDR, t_DATA)) writePorts;
endinterface


// ============================================================================
//
// Various memory structure implementation
//
// ============================================================================

//
// Implementations with a single (multi-port) private scratchpad
//
module [CONNECTED_MODULE] mkReadOnlyMemWithPrivScratchpad#(Integer scratchpadID,
                                                           SCRATCHPAD_CONFIG conf)
    // interface:
    (MATRIX_MEMORY_READ_ONLY_IFC#(n_PORTS, t_ADDR, t_DATA))
    provisos (Bits#(t_ADDR, t_ADDR_SZ),
              Bits#(t_DATA, t_DATA_SZ));
    
    MEMORY_MULTI_READ_IFC#(n_PORTS, t_ADDR, t_DATA) memory <- mkMultiReadScratchpad(scratchpadID, conf);
    interface readPorts = memory.readPorts;
endmodule

module [CONNECTED_MODULE] mkWriteOnlyMemWithPrivScratchpad#(Integer scratchpadID,
                                                            SCRATCHPAD_CONFIG conf)
    // interface:
    (MATRIX_MEMORY_WRITE_ONLY_IFC#(n_PORTS, t_ADDR, t_DATA))
    provisos (Bits#(t_ADDR, t_ADDR_SZ),
              Bits#(t_DATA, t_DATA_SZ));
    
    MEMORY_IFC#(t_ADDR, t_DATA) memory <- mkScratchpad(scratchpadID, conf);
    MERGE_FIFOF#(n_PORTS, Tuple2#(t_ADDR, t_DATA)) incomingWriteReqQ <- mkMergeBypassFIFOF();

    Vector#(n_PORTS, MEMORY_WRITER_IFC#(t_ADDR, t_DATA)) portsLocal = newVector();

    rule forwardWriteReq (True);
        match {.addr, .w_data} = incomingWriteReqQ.first();
        incomingWriteReqQ.deq();
        memory.write(addr, w_data);
    endrule

    for(Integer p = 0; p < valueOf(n_PORTS); p = p + 1)
    begin
        portsLocal[p] =
            interface MEMORY_WRITER_IFC#(t_ADDR, t_DATA);
                method Action write(t_ADDR addr, t_DATA val);
                    incomingWriteReqQ.ports[p].enq(tuple2(addr, val));
                endmethod
                method Bool writeNotFull = incomingWriteReqQ.ports[p].notFull();
            endinterface;
    end

    interface writePorts = portsLocal;

endmodule

module [CONNECTED_MODULE] mkMultiWriterMemWithPrivScratchpad#(Integer scratchpadID,
                                                              SCRATCHPAD_CONFIG conf)
    // interface:
    (MATRIX_MEMORY_ONE_READER_MULTI_WRITER_IFC#(n_PORTS, t_ADDR, t_DATA))
    provisos (Bits#(t_ADDR, t_ADDR_SZ),
              Bits#(t_DATA, t_DATA_SZ));
    
    MEMORY_IFC#(t_ADDR, t_DATA) memory <- mkScratchpad(scratchpadID, conf);
    MERGE_FIFOF#(n_PORTS, Tuple2#(t_ADDR, t_DATA)) incomingWriteReqQ <- mkMergeBypassFIFOF();

    Vector#(n_PORTS, MEMORY_WRITER_IFC#(t_ADDR, t_DATA)) portsLocal = newVector();

    rule forwardWriteReq (True);
        match {.addr, .w_data} = incomingWriteReqQ.first();
        incomingWriteReqQ.deq();
        memory.write(addr, w_data);
    endrule

    for(Integer p = 0; p < valueOf(n_PORTS); p = p + 1)
    begin
        portsLocal[p] =
            interface MEMORY_WRITER_IFC#(t_ADDR, t_DATA);
                method Action write(t_ADDR addr, t_DATA val);
                    incomingWriteReqQ.ports[p].enq(tuple2(addr, val));
                endmethod
                method Bool writeNotFull = incomingWriteReqQ.ports[p].notFull();
            endinterface;
    end
    
    interface readPort = interface MEMORY_READER_IFC#(t_ADDR, t_DATA);
                             method Action readReq(t_ADDR addr) = memory.readReq(addr);
                             method ActionValue#(t_DATA) readRsp() = memory.readRsp();
                             method t_DATA peek() = memory.peek();
                             method Bool notEmpty() = memory.notEmpty();
                             method Bool notFull() = memory.notFull();
                         endinterface;
    interface writePorts = portsLocal;

endmodule

module [CONNECTED_MODULE] mkMemWithPrivScratchpad#(Integer scratchpadID,
                                                   SCRATCHPAD_CONFIG conf)
    // interface:
    (MATRIX_MEMORY_IFC#(n_PORTS, t_ADDR, t_DATA))
    provisos (Bits#(t_ADDR, t_ADDR_SZ),
              Bits#(t_DATA, t_DATA_SZ));
    
    MEMORY_MULTI_READ_IFC#(n_PORTS, t_ADDR, t_DATA) memory <- mkMultiReadScratchpad(scratchpadID, conf);
    MERGE_FIFOF#(n_PORTS, Tuple2#(t_ADDR, t_DATA)) incomingWriteReqQ <- mkMergeBypassFIFOF();

    Vector#(n_PORTS, MEMORY_IFC#(t_ADDR, t_DATA)) portsLocal = newVector();

    rule forwardWriteReq (True);
        match {.addr, .w_data} = incomingWriteReqQ.first();
        incomingWriteReqQ.deq();
        memory.write(addr, w_data);
    endrule

    for(Integer p = 0; p < valueOf(n_PORTS); p = p + 1)
    begin
        portsLocal[p] =
            interface MEMORY_IFC#(t_ADDR, t_DATA);
                method Action readReq(t_ADDR addr) = memory.readPorts[p].readReq(addr);
                method ActionValue#(t_DATA) readRsp() = memory.readPorts[p].readRsp();
                method t_DATA peek() = memory.readPorts[p].peek();
                method Bool notEmpty() = memory.readPorts[p].notEmpty();
                method Bool notFull() = memory.readPorts[p].notFull();
                method Action write(t_ADDR addr, t_DATA val);
                    incomingWriteReqQ.ports[p].enq(tuple2(addr, val));
                endmethod
                method Bool writeNotFull = incomingWriteReqQ.ports[p].notFull();
            endinterface;
    end

    interface memoryPorts = portsLocal;

endmodule

