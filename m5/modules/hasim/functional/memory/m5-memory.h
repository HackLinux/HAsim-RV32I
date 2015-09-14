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
 
//
// @file m5_memory.h
// @brief ISA-independent m5 memory access
// @author Michael Adler
//

#ifndef __HASIM_M5_MEMORY__
#define __HASIM_M5_MEMORY__

#include "asim/syntax.h"
#include "asim/provides/m5_hasim_base.h"
#include "asim/provides/funcp_base_types.h"

// m5 includes
#include "cpu/base.hh"
#include "mem/port.hh"
#include "sim/process.hh"

typedef class FUNCP_SIMULATED_MEMORY_CLASS *FUNCP_SIMULATED_MEMORY;

// Response from VtoP
struct FUNCP_MEM_VTOP_RESP
{
    UINT64 pa;
    bool pageFault;    // Translation failed
    bool ioSpace;      // Reference is to uncacheable I/O space
};

class FUNCP_SIMULATED_MEMORY_CLASS : public M5_HASIM_BASE_CLASS,
                                     public TRACEABLE_CLASS
{
  public:
    //
    // Required public interface
    //

    FUNCP_SIMULATED_MEMORY_CLASS();
    ~FUNCP_SIMULATED_MEMORY_CLASS();

    bool Read(UINT64 paddr, UINT64 size, bool isSpeculative, void *dest);
    void Write(UINT64 paddr, UINT64 size, void *src);

    FUNCP_MEM_VTOP_RESP VtoP(CONTEXT_ID ctxId, UINT64 va, bool allocOnFault);

  private:
    MasterPort *memPort;
    Format fmt_va;

    Addr guard_page;        // Mapped to virtual address 0

    bool BlobHelper(Addr paddr, uint8_t *p, int size, MemCmd cmd, bool isSpeculative);
};

#endif //  __HASIM_M5_MEMORY__
