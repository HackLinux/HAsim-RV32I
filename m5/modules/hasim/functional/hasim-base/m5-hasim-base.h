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
// @file m5_hasim_base.h
// @brief Low level interface to m5 simulator
// @author Michael Adler
//

#ifndef __HASIM_M5_BASE__
#define __HASIM_M5_BASE__

#include <mutex>

#include "asim/syntax.h"
#include "asim/mesg.h"
#include "asim/atomic.h"

// m5
#include "cpu/simple/atomic.hh"

typedef class M5_HASIM_BASE_CLASS *M5_HASIM_BASE;
typedef AtomicSimpleCPU *AtomicSimpleCPU_PTR;

class M5_HASIM_BASE_CLASS
{
  public:
    M5_HASIM_BASE_CLASS();
    ~M5_HASIM_BASE_CLASS();

    UINT32 NumCPUs() const { return numCPUs; };

  protected:
    AtomicSimpleCPU *M5Cpu(UINT32 cpuId) const
    {
        VERIFY(cpuId <= numCPUs, "Told to load a workload context that does not exist!");
        return m5cpus[cpuId];
    };

    // Calling m5 in parallel is dangerous.
    static std::mutex m5mutex;

  private:
    static ATOMIC32_CLASS refCnt;
    static AtomicSimpleCPU_PTR *m5cpus;
    static UINT32 numCPUs;
};

#endif //  __HASIM_M5_BASE__
