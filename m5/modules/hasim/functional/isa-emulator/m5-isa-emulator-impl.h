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
 
#ifndef _M5_ISA_EMULATOR_IMPL_
#define _M5_ISA_EMULATOR_IMPL_

#include <stdio.h>

#include "asim/syntax.h"

#include "asim/provides/funcp_base_types.h"
#include "asim/provides/hasim_isa.h"
#include "asim/provides/isa_emulator.h"
#include "asim/provides/m5_hasim_base.h"

// this module provides both client and service functionalities
typedef class ISA_EMULATOR_IMPL_CLASS* ISA_EMULATOR_IMPL;

class ISA_EMULATOR_IMPL_CLASS : public M5_HASIM_BASE_CLASS
{
  public:
    ISA_EMULATOR_IMPL_CLASS(ISA_EMULATOR parent);
    ~ISA_EMULATOR_IMPL_CLASS();

    void SyncReg(
        CONTEXT_ID ctxId,
        ISA_REG_INDEX_CLASS rName,
        FUNCP_REG rVal);

    ISA_EMULATOR_RESULT Emulate(
        CONTEXT_ID ctxId,
        FUNCP_VADDR pc,
        ISA_INSTRUCTION inst,
        FUNCP_VADDR *newPC);

  private:
    ISA_EMULATOR_RESULT StartProgram(
        CONTEXT_ID ctxId,
        FUNCP_VADDR curPC,
        FUNCP_VADDR *newPC);

    ISA_EMULATOR parent;
    bool *didInit;      // Initialized bit (one per hardware context)
    UINT32 *skewCnt;    // Counter used during skewed start

    FUNCP_INT_REG intRegCache[TheISA::NumIntArchRegs];
    FUNCP_FP_REG fpRegCache[TheISA::NumFloatArchRegs];
};


// this module provides both client and service functionalities
typedef class ISA_REGOP_EMULATOR_IMPL_CLASS* ISA_REGOP_EMULATOR_IMPL;

class ISA_REGOP_EMULATOR_IMPL_CLASS : public M5_HASIM_BASE_CLASS
{
  public:
    ISA_REGOP_EMULATOR_IMPL_CLASS(ISA_REGOP_EMULATOR parent);
    ~ISA_REGOP_EMULATOR_IMPL_CLASS();

    FUNCP_REG EmulateRegOp(
        CONTEXT_ID ctxId,
        FUNCP_VADDR pc,
        ISA_INSTRUCTION inst,
        FUNCP_REG srcVal0,
        FUNCP_REG srcVal1,
        ISA_REG_INDEX_CLASS rNameSrc0,
        ISA_REG_INDEX_CLASS rNameSrc1,
        ISA_REG_INDEX_CLASS rNameDst);

  private:
    ISA_REGOP_EMULATOR parent;
};

#endif // _M5_ISA_EMULATOR_IMPL_
