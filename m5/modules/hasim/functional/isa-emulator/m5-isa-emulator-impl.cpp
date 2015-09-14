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
 
#include <mutex>

#include "asim/syntax.h"

#include "asim/provides/isa_emulator_impl.h"
#include "asim/provides/funcp_memory.h"
#include "asim/provides/command_switches.h"
#include "asim/provides/commands_service.h"
#include "asim/provides/stats_service.h"

// m5 includes
#include "sim/faults.hh"

typedef enum
{
    INSTR_NORMAL,
    INSTR_STAT_RESET,
    INSTR_STAT_DUMP,
    INSTR_STAT_DUMP_RESET,
}
INSTR_SPECIAL_EMULATION;

static void *StatsUpdateThread(void *arg);

// Allow only one instance of an emulator to be running in order to avoid
// the danger of multiple threads driving Gem5.
static std::mutex emulMutex;

// Global Gem5 lock management is complicated by HAsimNoteMemoryRead/Write
// because they call back to the hardware and may trigger memory operations.
// This lock is set and cleared only after emulMutex is locked, so the
// lock itself is shared among all users.
static unique_lock<std::mutex> *emulGem5Lock = NULL;


//***********************************************************************
//
// Callbacks from m5's physical memory access routines so HAsim
// can monitor memory updates during emulation and invalid FPGA-side
// memory caches.
//
//***********************************************************************

extern void (*HAsimNoteMemoryRead)(Addr paddr, uint64_t size);
extern void (*HAsimNoteMemoryWrite)(Addr paddr, uint64_t size);

static bool inEmulation = false;
static bool emulationMayRefMemory = false;
static CONTEXT_ID emulationCtxId = 0;

void
HAsimEmulMemoryRead(Addr paddr, UINT64 size)
{
    if (inEmulation)
    {
        // Some emulation modes assume no memory is referenced
        VERIFY(emulationMayRefMemory, "Emulated REGOP touches memory!");

        inEmulation = false;    // Prevent loops
        emulGem5Lock->unlock();
        FUNCP_MEMORY_CLASS::NoteSystemMemoryRead(emulationCtxId, paddr, size);
        emulGem5Lock->lock();
        inEmulation = true;
    }
}

void
HAsimEmulMemoryWrite(Addr paddr, UINT64 size)
{
    if (inEmulation)
    {
        // Some emulation modes assume no memory is referenced
        VERIFY(emulationMayRefMemory, "Emulated REGOP touches memory!");

        inEmulation = false;    // Prevent loops
        emulGem5Lock->unlock();
        FUNCP_MEMORY_CLASS::NoteSystemMemoryWrite(emulationCtxId, paddr, size);
        emulGem5Lock->lock();
        inEmulation = true;
    }
}


// ========================================================================
//
// m5 Full Instruction Emulation...
//
// ========================================================================

ISA_EMULATOR_IMPL_CLASS::ISA_EMULATOR_IMPL_CLASS(
    ISA_EMULATOR parent) :
    parent(parent)
{
    emulGem5Lock = new unique_lock<std::mutex>(m5mutex);
    emulGem5Lock->unlock();

    HAsimNoteMemoryRead = &HAsimEmulMemoryRead;
    HAsimNoteMemoryWrite = &HAsimEmulMemoryWrite;

    didInit = new bool[NumCPUs()];
    skewCnt = new UINT32[NumCPUs()];
    for (UINT32 c = 0; c < NumCPUs(); c++)
    {
        didInit[c] = false;
        skewCnt[c] = c * SKEW_CONTEXTS;
    }
}


ISA_EMULATOR_IMPL_CLASS::~ISA_EMULATOR_IMPL_CLASS()
{
    delete[] didInit;
    delete[] skewCnt;
}


void
ISA_EMULATOR_IMPL_CLASS::SyncReg(
    CONTEXT_ID ctxId,
    ISA_REG_INDEX_CLASS rName,
    FUNCP_REG rVal)
{
    //
    // Skip register sync until the hardware has asked to emulate one instruction.
    // It will start simulation at address 0, which holds 0's.
    //
    if (! didInit[ctxId]) return;

    if (rName.IsArchReg())
    {
        M5Cpu(ctxId)->tc->setIntReg(rName.ArchRegNum(), rVal.intReg);

        ASSERTX(rName.ArchRegNum() < TheISA::NumIntArchRegs);
        intRegCache[rName.ArchRegNum()] = rVal.intReg;
    }

    if (rName.IsFPReg())
    {
        M5Cpu(ctxId)->tc->setFloatReg(rName.FPRegNum(), rVal.fpReg);

        ASSERTX(rName.FPRegNum() < TheISA::NumFloatArchRegs);
        fpRegCache[rName.FPRegNum()] = rVal.fpReg;
    }
}


ISA_EMULATOR_RESULT
ISA_EMULATOR_IMPL_CLASS::Emulate(
    CONTEXT_ID ctxId,
    FUNCP_VADDR pc,
    ISA_INSTRUCTION inst,
    FUNCP_VADDR *newPC)
{
    unique_lock<std::mutex> isa_emul_lock(emulMutex);

    if (! didInit[ctxId])
    {
        return StartProgram(ctxId, pc, newPC);
    }

    INSTR_SPECIAL_EMULATION special_instr = INSTR_NORMAL;

#if THE_ISA == ALPHA_ISA
    //
    // HALT?  Return an error
    //
    if (inst == 0)
    {
        ASIMWARNING("HALT instruction executed.  Probably a bug." << endl);
        *newPC = 0;
        return ISA_EMULATOR_EXIT_FAIL;
    }

    //
    // These should be decoded in ISA-specific code.
    //
    switch (inst)
    {
      case 0x04000040:
        special_instr = INSTR_STAT_RESET;
        break;
        
      case 0x04000041:
        special_instr = INSTR_STAT_DUMP;
        break;
        
      case 0x04000042:
        special_instr = INSTR_STAT_DUMP_RESET;
        break;
    }
#endif


    if (special_instr != INSTR_NORMAL)
    {
        // Pause simulation so state doesn't change during stats dump
        COMMANDS_SERVER_CLASS::GetInstance()->Pause();

        // For now we need a separate thread to request the dump due
        // to RRR limitations.  When RRR is eliminated this will not be
        // needed.
        pthread_t tid;
        uintptr_t arg = uintptr_t(special_instr);
        VERIFYX(! pthread_create(&tid, NULL, &StatsUpdateThread, (void*)arg));
    }

    //
    // Set the m5 state and emulate a tick.  The code below is derived from
    // m5's AtomicSimpleCPU::tick()
    //
    emulGem5Lock->lock();
    AtomicSimpleCPU *cpu = M5Cpu(ctxId);

    // m5 better not be in the middle of an instruction
    VERIFYX(! cpu->curMacroStaticInst);

    //
    // Set the machine state and execute the instruction
    //
    curEventQueue(mainEventQueue[0]);
    TheISA::PCState new_pc = cpu->tc->pcState();
    new_pc.set(pc);
    cpu->tc->pcState(new_pc);

    cpu->inst = inst;
    cpu->preExecute();

    VERIFYX(cpu->curStaticInst);

    // Start watching memory
    inEmulation = true;
    emulationMayRefMemory = true;

    emulationCtxId = ctxId;

    //
    // Is the instruction a branch?
    //
    TheISA::PCState branchTarget;
    bool isBranch = cpu->curStaticInst->isControl();
    if (isBranch)
    {
        cpu->curStaticInst->hasBranchTarget(new_pc, cpu->tc, branchTarget);
    }

    Fault fault;
    int fault_trips = 0;
    do
    {
        fault = cpu->curStaticInst->execute(cpu, cpu->getTraceData());
        if (fault != NoFault)
        {
            fault_trips += 1;
            VERIFY(fault_trips < 10, "Too many faults while emulating instr at 0x" << fmt_x(pc) << " in m5");
            fault->invoke(cpu->tc);
        }
    }
    while (fault != NoFault);

    cpu->postExecute();
    VERIFYX(! cpu->stayAtPC);

    cpu->advancePC(fault);

    // Stop watching memory
    inEmulation = false;

    //
    // Update registers
    //
    for (int r = TheISA::NumFloatArchRegs - 1; r >= 0 ; r--)
    {
        ISA_REG_INDEX_CLASS rName;
        rName.SetFPReg(r);
        FUNCP_FP_REG rVal = M5Cpu(ctxId)->tc->readFloatReg(r);
        if (fpRegCache[r] != rVal)
        {
            FUNCP_REG v;
            v.fpReg = rVal;
            parent->UpdateRegister(ctxId, rName, v);
            fpRegCache[r] = rVal;
        }
    }

    // We loop backwards because the protocol requires integer r0 to
    // be sent last.
    for (int r = TheISA::NumIntArchRegs - 1; r >= 0 ; r--)
    {
        ISA_REG_INDEX_CLASS rName;
        rName.SetArchReg(r);
        FUNCP_INT_REG rVal = M5Cpu(ctxId)->tc->readIntReg(r);
        if ((intRegCache[r] != rVal) || (r == 0))
        {
            FUNCP_REG v;
            v.intReg = rVal;
            parent->UpdateRegister(ctxId, rName, v);
            intRegCache[r] = rVal;
        }
    }

    if (isBranch)
    {
        Addr cur_pc = cpu->tc->pcState().pc();
        if (cur_pc == branchTarget.pc())
        {
            *newPC = cur_pc;
        }
        else
        {
            *newPC = 0;
            isBranch = false;
        }
    }

    if (cpu->tc->exitCalled())
    {
        emulGem5Lock->unlock();
        return (cpu->tc->exitCode() == 0) ? ISA_EMULATOR_EXIT_OK : ISA_EMULATOR_EXIT_FAIL;
    }

    if (cpu->tc->status() == ThreadContext::Halted)
    {
        // Emulation caused the thread to halt.  Branch to 0 (the normal startup
        // location) and reset the context's state.
        *newPC = 0;
        isBranch = true;
        didInit[ctxId] = false;
    }

    emulGem5Lock->unlock();
    return isBranch ? ISA_EMULATOR_BRANCH : ISA_EMULATOR_NORMAL;
}


//
// Called once at the beginning of the program to set initial register state
// and the PC.
//
ISA_EMULATOR_RESULT
ISA_EMULATOR_IMPL_CLASS::StartProgram(
    CONTEXT_ID ctxId,
    FUNCP_VADDR curPC,
    FUNCP_VADDR *newPC)
{
    ASSERTX(sizeof(ISA_INSTRUCTION) == sizeof(TheISA::MachInst));

    //
    // Skewed start forces contexts to loop back to the start PC for some
    // number of instructions so that the contexts won't all be doing the
    // same thing every cycle when each context is running the same workload.
    //

    bool active = true;

    if (skewCnt[ctxId] != 0)
    {
        // Artificial skew at program start requires more waiting.
        skewCnt[ctxId] -= 1;
        active = false;
    }
    
    if (M5Cpu(ctxId)->tc->status() == ThreadContext::Halted)
    {
        // Thread is halted.  Continue sleeping.
        active = false;
    }

    if (! active)
    {
        //
        // Not ready to start this context.  Loop back to same PC for a while, then check again.
        //
        *newPC = curPC;

        // The protocol requires we update register 0.
        ISA_REG_INDEX_CLASS rName;
        FUNCP_REG rVal;
        rName.SetArchReg(0);
        rVal.intReg = M5Cpu(ctxId)->tc->readIntReg(0);
        parent->UpdateRegister(ctxId, rName, rVal);

        return ISA_EMULATOR_SLEEP;
    }
    else
    {
        //
        // Startup sequence.  HAsim model starts at PC 0.  m5 returns 0
        // for the instruction.  HAsim calls here to emulate the instruction.
        // If the PC is non-0 in M5 then the context is ready to start.
        // Otherwise tell it to sleep for a while and then check again.

        *newPC = M5Cpu(ctxId)->tc->pcState().pc();
        didInit[ctxId] = (*newPC != 0);
        
        if (didInit[ctxId])
        {

            //
            // Context is ready to start.
            // Now set all the start register values and jump to the right PC.
            //

            for (int r = TheISA::NumFloatArchRegs - 1; r >= 0 ; r--)
            {
                ISA_REG_INDEX_CLASS rName;
                FUNCP_REG rVal;
                rName.SetFPReg(r);
                rVal.fpReg = M5Cpu(ctxId)->tc->readFloatReg(r);
                parent->UpdateRegister(ctxId, rName, rVal);
                fpRegCache[r] = rVal.fpReg;
            }

            // We loop backwards because the protocol requires integer r0 to
            // be sent last.
            for (int r = TheISA::NumIntArchRegs - 1; r >= 0 ; r--)
            {
                ISA_REG_INDEX_CLASS rName;
                FUNCP_REG rVal;
                rName.SetArchReg(r);
                rVal.intReg = M5Cpu(ctxId)->tc->readIntReg(r);
                parent->UpdateRegister(ctxId, rName, rVal);
                intRegCache[r] = rVal.intReg;
            }

            ASIMWARNING("Activating Context: " << (int) ctxId << endl);
            return ISA_EMULATOR_BRANCH;
        }
        else
        {
        
            // Context is not ready. Tell the functional partition to sleep
            // this context for a while, and then check again.

            // The protocol requires we update register 0.
            ISA_REG_INDEX_CLASS rName;
            FUNCP_REG rVal;
            rName.SetArchReg(0);
            rVal.intReg = M5Cpu(ctxId)->tc->readIntReg(0);
            parent->UpdateRegister(ctxId, rName, rVal);

            return ISA_EMULATOR_SLEEP;
        }

    }
}


// ========================================================================
//
// m5 Operation-Level Emulation...
//
// ========================================================================

ISA_REGOP_EMULATOR_IMPL_CLASS::ISA_REGOP_EMULATOR_IMPL_CLASS(
    ISA_REGOP_EMULATOR parent) :
    parent(parent)
{}


ISA_REGOP_EMULATOR_IMPL_CLASS::~ISA_REGOP_EMULATOR_IMPL_CLASS()
{}


FUNCP_REG
ISA_REGOP_EMULATOR_IMPL_CLASS::EmulateRegOp(
    CONTEXT_ID ctxId,
    FUNCP_VADDR pc,
    ISA_INSTRUCTION inst,
    FUNCP_REG srcVal0,
    FUNCP_REG srcVal1,
    ISA_REG_INDEX_CLASS rNameSrc0,
    ISA_REG_INDEX_CLASS rNameSrc1,
    ISA_REG_INDEX_CLASS rNameDst)
{
    unique_lock<std::mutex> isa_emul_lock(emulMutex);
    emulGem5Lock->lock();

    if (rNameSrc0.IsArchReg())
    {
        M5Cpu(ctxId)->tc->setIntReg(rNameSrc0.ArchRegNum(), srcVal0.intReg);
    }

    if (rNameSrc0.IsFPReg())
    {
        M5Cpu(ctxId)->tc->setFloatReg(rNameSrc0.FPRegNum(), srcVal0.fpReg);
    }

    if (rNameSrc1.IsArchReg())
    {
        M5Cpu(ctxId)->tc->setIntReg(rNameSrc1.ArchRegNum(), srcVal1.intReg);
    }

    if (rNameSrc1.IsFPReg())
    {
        M5Cpu(ctxId)->tc->setFloatReg(rNameSrc1.FPRegNum(), srcVal1.fpReg);
    }


    //
    // Similar to Emulate() above but restricts the instruction to non-branch,
    // no faults and no memory.
    //

    AtomicSimpleCPU *cpu = M5Cpu(ctxId);

    // m5 better not be in the middle of an instruction
    VERIFYX(! cpu->curMacroStaticInst);

    //
    // Set the machine state and execute the instruction
    //
    curEventQueue(mainEventQueue[0]);
    TheISA::PCState new_pc = cpu->tc->pcState();
    new_pc.set(pc);
    cpu->tc->pcState(new_pc);

    cpu->inst = inst;
    cpu->preExecute();

    VERIFYX(cpu->curStaticInst);

    // Start watching memory
    inEmulation = true;
    emulationMayRefMemory = false;
    emulationCtxId = ctxId;

    //
    // Is the instruction a branch?
    //
    bool isBranch = cpu->curStaticInst->isControl();
    Fault fault = cpu->curStaticInst->execute(cpu, cpu->getTraceData());

    VERIFY((fault == NoFault), "RegOP Emulator triggered a fault!");
    VERIFY(! isBranch, "RegOP attempted emulation of branch!");

    cpu->postExecute();
    VERIFYX(! cpu->stayAtPC);

    cpu->advancePC(fault);

    // Stop watching memory
    inEmulation = false;

    FUNCP_REG rVal;
    if (rNameDst.IsArchReg())
    {
        rVal.intReg = cpu->tc->readIntReg(rNameDst.ArchRegNum());
    }
    else if (rNameDst.IsFPReg())
    {
        rVal.fpReg = cpu->tc->readFloatReg(rNameDst.FPRegNum());
    }
    else
    {
        ASIMERROR("Unexpected register type");
    }

    emulGem5Lock->unlock();

    return rVal;
}


//
// StatsUpdateThread is a hack to allow calling DumpStats() as a side-effect
// of simulating an instruction.  RRR doesn't deal well with a software service
// that calls a hardware service when the caller blocks until some other
// software service is called.  When we replace RRR with threads this pthreads
// hack may become unnecessary.
//
static void *StatsUpdateThread(void *arg)
{
    INSTR_SPECIAL_EMULATION special_instr = INSTR_SPECIAL_EMULATION(uintptr_t(arg));

    bool stat_dump;
    bool stat_reset;
    
    switch (special_instr)
    {
      case INSTR_STAT_RESET:
        stat_dump = false;
        stat_reset = true;
        break;

      case INSTR_STAT_DUMP:
        stat_dump = true;
        stat_reset = false;
        break;

      case INSTR_STAT_DUMP_RESET:
        stat_dump = true;
        stat_reset = true;
        break;

      default:
        stat_dump = false;
        stat_reset = false;
        break;
    }

    if (stat_dump)
    {
        static UINT32 region_id = 0;
        cout << "Dumping region " << region_id << " statistics..." << endl;

        // Scan out current values
        STATS_SERVER_CLASS::GetInstance()->DumpStats();

        // Write to a file
        string file_name = string(globalArgs->Workload()) + "_region_" +
                           std::to_string(region_id) + ".stats";
        STATS_SERVER_CLASS::GetInstance()->EmitFile(file_name);

        region_id += 1;
    }
        
    if (stat_reset)
    {
        cout << "Resetting statistics..." << endl;
        STATS_SERVER_CLASS::GetInstance()->ResetStatValues();
    }

    // Simulation was paused before when the special instruction was encountered.
    COMMANDS_SERVER_CLASS::GetInstance()->Resume();
}
