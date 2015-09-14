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
// @file m5_hasim_base.cpp
// @brief Low level interface to m5 simulator
// @author Michael Adler
//

#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <math.h>

#include <Python.h>

#include "asim/syntax.h"
#include "asim/mesg.h"

#include "asim/provides/command_switches.h"
#include "asim/provides/m5_hasim_base.h"

// m5
#include "sim/init.hh"
#include "sim/sim_object.hh"
extern SimObject *resolveSimObject(const string &);

std::mutex M5_HASIM_BASE_CLASS::m5mutex;
ATOMIC32_CLASS M5_HASIM_BASE_CLASS::refCnt;
AtomicSimpleCPU_PTR *M5_HASIM_BASE_CLASS::m5cpus;
UINT32 M5_HASIM_BASE_CLASS::numCPUs;


void
m5HAsimExitNowHandler(int sigtype)
{
    exit(1);
}


M5_HASIM_BASE_CLASS::M5_HASIM_BASE_CLASS()
{
    if (refCnt++ == 0)
    {
        //
        // Initialize m5
        //

        // M5 expects the executable name to be in argv[0]
        // and to receive MAX_NUM_CPUS in --num-cpus
        char** new_argv = new char* [globalArgs->FuncPlatformArgc() + 3];
        new_argv[0] = (char *)globalArgs->ExecutableName();

        // Initialize m5 special signal handling.
        initSignals();

        Py_SetProgramName(new_argv[0]);

        // initialize embedded Python interpreter
        Py_Initialize();

        // Initialize the embedded m5 python library
        VERIFY(initM5Python() == 0, "Failed to initialize m5 Python");


        for (int i = 0; i < globalArgs->FuncPlatformArgc(); i++)
        {
            new_argv[i + 1] = strdup(globalArgs->FuncPlatformArgv()[i]);
        }

        char* cpuArg  = new char[32];
        char* dirName = new char[32];

        // TEMPORARY:: For now we only load an M5 CPU for each existing
        // program.N directory. In the future this should be specified better.
        // Note that if the run script specifies it then it should only specify
        // it for HAsim models, not for every application.
        
        // Probably the right solution in the future involves have separate setup/run
        // scripts for hasim benchmarks and generic benchmarks.
        
        numCPUs = 0;
        for (int i = 0; i < MAX_NUM_CONTEXTS; i++)
        {
            sprintf(dirName, "program.%d", i);
            struct stat stFileInfo; 
            int statRes = stat(dirName, &stFileInfo); 
            bool program_exists = statRes ==0;
            if (program_exists)
                numCPUs++;
        }
        delete dirName;
        
        VERIFY(numCPUs <= MAX_NUM_CONTEXTS, "Error: more programs set up than available hardware threads!");
        VERIFY(numCPUs > 0, "Error: no programs found for hardware threads.");

        sprintf(cpuArg, "--num-cpus=%d", numCPUs);
        new_argv[globalArgs->FuncPlatformArgc() + 1] = cpuArg;

        m5Main(globalArgs->FuncPlatformArgc() + 2, new_argv);

        // Drop m5 handling of SIGINT and SIGABRT.  These don't work well since
        // m5's event loop isn't running.  Simply exit, hoping that some buffers
        // will be flushed.
        signal(SIGINT, m5HAsimExitNowHandler);
        signal(SIGABRT, m5HAsimExitNowHandler);

        ASSERTX(numCPUs != 0);

        // Cache pointers to m5 CPUs here
        m5cpus = new AtomicSimpleCPU_PTR[numCPUs];

        if (numCPUs == 1)
        {
            // If there is a single CPU it is named system.cpu
            SimObject *so = resolveSimObject("system.cpu");
            m5cpus[0] = dynamic_cast<AtomicSimpleCPU*>(so);
            ASSERT(m5cpus[0] != NULL, "Failed to find m5 cpu object:  system.cpu");
        }
        else
        {
            // m5 CPU name width varies with the number of CPUs
            int width = 1 + int(log10(double(numCPUs)));

            for (int cpu = 0; cpu < numCPUs; cpu++)
            {
                char cpu_name[64];
                sprintf(cpu_name, "system.cpu%0*d", width, cpu);
                SimObject *so = resolveSimObject(cpu_name);
                m5cpus[cpu] = dynamic_cast<AtomicSimpleCPU*>(so);
                ASSERT(m5cpus[cpu] != NULL, "Failed to find m5 cpu object: " << cpu_name);
            }
        }
    }
}


M5_HASIM_BASE_CLASS::~M5_HASIM_BASE_CLASS()
{
    if (refCnt-- == 1)
    {
        Py_Finalize();
        delete[] m5cpus;
    }
}
