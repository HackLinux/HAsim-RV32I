#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <sys/stat.h>
#include <glib.h>

#include "awb/rrr/service_ids.h"
#include "awb/provides/mmm_memory_switch.h"
#include "awb/rrr/server_stub_MEMORY_RRR.h"

using namespace std;

// ===== service instantiation =====
MEMORY_RRR_SERVER_CLASS MEMORY_RRR_SERVER_CLASS::instance;

// constructor
MEMORY_RRR_SERVER_CLASS::MEMORY_RRR_SERVER_CLASS()
{
    // instantiate stub
    serverStub = new MEMORY_RRR_SERVER_STUB_CLASS(this);
}

// destructor
MEMORY_RRR_SERVER_CLASS::~MEMORY_RRR_SERVER_CLASS()
{
    Cleanup();
}

// init
void
MEMORY_RRR_SERVER_CLASS::Init(
    PLATFORMS_MODULE p)
{
    parent = p;
    // Glib needs this or it complains
    if(!g_thread_supported()) {
      g_thread_init(NULL);
    }
    // Set up my FIFOs
    dataQ   = g_async_queue_new();
}

// uninit
void
MEMORY_RRR_SERVER_CLASS::Uninit()
{
    Cleanup();
    PLATFORMS_MODULE_CLASS::Uninit();
     g_async_queue_unref(dataQ);
}

// cleanup
void
MEMORY_RRR_SERVER_CLASS::Cleanup()
{
    delete serverStub;
}

// poll

bool
MEMORY_RRR_SERVER_CLASS::Poll()
{
    return false;
}


UINT64 *MEMORY_RRR_SERVER_CLASS::getResponse()
{

    return (UINT64*) g_async_queue_pop(dataQ);

}


void
MEMORY_RRR_SERVER_CLASS::MemResp(UINT64 payload)
{

    UINT64 *dataPtr = (UINT64*) malloc(sizeof(UINT64));
    g_async_queue_push(dataQ,dataPtr);

}



