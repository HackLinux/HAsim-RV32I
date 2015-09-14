#ifndef _MEMORYRRR_
#define _MEMORYRRR_

#include <stdio.h>
#include <sys/time.h>
#include <glib.h>

#include "awb/provides/low_level_platform_interface.h"
#include "awb/provides/rrr.h"
#include "awb/provides/mmm_memory_switch.h"

// this module provides the RRRTest server functionalities


typedef class MEMORY_RRR_SERVER_CLASS* MEMORY_RRR_SERVER;
class MEMORY_RRR_SERVER_CLASS: public RRR_SERVER_CLASS,
                               public PLATFORMS_MODULE_CLASS
{
  private:
    // self-instantiation
    static MEMORY_RRR_SERVER_CLASS instance;
    // server stub
    RRR_SERVER_STUB serverStub;
    GAsyncQueue *dataQ;

  public:
    MEMORY_RRR_SERVER_CLASS();
    ~MEMORY_RRR_SERVER_CLASS();

    // static methods
    static MEMORY_RRR_SERVER GetInstance() { return &instance; }

    // required RRR methods
    void Init(PLATFORMS_MODULE);
    void Uninit();
    void Cleanup();
    bool Poll();

    UINT64 *getResponse();

    //
    // RRR service methods
    //
    void MemResp(UINT64 resp);
};



// include server stub
#include "asim/rrr/server_stub_MEMORY_RRR.h"


#endif
