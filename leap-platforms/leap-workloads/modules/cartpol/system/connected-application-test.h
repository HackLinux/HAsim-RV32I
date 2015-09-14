
#ifndef __CONNECTED_APPLICATION__
#define __CONNECTED_APPLICATION__

#include "awb/provides/virtual_platform.h"
#include "awb/rrr/client_stub_CARTPOLCONTROLRRR.h"

// Default software does nothing and immediately returns 0;

typedef class CONNECTED_APPLICATION_CLASS* CONNECTED_APPLICATION;
class CONNECTED_APPLICATION_CLASS
{
  private:
    CARTPOLCONTROLRRR_CLIENT_STUB clientStub;

  public:
    CONNECTED_APPLICATION_CLASS(VIRTUAL_PLATFORM vp);
    ~CONNECTED_APPLICATION_CLASS();
    void Init();
    int Main();
};


#endif
