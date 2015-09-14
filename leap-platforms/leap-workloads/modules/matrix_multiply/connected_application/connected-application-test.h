
#ifndef __CONNECTED_APPLICATION__
#define __CONNECTED_APPLICATION__

#include "awb/provides/virtual_platform.h"
#include "awb/provides/channelio.h"

// Default software does nothing and immediately returns 0;

typedef class CONNECTED_APPLICATION_CLASS* CONNECTED_APPLICATION;
class CONNECTED_APPLICATION_CLASS
{
  private:
    int convertBlockInfo(bool first_block, bool last_block, bool need_wb, bool new_block, bool last_inst);
    void incrBlockId(int old_x, int old_y, int& new_x, int& new_y, int max_x, int max_y);
  public:
    CONNECTED_APPLICATION_CLASS(VIRTUAL_PLATFORM vp);
    ~CONNECTED_APPLICATION_CLASS();
    void Init();
    int Main();
};


#endif
