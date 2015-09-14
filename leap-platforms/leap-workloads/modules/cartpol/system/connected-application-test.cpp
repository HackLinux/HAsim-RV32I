#include <stdio.h>
#include <sstream>
#include <time.h>
#include "awb/provides/stats_service.h"
#include "awb/rrr/client_stub_CARTPOLCONTROLRRR.h"
#include "awb/provides/connected_application.h"

using namespace std;


// constructor                                                                                                                      
CONNECTED_APPLICATION_CLASS::CONNECTED_APPLICATION_CLASS(VIRTUAL_PLATFORM vp)
  
{
    clientStub = new CARTPOLCONTROLRRR_CLIENT_STUB_CLASS(NULL);
}

// destructor                                                                                                                       
CONNECTED_APPLICATION_CLASS::~CONNECTED_APPLICATION_CLASS()
{
}

// init                                                                                                                             
void
CONNECTED_APPLICATION_CLASS::Init()
{
}

struct test{
  int N;
  double theta;
  double R;
};
typedef struct test TESTCASE;


#define PI 3.14159

// main                                                             
int
CONNECTED_APPLICATION_CLASS::Main()
{
    OUT_TYPE_ReadCycleCount result;
    int N[] = {10, 18, 31, 55, 96, 169, 297, 523, 729, 921};
    double r = 10.0;
    double theta = PI/6;
    
    // Send a start command down. 
    // First - the size of the projection matrix (NxN)
    // Second - the length of the radius in radians
    // Third - the angle to be swept 


    for( int index = 0;  index < sizeof(N)/sizeof(int); index++) {
        for(r = 10.0; r < 1000.0; r = r * 1.76) {
            for(theta = PI/16; theta < PI/2; theta += PI/4) {
  	        stringstream filename;
                UINT64 r_val = *((UINT64*)&r);
                UINT64 theta_val = *((UINT64*)&theta);    
		
                clientStub->PutCommand(N[index],r_val,theta_val);
                do {
                    result = clientStub->ReadCycleCount(0);
                    sleep(1);
                } while(!result.done);
		
		printf("CARTPOL:%d:%f:%f:%llu\n", N[index], r, theta, result.cycleCount);
                filename << "cartpol_" << index << "_" << r << "_" << theta << ".stats";
	        STATS_SERVER_CLASS::GetInstance()->DumpStats();
	        STATS_SERVER_CLASS::GetInstance()->EmitFile(filename.str());
	        STATS_SERVER_CLASS::GetInstance()->ResetStatValues();
	    }
	}
    }

    STARTER_DEVICE_SERVER_CLASS::GetInstance()->End(0);
  
    return 0;
}
