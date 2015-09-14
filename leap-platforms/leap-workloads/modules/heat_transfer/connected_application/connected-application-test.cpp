#include <stdio.h>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include "awb/provides/connected_application.h"
#include "awb/provides/fpga_components.h"
#include "awb/provides/heat_transfer_common.h"
#include "awb/provides/dynamic_parameters_service.h"

using namespace std;

// constructor                                                                                                                      
CONNECTED_APPLICATION_CLASS::CONNECTED_APPLICATION_CLASS(VIRTUAL_PLATFORM vp)
{
}

// destructor                                                                                                                       
CONNECTED_APPLICATION_CLASS::~CONNECTED_APPLICATION_CLASS()
{
    if (HEAT_TRANSFER_RESULT_CHECK == 1)
    {
        ResultCheck();
    }
}

void
CONNECTED_APPLICATION_CLASS::ResultCheck()
{
    int frameX = HEAT_TRANSFER_TEST_X_POINTS;
    int frameY = HEAT_TRANSFER_TEST_Y_POINTS;
    int pixel_value = 0;
    int golden_value = 0;
    int error = 0;

    ifstream outfile ("output.hex", ifstream::in);   
    ifstream goldenfile ("output_golden.hex", ifstream::in);

    int total_pixel_num = frameX * frameY * 2;

    for (int i = 0; i < total_pixel_num; i++)
    {
        outfile >> hex >> pixel_value;
        goldenfile >> hex >> golden_value;
        if (pixel_value != golden_value)
        {
            error++;
        }
    }
    if (error == 0)
        cout << "Result check: Correct!" << endl;
    else
        cout << "Result check: " << dec << error << " errors" << endl;
}

// init                                                                                                                             
void
CONNECTED_APPLICATION_CLASS::Init()
{
}

// main                                                                                                                             
int
CONNECTED_APPLICATION_CLASS::Main()
{
}
