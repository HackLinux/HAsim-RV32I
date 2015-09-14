#include <stdio.h>
#include <sstream>
#include <iostream>
#include <math.h>
#include "awb/provides/connected_application.h"
#include "awb/provides/stats_service.h"
#include "awb/provides/li_base_types.h"
#include "awb/provides/umf.h"
#include "awb/provides/matrix_multiply_common.h"

static UINT32 matrixTestNum  = 5;
static UINT32 matrixAsizeX[] = {64,128,256,512,1024};
static UINT32 matrixAsizeY[] = {64,128,256,512,1024};
static UINT32 matrixBsizeX[] = {64,128,256,512,1024};
static UINT32 matrixBsizeY[] = {64,128,256,512,1024};
static UINT32 matrixTestIter = 4;
// static UINT32 matrixTestNum  = 1;
// static UINT32 matrixAsizeX[] = {256};
// static UINT32 matrixAsizeY[] = {256};
// static UINT32 matrixBsizeX[] = {256};
// static UINT32 matrixBsizeY[] = {256};
// static UINT32 matrixTestIter = 1;

#define initInstruction(engine_num, matrix_a_size, matrix_b_size, block_size) \
    (UINT128(engine_num) << 24 | UINT128(matrix_a_size) << 16 | UINT128(matrix_b_size) << 8 | UINT128(block_size))

#define computeInstruction(e_id, a_x, a_y, b_x, b_y, block_info) \
    (UINT128(1) << 120 | UINT128(e_id) << 104 | UINT128(a_x) << 80 | UINT128(a_y) << 56 | UINT128(b_x) << 32 | UINT128(b_y) << 8 | UINT128(block_info))

#define checkInstruction(log_element) \
    (UINT128(2) << 120 | UINT128(log_element))

using namespace std;

// constructor                                                                                                                      
CONNECTED_APPLICATION_CLASS::CONNECTED_APPLICATION_CLASS(VIRTUAL_PLATFORM vp)
{
}

// destructor                                                                                                                       
CONNECTED_APPLICATION_CLASS::~CONNECTED_APPLICATION_CLASS()
{
}

// getBlockInfo                                                                                 
int
CONNECTED_APPLICATION_CLASS::convertBlockInfo(bool first_block, bool last_block, bool need_wb, bool new_block, bool last_inst)
{
    return (int(first_block)*16 + int(last_block)*8 + int(need_wb)*4 + int(new_block)*2 + int(last_inst));
}

// calBlockId
void 
CONNECTED_APPLICATION_CLASS::incrBlockId(int old_x, int old_y, int& new_x, int& new_y, int max_x, int max_y)
{
    // z shape block ID walkthrough 
    if (max_x >= max_y)
    {
        if ((old_x == old_y) || ((old_x > old_y) && (old_y == max_y)) )
        {
            new_x = old_x + 1;
            new_y = 0;
        }
        else if (old_x < old_y)
        {
            new_x = old_x + 1;
            new_y = old_y;
        }
        else // old_x > old_y && old_y < max_y
        {
            new_x = (old_x == (old_y + 1))? 0 : old_x;
            new_y = old_y + 1;
        }
    }
    else // max_x < max_y
    {
        if (old_x == old_y)
        {
            new_x = old_x + 1;
            new_y = 0;
        }
        else if (old_x > old_y)
        {
            new_x = (old_x == (old_y + 1))? 0 : old_x;
            new_y = old_y + 1;
        }
        else if (old_x < max_x) // old_x < old_y && old_x < max_x
        {
            new_x = old_x + 1;
            new_y = old_y;
        }
        else
        {
            new_x = 0;
            new_y = old_y + 1;
        }
    }
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
    
    std::string inputName("FPGATOCPU");
    LI_CHANNEL_RECV_CLASS<UINT128> input(inputName);
    std::string outputName("CPUTOFPGA");
    LI_CHANNEL_SEND_CLASS<UINT128> output(outputName);
    
    int maxMatrixSizeX = MATRIX_MULTIPLY_MAX_SIZE_X;
    int maxMatrixSizeY = MATRIX_MULTIPLY_MAX_SIZE_Y;

    int blockSize = MATRIX_MULTIPLY_BLOCK_SIZE;
    //int blockSize = 32;
    int engineNumber = MATRIX_MULTIPLY_NUM_ENGINES;
    //int engineNumber = 1;
    
    bool resultCheck = (MATRIX_MULTIPLY_RESULT_CHECK == 1);
    //bool resultCheck = false;
    
    UINT128 cycleCnt = 0;
    UINT128 errorCnt = 0;

    for(int test = 0; test < matrixTestNum; test = test + 1)
    {
        int matrix_a_x = matrixAsizeX[test];
        int matrix_a_y = matrixAsizeY[test];
        int matrix_b_x = matrixBsizeX[test];
        int matrix_b_y = matrixBsizeY[test];
        int matrix_c_x = matrix_b_x;
        int matrix_c_y = matrix_a_y;

        if (matrix_a_x > maxMatrixSizeX)
        {
            cerr << "Error: matrix A size X (" << matrix_a_x << ") exceeds the maximum matrix size X (" << maxMatrixSizeX << ")" << endl;
            break;
        }

        if (matrix_a_y > maxMatrixSizeY)
        {
            cerr << "Error: matrix A size Y (" << matrix_a_y << ") exceeds the maximum matrix size Y (" << maxMatrixSizeY << ")" << endl;
            break;
        }
        
        if (matrix_b_x > maxMatrixSizeX)
        {
            cerr << "Error: matrix B size X (" << matrix_b_x << ") exceeds the maximum matrix size X (" << maxMatrixSizeX << ")" << endl;
            break;
        }

        if (matrix_b_y > maxMatrixSizeY)
        {
            cerr << "Error: matrix B size Y (" << matrix_b_y << ") exceeds the maximum matrix size Y (" << maxMatrixSizeY << ")" << endl;
            break;
        }

        if (matrix_a_x != matrix_b_y)
        {
            cerr << "Error: matrix A size X (" << matrix_a_x << ") does not match matrix B size Y (" << matrix_b_y << ")" << endl;
            break;
        }
        
        int block_num_x = matrix_c_x/blockSize;
        int block_num_y = matrix_c_y/blockSize;
        int block_num = block_num_x * block_num_y;
        engineNumber = (engineNumber <= block_num)? engineNumber : block_num;

        printf("MatrixSize: matrix A: %d x %d, matrix B: %d x %d, matrix C: %d x %d \nBlockSize: %d x %d, BlockNum: %d, EngineNumber: %d \n",
               matrix_a_x, matrix_a_y, matrix_b_x, matrix_b_y, matrix_c_x, matrix_c_y, blockSize, blockSize, block_num, engineNumber); 
        
        int log_matrix_a_x = log2(matrix_a_x);
        //int log_matrix_a_y = log2(matrix_a_y);
        int log_matrix_b_x = log2(matrix_b_x);
        int log_block_size = log2(blockSize);
        int accu_cnt = matrix_a_x/blockSize;
        
        int* block_id_x_arr = new int [engineNumber];
        int* block_id_y_arr = new int [engineNumber];
        
        for(int iters = 0; iters < matrixTestIter; iters++)
        {
            stringstream filename;
            // send initialization instruction
            UINT128 init_inst = (UINT128) initInstruction(engineNumber, log_matrix_a_x, log_matrix_b_x, log_block_size);
            output.push(init_inst);
            
            int block_id_x = 0;
            int block_id_y = 0;
            int block_cnt  = 0;

            while (block_cnt < block_num)
            {
                // calculate block ids
                block_id_x_arr[0] = block_id_x;
                block_id_y_arr[0] = block_id_y;
                if (engineNumber > 1)
                {
                    for (int e = 1; e < engineNumber; e++)
                        incrBlockId(block_id_x_arr[e-1], block_id_y_arr[e-1], block_id_x_arr[e], block_id_y_arr[e], block_num_x, block_num_y);
                }
                
                for (int k = 0; k < accu_cnt; k++)
                {
                    for (int e = 0; e < engineNumber; e++)
                    {
                        if ((block_cnt + e) < block_num)
                        {
                            int start_a_x = k * blockSize;
                            int start_a_y = block_id_y_arr[e] * blockSize;
                            int start_b_x = block_id_x_arr[e] * blockSize;
                            int start_b_y = k * blockSize;
                            bool last_block = (block_cnt+1+e >= block_num);
                            int block_info = convertBlockInfo((block_cnt == 0) && (k == 0), (block_cnt + engineNumber + e >= block_num) && (k == (accu_cnt-1)), k == (accu_cnt-1), k == 0, ((block_cnt + e) == (block_num-1)) && (k == (accu_cnt-1)));
                            UINT128 compute_inst = (UINT128) computeInstruction(e, start_a_x, start_a_y, start_b_x, start_b_y, block_info);
                            output.push(compute_inst);
                            // printf("Engine Instruction: engine ID: %d, block ID (%d,%d), Step %d \n", e, block_id_x_arr[e], block_id_y_arr[e], k); 
                        }
                    }
                }
                incrBlockId(block_id_x_arr[engineNumber-1], block_id_y_arr[engineNumber-1], block_id_x, block_id_y, block_num_x, block_num_y);
                block_cnt += engineNumber;
            }

            input.pop(cycleCnt);
	        printf("Iter %d: Cycle count: %d\n", iters, cycleCnt);
            filename << "matrix_multiply_A_" << matrix_a_x << "x" << matrix_a_y << "_B_" << matrix_b_x << "x" << matrix_b_y << "_" << iters << ".stats";
            STATS_SERVER_CLASS::GetInstance()->DumpStats();
            STATS_SERVER_CLASS::GetInstance()->EmitFile(filename.str());
            STATS_SERVER_CLASS::GetInstance()->ResetStatValues();
        
            if (resultCheck)
            {
                int log_element = log2(matrix_c_x*matrix_c_y);
                UINT128 check_inst = checkInstruction(log_element);
                output.push(check_inst);
                input.pop(errorCnt);
	            printf("Iter %d: Error count: %d\n", iters, errorCnt);
                STATS_SERVER_CLASS::GetInstance()->ResetStatValues();
            }
        }

        delete [] block_id_x_arr;
        delete [] block_id_y_arr;
    }

    STARTER_SERVICE_SERVER_CLASS::GetInstance()->End(0);
  
    return 0;

}

