/*
Copyright (c) 2007 MIT

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

Author: Muralidaran Vijayaraghavan
*/

#include <stdio.h>
#include "cLib.h"

#define MATRIX_A            0x00000000
#define MATRIX_B            0x00100000
#define MATRIX_C            0x00200000

#define INST_FULL_ADDR      0x00000000
#define INST_FIFO_ADDR      0x00000001
#define DATA_IN_FULL_ADDR   0x00000002
#define DATA_IN_FIFO_ADDR   0x00000003
#define DATA_OUT_EMPTY_ADDR 0x00000004
#define DATA_OUT_FIFO_ADDR  0x00000005
#define STORE_ADDR          0x00000006

#define ZERO                0x00000000
#define LOAD                0x00000001
#define LOAD_MUL            0x00000002
#define MUL                 0x00000003
#define STORE               0x00000004
#define LOADADDR            0x00000005
#define STOREADDR           0x00000006
#define SET_ROW_SIZE        0x00000007

#define BLOCK_SIZE          16

inline void zero()
{
    while(putInstruction(0, ZERO) == -1)
    {
    }
}

inline void load(int i, int j, int size)
{
    while(putInstruction(MATRIX_B + i*BLOCK_SIZE*size + j*BLOCK_SIZE, LOADADDR) == -1)
    {
    }

    while(putInstruction(0, LOAD))
    {
    }
}

inline void loadMul(int i, int j, int size)
{
    while(putInstruction(MATRIX_A + i*BLOCK_SIZE*size + j*BLOCK_SIZE, LOADADDR) == -1)
    {
    }

    while(putInstruction(0, LOAD_MUL))
    {
    }
}

inline void store(int i, int j, int size)
{
    while(putInstruction(MATRIX_C + i*BLOCK_SIZE*size + j*BLOCK_SIZE, STOREADDR) == -1)
    {
    }

    while(putInstruction(0, STORE))
    {
    }
}

inline void setRowSize(int rowSize)
{
    while(putInstruction(rowSize, SET_ROW_SIZE) == -1)
    {
    }
}

int multiplyFunction(int size, int logSize)
{
    int blockNum = size/BLOCK_SIZE;

    int storeTotal = blockNum*blockNum;

    int i, j, k;

    initialize();

    setRowSize(logSize);

    for(i = 0; i < blockNum; i++)
    {
        for(j = 0; j < blockNum; j++)
        {
            zero();
            for(k = 0; k < blockNum; k++)
            {
                load(k, j, size);
                loadMul(i, k, size);
            }
            store(i, j, size);
        }
    }

    while(getResponse() != storeTotal)
    {
    }
}
