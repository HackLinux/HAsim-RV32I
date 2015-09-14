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
#include "instructions.h"
#include "instructionParameters.h"
#include "matrixPointers.h"
#include "cLib.h"


#define Size 128
#define LogSize 7

//#define Size 256
//#define LogSize 8

//#define Size 64
//#define LogSize 6
//#define BlockNum 1

#define BlockSize 64

int brainDeadAlgo()
{
    int i;
    int j;
    int k;
    int response;

    UInt64 a = (UInt64)aMatrix;
    UInt64 b = (UInt64)bMatrix;
    UInt64 c = (UInt64)cMatrix;
 
    initialize();
    putInstruction(createSetRowSizeInstruction(LogSize));

    for(i = 0; i < Size; i=i+BlockSize)
    {
        for(j = 0; j < Size; j=j+BlockSize)
        {
            putInstruction(createArithmeticInstruction(1, Zero));
            for(k = 0; k < Size; k=k+BlockSize)
            {
                UInt64 aPtr = a + 4*(Size*i + k);
                UInt64 bPtr = b + 4*(Size*k + j);
                putInstruction(createLoadInstruction(1, B, bPtr));
                putInstruction(createLoadInstruction(1, A, aPtr));
                putInstruction(createArithmeticInstruction(1, MultiplyAddAccumulate));
            }
            UInt64 cPtr = c + 4*(Size*i + j);
            putInstruction(createStoreInstruction(0, C, cPtr));
        }
    }

    response = getResponse();
    if(response == (Size/BlockSize)*(Size/BlockSize))
        printf("Passed\n");
    else if(response != 0)
        printf("Failed\n");
    else
        fprintf(stderr, "Passed if simulation\n");
}

int main()
{
    brainDeadAlgo();
}
