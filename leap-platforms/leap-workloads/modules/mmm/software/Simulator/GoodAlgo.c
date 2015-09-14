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

#include "instructions.h"
#include "instructionParameters.h"
#include "matrixPointers.h"
#include "cLib.h"

#define createSec(m, i, j, x) \
            (m + i*BlockSize*Size*4 + (FU_Number*j+x)*BlockSize*4)

#define createPrim(m, i, j) \
            (m + i*BlockSize*Size*4 + j*BlockSize*4)

void main()
{
  //int Size = 64;
  int Size = 128;
    int BlockSize = 64;
    UInt64 a = (UInt64)aMatrix;
    UInt64 b = (UInt64)bMatrix;
    UInt64 c = (UInt64)cMatrix;

    int FU_Number = (int) FunctionalUnitNumber;

    int BlockNum = Size/BlockSize;
    int LogSize;

    int BigBlockNum = BlockNum/FU_Number;
    int BigBlockRest = BlockNum%FU_Number;

    int response;

    UInt64 inst;    

    switch(Size)
    {
        case 64:
            LogSize = 6;
            break;
        case 128:
            LogSize = 7;
            break;
        case 256:
            LogSize = 8;
            break;
        case 512:
            LogSize = 9;
            break;
        default:
            LogSize = 10;
            break;
    }

    inst = createSetRowSizeInstruction(LogSize);  

    putInstruction(inst);

    int i;
    int j;
    int k;
    int f;

    for(i = 0; i < BlockNum; i=i+1)
    {
        for(j = 0; j < BigBlockNum; j=j+1)
        {
	    inst = createArithmeticInstruction(All_FU_Mask, Zero);
            putInstruction(inst);
            for(k = 0; k < BlockNum; k=k+1)
            {
                for(f = 0; f < FU_Number; f=f+1)
                {
 	 	   inst = createLoadInstruction((((int)1)<<f), B, createSec(b, k, j, f)); 
		   putInstruction(inst);		  
                }
		   inst = createLoadInstruction(All_FU_Mask, A, createPrim(a, i, k));
		   putInstruction(inst);
		{ 

                }
  
                inst = createArithmeticInstruction(All_FU_Mask, MultiplyAddAccumulate); 
                putInstruction(inst);
		

		
            }
            for(f = 0; f < FU_Number; f=f+1)
            {
              int i = 0;
              inst = createStoreInstruction(f, C, createSec(c, i, j, f));
	      putInstruction(inst);
 
              while(getResponse() >= 0)
                {
                  i++;
                  if(i == 300000000)
		    {      
		      printf(" Uhoh were hanging \r\n");
		    }		 
                } 

            }
        }
        if(BigBlockRest != 0)
        {
	  inst = createArithmeticInstruction(All_FU_Mask, Zero);
	  putInstruction(inst);

            for(k = 0; k < BlockNum; k=k+1)
            {
                for(f = 0; f < BigBlockRest; f=f+1)
                {
                  inst = createLoadInstruction(((int)1<<f), B, createPrim(b, k, j*FU_Number+f));
		  putInstruction(inst);
 
                }

                inst = createLoadInstruction(All_FU_Mask, A, createPrim(a, i, k));
                putInstruction(inst);

                inst = createArithmeticInstruction(All_FU_Mask, MultiplyAddAccumulate);
                putInstruction(inst);

            }
            for(f = 0; f < BigBlockRest; f=f+1)
            {
              int i = 0;
              inst = createStoreInstruction(f, C, createPrim(c, i, j*FU_Number+f));
	      putInstruction(inst);

              while(getResponse <= 0)
                {
                  i++;
                  if(i == 300000000)
		    {      
		      printf(" Uhoh were hanging \r\n");
		    }		 
                }         

            }
        }
    }
}
