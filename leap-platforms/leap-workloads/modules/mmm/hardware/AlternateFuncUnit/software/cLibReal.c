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

#include "cLib.h"

#define BramBaseAddress ((volatile int*)(0xe8800000))
#define BramInstructionStart (BramBaseAddress)
#define BramOutputStart (BramBaseAddress+32)

static int bramInstruction = 0;
static int bramOutput = 0;

void initialize()
{
    int i;
    bramInstruction = 0;
    bramOutput = 0;
	for(i=0; i<64; i++)
	{
	   *(BramBaseAddress + i) = 0;
	}
}

inline int byteReverse(int x)
{
    int a =  ((x>>24) & 0xff) +
             ((x>>8 ) & 0xff00) + 
             ((x<<8 ) & 0xff0000) +
             ((x<<24) & 0xff000000); 
    return(a);
}

int putInstruction(int x0, int x1)
{
    volatile int* ptr;
    int i = 0;
    int x1rev;
    int x0rev;

    ptr = BramInstructionStart + bramInstruction;
    while(*(ptr) != 0 )
    {
        i++;
	    if(i > 1000)
	    {
            return -1;
        }
    }
    x0 = x0 | 0x80000000;

    x1rev = x1;
    x0rev = x0;

    // Write the second value first to avoid timing problems
    (*(ptr + 1)) = x1rev;
    //xil_printf("base: %x offset:%x addr: %x addra: %x \r\n",BramInstructionStart, bramInstruction, ptr, (*(ptr + 1)));
    (*ptr)       = x0rev;
    //xil_printf("Failed: %x %x\r\n",  byteReverse(*(ptr)),byteReverse(*(ptr + 1)) );
    //xil_printf("bram %x \r\n", bramInstruction); 
    while((*(ptr)) != 0)
    {
        // xil_printf("Failed: %x %x\r\n",  byteReverse(*(ptr)),byteReverse(*(ptr + 1)) );
    }	 
	//print("passed");	
	//xil_printf("bram %x \r\n", bramInstruction); 
	if((bramInstruction + 2) == 32)
    {	
        bramInstruction = 0;
    }  
	else
    {
        //xil_printf("bram %x \r\n", bramInstruction);
        bramInstruction += 2;
        //xil_printf("bram %x \r\n", bramInstruction); 
    }
 
    return 0;
}

int getResponse()
{
    int i = 0;
    int output;
    int outputRev;
    volatile int* ptr;
 
    ptr = BramOutputStart + bramOutput;

    //xil_printf("ptr = %x\r\n", byteReverse(*ptr));
    while((*ptr & 0x80000000) == 0)
    {
        i++;
        if(i > 1000)
        {
            return -1;
        }
    }
 
    //output = (*(int *)ptr) & 0x7ffffff;
    //xil_printf("ptr: %x ptr+1:%x", (*ptr & 0x80000000), (*(int*)(ptr + 1)));
    output = (*(int*)(ptr + 1)); //Read next line
    outputRev = output;
    (*(int *)ptr) = 0;
 

    if(bramOutput + 2 == 32)
    {
        bramOutput = 0;
    }
    else
    {
        bramOutput = bramOutput + 2;
    }
 
    return(outputRev & 0x7FFFFFF);
}
