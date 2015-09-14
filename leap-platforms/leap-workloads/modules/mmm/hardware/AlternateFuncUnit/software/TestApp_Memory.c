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

Author: James Hoe, Kermin Fleming
*/

#include "xparameters.h"

#include "stdio.h"

#include "xutil.h"
#include "ddr_header.h"
#include "instructions.h"
#include "cLib.h"
#include "matrixPointers.h"

//====================================================
typedef signed int Number;
#define WIDTH (16)
#define MASK (0xffff)
#define PACK(r,i) (((r)<<WIDTH)|((i)&MASK))
#define UNPACKR(r) ((r)>>WIDTH)
#define UNPACKI(i) (((i)<<WIDTH)>>WIDTH)

#define OVERFLOW(f) (!((((f)&0xffff8000)==0)||(((f)&0xffff8000)==0xffff8000)))


static inline Number complexMult(Number A, Number B) {
        // Our interpretation is
	//   B and the return value use the same 2's complement fixed-point format
	//   A uses 2's complement fixed-point format with 1-bit sign, 1-bit integer, 14-bit fraction
	signed int Ar=UNPACKR(A);
	signed int Ai=UNPACKI(A);
	signed int Br=UNPACKR(B);
	signed int Bi=UNPACKI(B);
	Number Cr, Ci;

	Cr=((Ar*Br)>>14)-((Ai*Bi)>>14);
	Ci=((Ar*Bi)>>14)+((Ai*Br)>>14);	

	return PACK(Cr,Ci);
}

static inline Number complexAdd(Number A, Number B) {
	// A, B and the return value use the same 2's complement fixed-point format
	signed long Ar=UNPACKR(A);
	signed long Ai=UNPACKI(A);
	signed long Br=UNPACKR(B);
	signed long Bi=UNPACKI(B);
	Number Cr, Ci;

	Cr=Ar+Br;
	Ci=Ai+Bi;

	//assert(!OVERFLOW(Cr));
	//assert(!OVERFLOW(Ci));

	return PACK(Cr,Ci);
}

static inline Number complexRandom() {
	Number Cr, Ci;
	//unsigned long x=0;
   static x;

	// no random in EDK stdlib
	
	//Cr=(Number)(rand()%1024)-512;
	//Ci=(Number)(rand()%1024)-512;

	//Cr=(Number)((x+=751)%64)-32;
	//Ci=(Number)((x+=751)%64)-32;

	Cr=(Number)(x%64-32);
	Ci=(Number)(x%64-32);

   x = x + 1;
	return PACK(Cr,Ci);
}

// -----------------------------------------------------------------------------

/* Multiplies a [NB x NB] block of two square, row-major order matricies (A, B), 
	where the size of matrix A, B, and C is [N X N]. */

void mmmKernel(Number* A, Number* B, Number* C, int N, int NB) {	
        int i, j, k;
	for (j = 0; j < NB; j++)
		for (i = 0; i < NB; i++)
			for (k = 0; k < NB; k++)
				C[i * N + j] = complexAdd( C[i * N + j], complexMult(A[i * N + k], B[k * N + j]) );
}


void mmm(Number* A, Number* B, Number *C, int N) {
	// verify all arguments, sizes of A and B are not checked
	//assert(A != NULL);
	//assert(B != NULL);
	//assert((N > 0) && (N <= 0x4000));
	//assert(C != NULL);
	
	// perform the matrix-matrix multiply on the whole matrix at once
	mmmKernel(A, B, C, N, N);
	
}


int main() {

  int size, seed, i;
  Number *A, *B, *C, *ours;
  
  /*int algo[12] = {0x80000020,0x00000006,
                  0x80000000,0x00000009,
                  0x8000000c,0x00000000,
                  0x8000000d,0x00400000,
                  0x80000000,0x0000000a,
                  0x80000012,0x00800000};*/

static int algo[] = {
0x00000006, 0x00000007,
0x00000000, 0x00000005,
0x00000000, 0x00000001,
0x00100000, 0x00000005,
0x00000000, 0x00000002,
0x00000400, 0x00000005,
0x00000000, 0x00000001,
0x00100010, 0x00000005,
0x00000000, 0x00000002,
0x00000800, 0x00000005,
0x00000000, 0x00000001,
0x00100020, 0x00000005,
0x00000000, 0x00000002,
0x00000c00, 0x00000005,
0x00000000, 0x00000001,
0x00100030, 0x00000005,
0x00000000, 0x00000002,
0x00200000, 0x00000006,
0x00000000, 0x00000004,
0x00000010, 0x00000005,
0x00000000, 0x00000001,
0x00100000, 0x00000005,
0x00000000, 0x00000002,
0x00000410, 0x00000005,
0x00000000, 0x00000001,
0x00100010, 0x00000005,
0x00000000, 0x00000002,
0x00000810, 0x00000005,
0x00000000, 0x00000001,
0x00100020, 0x00000005,
0x00000000, 0x00000002,
0x00000c10, 0x00000005,
0x00000000, 0x00000001,
0x00100030, 0x00000005,
0x00000000, 0x00000002,
0x00200010, 0x00000006,
0x00000000, 0x00000004,
0x00000020, 0x00000005,
0x00000000, 0x00000001,
0x00100000, 0x00000005,
0x00000000, 0x00000002,
0x00000420, 0x00000005,
0x00000000, 0x00000001,
0x00100010, 0x00000005,
0x00000000, 0x00000002,
0x00000820, 0x00000005,
0x00000000, 0x00000001,
0x00100020, 0x00000005,
0x00000000, 0x00000002,
0x00000c20, 0x00000005,
0x00000000, 0x00000001,
0x00100030, 0x00000005,
0x00000000, 0x00000002,
0x00200020, 0x00000006,
0x00000000, 0x00000004,
0x00000030, 0x00000005,
0x00000000, 0x00000001,
0x00100000, 0x00000005,
0x00000000, 0x00000002,
0x00000430, 0x00000005,
0x00000000, 0x00000001,
0x00100010, 0x00000005,
0x00000000, 0x00000002,
0x00000830, 0x00000005,
0x00000000, 0x00000001,
0x00100020, 0x00000005,
0x00000000, 0x00000002,
0x00000c30, 0x00000005,
0x00000000, 0x00000001,
0x00100030, 0x00000005,
0x00000000, 0x00000002,
0x00200030, 0x00000006,
0x00000000, 0x00000004,
};						

  A = ((Number*)aMatrix);
  B = ((Number*)bMatrix);
  ours = ((Number*)cMatrix);
  C = ((Number*)scratch);

  size = 64;
  for(i=0; i < size * size; i++)
  {
    *(A+i) = complexRandom();   
  }
  
  for(i=0; i < size * size; i++)
  {
    *(B+i) = complexRandom();   
  } 

  print("in main \r\n");
  mmm(A,B,C,size);
  for(i=0; i < 16; i++)
  {  
     xil_printf("first data: %x\r\n", *(A+i));
  }
  initialize();
  //print("after init\r\n");
  
  i = 0;
  
  while(1)
  {
     //print("loop\r\n");
     int response;
     if(i < sizeof(algo)/sizeof(UInt64))//i is number of 2-elem slots
	  {
	     
		  if(putInstruction(algo[2*i], algo[2*i + 1]) >= 0)
		  {
			  //print("add\r\n");	
		     //xil_printf("Add %x:%x\r\n",algo[2*i],algo[2*i+1]);
			  i++;
     
		  }
		  else
		  {
		     print("Failed inst\r\n");
		  }
	  }
	  
	  response = getResponse();
	  if(response >= 0)
	  {
	     if(response == 0)
		  {
		     print("Beat\r\n");
		  }
		  else
		  {
		    xil_printf("Store resp %x\r\n", response);
		  }
	  }
  }
  
  for(i = 0; i < size * size; i++)
  {
     if(ours[i] != C[i])
	  {
	     print("Mismatch!\n\r");
		  break;
	  }
  }
}





