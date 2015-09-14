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

#include<stdio.h>
#include<stdlib.h>
#include<assert.h>

typedef signed int Number;
#define WIDTH (16)
#define MASK (0xffff)
#define PACK(r,i) (((r)<<WIDTH)|((i)&MASK))
#define UNPACKR(r) ((r)>>WIDTH)
#define UNPACKI(i) (((i)<<WIDTH)>>WIDTH)

#define OVERFLOW(f) (!((((f)&0xffff8000)==0)||(((f)&0xffff8000)==0xffff8000)))

#ifdef MYRON
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
#endif

#ifdef MURALI
static inline Number complexMult(Number A, Number B) {
        // Our interpretation is
	//   B and the return value use the same 2's complement fixed-point format
	//   A uses 2's complement fixed-point format with 1-bit sign, 1-bit integer, 14-bit fraction
	signed int Ar=UNPACKR(A);
	signed int Ai=UNPACKI(A);
	signed int Br=UNPACKR(B);
	signed int Bi=UNPACKI(B);
	signed int Arsign=UNPACKR(A)&0x8000;
	signed int Aisign=UNPACKI(A)&0x8000;
	signed int Brsign=UNPACKR(B)&0x8000;
	signed int Bisign=UNPACKI(B)&0x8000;
 	signed int Crr;
	signed int Cir;
	signed int Cri;
	signed int Cii;       

	Number Cr, Ci;
 
        if(Arsign)
        {
          Ar = (~Ar + 1) & 0xffff;
        }
        if(Aisign)
        {
          Ai = (~Ai + 1) & 0xffff;
        }
        if(Brsign)
        {
          Br = (~Br + 1) & 0xffff;
        }
        if(Bisign)
        {
          Bi = (~Bi + 1) & 0xffff;
        }

        unsigned int ArBr = (Ar*Br);
        unsigned int AiBi = (Ai*Bi);
        unsigned int AiBr = (Ai*Br);
        unsigned int ArBi = (Ar*Bi);

        
        Crr = ArBr;
        Cri = ArBi;
        Cir = AiBr;
        Cii = AiBi;

        if(Arsign ^ Brsign)
          Crr = (~Crr + 1);
        if(Aisign ^ Brsign )
          Cir = (~Cir + 1);       
        if(Bisign ^Arsign )
          Cri = (~Cri + 1);
        if(Aisign ^Bisign )
          Cii = (~Cii + 1);

        Crr = (Crr >> 14) & 0xffff;
        Cri = (Cri >> 14) & 0xffff;
        Cir = (Cir >> 14) & 0xffff;
        Cii = (Cii >> 14) & 0xffff;

	Cr=Crr-Cii;
	Ci=Cri+Cir;	

	return PACK(Cr,Ci);
}
#endif

static inline Number complexAdd(Number A, Number B) {
	// A, B and the return value use the same 2's complement fixed-point format
	signed long Ar=UNPACKR(A);
	signed long Ai=UNPACKI(A);
	signed long Br=UNPACKR(B);
	signed long Bi=UNPACKI(B);
	Number Cr, Ci;

	Cr=Ar+Br;
	Ci=Ai+Bi;

	assert(!OVERFLOW(Cr));
	assert(!OVERFLOW(Ci));

	return PACK(Cr,Ci);
}

static inline Number complexRandom() {
	Number Cr, Ci;
	unsigned long x=0;

	// no random in EDK stdlib
	Cr=(Number)(rand()%1024)-512;
	Ci=(Number)(rand()%1024)-512;

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
	assert(A != NULL);
	assert(B != NULL);
	assert((N > 0) && (N <= 0x4000));
	assert(C != NULL);
	
	// perform the matrix-matrix multiply on the whole matrix at once
	mmmKernel(A, B, C, N, N);
	
}


int main(int argc, char** argv) {

  int size, seed, i;
  Number *A, *B, *C;
  FILE* Af, *Bf, *Cf, *Rf;


  size = atoi(argv[1]);
  seed = atoi(argv[2]);
  srand(seed);
  
  A = ((Number*)malloc(size*size*sizeof(Number)));
  B = ((Number*)malloc(size*size*sizeof(Number)));
  C = ((Number*)malloc(size*size*sizeof(Number)));
  for(i = 0; i < size*size; i++) {
    A[i] = complexRandom();
    B[i] = complexRandom();
  }
  mmm(A,B,C,size);

  Af = fopen("matrixA.hex","w");
  Bf = fopen("matrixB.hex","w");
  Cf = fopen("golden.hex","w");
  Rf = fopen("rowSize.hex","w");

  for(i = 0; i < 16; i++) {
    fprintf(Rf,"%08x\n",size);  
  }

  for(i = 0; i < size*size; i++) {
    fprintf(Af,"%08x\n",((int*)A)[i]);
    fprintf(Bf,"%08x\n",((int*)B)[i]);
    fprintf(Cf,"%08x\n",((int*)C)[i]);
  }
}
