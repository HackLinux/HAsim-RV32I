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

Author: Michael Pellauer
*/

#include <stdio.h>
#include <stdlib.h>

#define RowSize 64

#define Tests 1

inline void to_mem(int *m, int k)
{
    int i,j;

    for(i = 0; i < RowSize; i++)
    {
        for(j = 0; j < RowSize; j++)
        {
            *(m+2*RowSize*RowSize*k+i*RowSize+j) = k*25+100 + i + j;
            //fprintf(stdout, "%x %d %d %d\n", m+2*RowSize*RowSize*k+i*RowSize+j, i, j, *(m+2*RowSize*RowSize*k+i*RowSize+j));
        }
    }
}

inline void from_mem(int *m, int k)
{
    int i,j;

    for(i = 0; i < RowSize; i++)
    {
        for(j = 0; j < RowSize; j++)
        {
            if(*(m+2*RowSize*RowSize*k+i*RowSize+j) != k*25+100 + i + j)
            {
                fprintf(stdout, "Error: %x %d %d %d\n", m+2*RowSize*RowSize*k+i*RowSize+j, i, j, *(m+2*RowSize*RowSize*k+i*RowSize+j));
            }
        }
    }
}

int main()
{
    int f = 0;
    int count = 0;

    int *matToMem   = (int *)malloc(20000000);

    to_mem(matToMem, 0);
    int t=1;

    while(count < 1000)
    {
        int random = rand()%2;
        if(random == 0)
        {
            to_mem(matToMem, t);
            t++;
        }
        else
        {
            from_mem(matToMem, f);
            if(f < t-1)
                f++;
        }
        count++;
    }    
}
