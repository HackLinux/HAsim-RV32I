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
#include "cLib.h"

#define BramInstructionStart 0x00001111
#define BramOutputStart 0x10001111

int bramInstruction;
int bramOutput;

void initialize()
{
    bramInstruction = 0;
    bramOutput = 0;
}

void putInstruction(UInt64 x)
{
    UInt64 xShift = x >> 32;
    int x0 = (int)xShift;
    int x1 = (int)x;
    *(int *)(BramInstructionStart + bramInstruction) = x0;
    *(int *)(BramInstructionStart + bramInstruction + 1) = x1;
    bramInstruction += 2;
}

int getResponse()
{
    return *(int *)(BramOutputStart + bramOutput);
}
