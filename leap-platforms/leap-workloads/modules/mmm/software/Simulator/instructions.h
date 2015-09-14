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

#ifndef _INSTRUCTIONS_H_
#define _INSTRUCTIONS_H_

#include "instructionParameters.h"

// Common header for instruction macros

typedef enum instruction {
    ArithmeticInstruction,
    LoadInstruction,
    StoreInstruction,
    ForwardInstruction,
    SetRowSizeInstruction
} Instruction;

typedef enum functionalUnitOp {
    Multiply,
    Zero,
    MultiplyAddAccumulate,
    MultiplySubAccumulate,
    AddAccumulate,
    SubAccumulate  
} FunctionalUnitOp;

typedef enum matrixRegister {
    A,
    B,
    C
} MatrixRegister;

typedef long long UInt64;

#define ExpFunctionalUnitNumber ((UInt64)1<<FunctionalUnitNumber)
#define ExpLogFunctionalUnitNumber ((UInt64)1<<LogFunctionalUnitNumber)
#define ExpRegSize ((UInt64)1<<RegSize)
#define ExpArithOpSize ((UInt64)1<<ArithOpSize)
#define ExpOpCodeSize ((UInt64)1<<OpCodeSize)
#define ExpAddrSize ((UInt64)1<<AddrSize)
#define ExpLogRowSize ((UInt64)1<<LogRowSize)
#define ExpTotalSize ((UInt64)1<<TotalSize)

#define OpCodePosition      ((UInt64)(TotalSize-OpCodeSize))

#define FUMaskPosition      ((UInt64)(ArithOpSize))
#define ArithOpPosition     ((UInt64)(0))

#define MemFUMaskPosition   ((UInt64)(MemRegNamePosition+RegSize))
#define MemRegNamePosition  ((UInt64)(AddrSize))
#define MemAddrPosition     ((UInt64)(0))

#define FUSrcPosition       ((UInt64)(RegSrcPosition+RegSize))
#define RegSrcPosition      ((UInt64)(FUDestMaskPosition+FunctionalUnitNumber))
#define FUDestMaskPosition  ((UInt64)(RegSize))
#define RegDestPosition     ((UInt64)(0))

#define RowSizePosition     ((UInt64)(0))

#define createArithmeticInstruction(fus, op) \
    ((((UInt64)1)<<63) |\
     (ArithmeticInstruction%ExpOpCodeSize)<<OpCodePosition |\
     (fus%ExpFunctionalUnitNumber)<<FUMaskPosition |\
     (op%ExpArithOpSize)<<ArithOpPosition)

#define createLoadInstruction(fus, regName, addr) \
    ((((UInt64)1)<<63) |\
     (LoadInstruction%ExpOpCodeSize)<<OpCodePosition |\
     (fus%ExpFunctionalUnitNumber)<<MemFUMaskPosition |\
     (regName%ExpRegSize)<<MemRegNamePosition |\
     (addr%ExpAddrSize)<<MemAddrPosition)

#define createStoreInstruction(fus, regName, addr) \
    ((((UInt64)1)<<63) |\
     (StoreInstruction%ExpOpCodeSize)<<OpCodePosition |\
     (fus%ExpFunctionalUnitNumber)<<MemFUMaskPosition |\
     (regName%ExpRegSize)<<MemRegNamePosition |\
     (addr%ExpAddrSize)<<MemAddrPosition)

#define createForwardInstruction(fuSrc, regSrc, fuDest, regDest) \
    ((((UInt64)1)<<63) |\
     (ForwardInstruction%ExpOpCodeSize)<<OpCodePosition |\
     (fuSrc%ExpLogFunctionalUnitNumber)<<FUSrcPosition |\
     (regSrc%ExpRegSize)<<RegSrcPosition |\
     (fuDest%ExpFunctionalUnitNumber)<<FUDestMaskPosition |\
     (regDest%ExpRegSize)<<RegDestPosition)

#define createSetRowSizeInstruction(rowSize) \
    ((((UInt64)1)<<63) |\
     (SetRowSizeInstruction%ExpOpCodeSize)<<OpCodePosition |\
     (rowSize%ExpLogRowSize)<<RowSizePosition)

#endif //_INSTRUCTIONS_H_

