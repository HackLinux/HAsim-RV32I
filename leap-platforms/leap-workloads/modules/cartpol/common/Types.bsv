//----------------------------------------------------------------------//
// The MIT License 
// 
// Copyright (c) 2009 MIT
// 
// Permission is hereby granted, free of charge, to any person 
// obtaining a copy of this software and associated documentation 
// files (the "Software"), to deal in the Software without 
// restriction, including without limitation the rights to use,
// copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
// HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
//
// Author: Abhinav Agarwal, Nirav Dave, Muralidaran Vijayaraghavan
//
//----------------------------------------------------------------------//

import FixedPointNew::*;

typedef 14 IntegralWidth;
typedef 36 FractionWidth; // Pre near integral check val = 40
typedef  2 IntWidthTheta;
typedef 42 FracWidthTheta; // If changed, rerun mkLutInv to regen lutInv.txt, Pre val = 48
typedef  8 IntWidthData;
typedef 36 FracWidthData;
typedef 10 IndexWidth;

Integer cordicCosSinIters  = valueOf(FracWidthTheta) + 1;
Integer cordicCosSinStages = 1;
Integer cordicDivIters     = valueOf(IntegralWidth) + valueOf(FractionWidth) - 1;
Integer cordicDivStages    = 1;
Integer angFifoSz          = 4;

typedef FixedPoint#(IntWidthData, FracWidthData) Data;
typedef FixedPoint#(IntegralWidth, FractionWidth) LData;
typedef FixedPoint#(IntWidthTheta, FracWidthTheta) TData;
typedef Bit#(IndexWidth) Index;

typedef struct {
   Index maxN;
   Data r;
   TData dR;
   ThetaTrig trig;
   Data rCos;
   LData invDeltaX;
   LData invDeltaY;
} RayInit deriving (Bits, Eq);

typedef struct {
  TData cosA;
  TData sinA;
} ThetaTrig deriving (Bits, Eq);

typedef struct {
   Index x;
   Index y;
} Pos deriving (Bits, Eq);
