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

typedef 8 IntegralWidth;
typedef 2 IntWidthTheta;
typedef 14 IntWidthLong;

typedef 43 FractionWidth;
typedef 49 FracWidthTheta;
typedef 28 FracWidthLong;

//typedef `FRACTION_WIDTH FractionWidth; //43
//typedef `FRAC_WIDTH_THETA FracWidthTheta; //49
//typedef `FRAC_WIDTH_LONG FracWidthLong; //28

Integer cordicCosSinIters  = valueOf(FracWidthTheta) + 1;
Integer cordicCosSinStages = 1;
Integer cordicDivIters     = valueOf(IntWidthLong) + valueOf(FracWidthLong) - 1;
Integer cordicDivStages    = 1;

typedef FixedPoint#(IntegralWidth, FractionWidth) Data;
typedef FixedPoint#(IntWidthTheta, FracWidthTheta) TData;
typedef FixedPoint#(IntWidthLong, FracWidthLong) LData;
typedef 10 IndexWidth;
typedef Bit#(IndexWidth) Index;
typedef FixedPoint#(TAdd#(IndexWidth,1), 0) IntData;

typedef struct {
    Index maxN;
    Data r;
    TData theta;
} TopInit deriving (Bits, Eq);

typedef struct {
    Index maxN;
    Data r;
    Data rCos;
    TData cosDTheta;
    TData sinDTheta;
    TData dr;
    LData invDeltaX; // = maxN/(r+1 - rCos)
    LData invDeltaY; // = maxN/((r+1)Sin)
} ThetaInit deriving (Bits, Eq);

typedef struct {
    Index maxN;
    Data x;
    Data y;
    TData dx;
    TData dy;
    LData invDeltaX;
    LData invDeltaY;
} RayInit deriving (Bits, Eq);

typedef struct {
    Index x;
    Index y;
} Pos deriving (Bits, Eq);
