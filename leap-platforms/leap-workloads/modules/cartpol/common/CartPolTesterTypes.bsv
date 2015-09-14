/*
Copyright (c) 2009 MIT

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

*/

import Real::*;


// Kinda lame, oh well
typedef  TMul#(2,BufferSz) OutputBufferBase; // (1024 * 1024 * 4) * 2
Integer maxTests = 64000;


//Theta values 
Real maxTheta    = (pi/4);
Real minTheta    = (pi/256);	
Real thetaStep   = (pi/256);	


//R values
Real minR = 10.0;
Real maxR = 1000.0;
Real rStep = 0.5;

//N values 
Integer minN = 10;
Integer maxN = 1000;
Integer nStep = 1;
