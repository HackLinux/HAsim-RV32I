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
// Author: Muralidaran Vijayaraghavan
//
//----------------------------------------------------------------------//

#include <iostream>
#include <fstream>
#include <math.h> 
#include <stdio.h> 
#include <stdlib.h> 

#define maxN 1000

int main (int argc, char* argv [])
{
   srand (time (NULL));

   FILE* output_byte_stream;

   std::ofstream file (argv [1]);
   if (false == file.is_open ())
   {
      std::cout << "[ERROR]  Failed to open output file '" << argv [3] << "'." << std::endl;
      return 1;
   }
   file << "import Types::*;" << std::endl;
   file << "import List::*;" << std::endl;
   file << "import Vector::*;" << std::endl;
   file << "import FixedPointNew::*;" << std::endl;
   file << std::endl;
   file << "function TData getInverse(Index n);" << std::endl;
   file << "    Vector#(" << maxN << ", TData) tempV = Vector::toVector(" << std::endl;
   file << "            List::cons(0," << std::endl;

   int iByteCount = maxN;
   for (int i = 1; i < iByteCount; ++ i)
      file << "            List::cons(fromRational(1," << i << ")," << std::endl;
   

   file << "   List::nil";
   iByteCount = maxN;
   for (int i = 0; i <= iByteCount; ++ i)
      file << ")";
   file << ";" << std::endl;
   file << "   return tempV[n];" << std::endl;
   file << "endfunction" << std::endl;
}
