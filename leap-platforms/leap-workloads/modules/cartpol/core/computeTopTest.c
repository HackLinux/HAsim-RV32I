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
// Author: Abhinav Agarwal, Asif Khan, Muralidaran Vijayaraghavan
//
//----------------------------------------------------------------------//

#include <math.h>
#include <stdlib.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

#define PI (4*atan2(1,1))
#define PRECISION (1.0/32768)

static unsigned int pol_array[1024*1024];
static unsigned char chk_array[1024*1024];

unsigned int readCartArray(unsigned int x_idx,
                           unsigned int y_idx)
{
//   printf("readCartArray[%d][%d] returns %x %d\n",x_idx,y_idx,(x_idx<<18)+(y_idx<<2),(x_idx<<18)+(y_idx<<2));
  return x_idx+(y_idx*16384);
}

unsigned int readPolArray(unsigned int arr_idx)
{
  printf("readPolArray[%d]\n",arr_idx);
  return (arr_idx >= 1024*1024) ? 0 : pol_array[arr_idx];
}

unsigned int writePolArray(unsigned int arr_idx,
                           unsigned int val)
{
   pol_array[arr_idx] = val;
   return 0;
}

unsigned char readChkArray(unsigned int arr_idx)
{
  return (arr_idx >= 1024*1024) ? 1 : chk_array[arr_idx];
}

unsigned int signExtend(unsigned int a)
{
   unsigned int mask = 0x00008000;
   unsigned int sign_extend = 0x00030000;
   return ((a & mask) > 0) ? a + sign_extend : a;
}

unsigned int cGetPos(unsigned char      reset_val,
                     unsigned long long rad,
                     unsigned long long ang,
                     unsigned int       n, 
                     unsigned int       r_idx,
                     unsigned int       c_idx)
{
   static unsigned int pol_arr_idx;
   static double row[1024]; // store r interpolated value
   static double col[1024]; // store theta interpolated value
   static double stored_r;
   static double x_scaling; // x scaling factor to get index
   static double y_scaling; // y scaling factor to get index
   static unsigned int max_n;
   unsigned char chk_x,chk_y;
   double x, y, long_rad, long_ang, long_n, inverse;
   unsigned int i, out_x, out_y, return_val, x_int, y_int, pol_array_val, x0y0, x0y1, x1y0, x1y1, pol_array_val_x, pol_array_val_y;
   unsigned int x0y0_rel, x0y0_img, x1y0_rel, x1y0_img, x0y1_rel, x0y1_img, x1y1_rel, x1y1_img;
//   printf("reset: %d\n", reset_val);
   if (reset_val)
   {
      //printf("rad %d ang %d rad_shf %d ang_shf %d \n",rad,ang,rad_shf,ang_shf);
      //printf("rad %d ang %d rad_shf %d ang_shf %d \n",rad,ang,rad_shf,ang_shf);
//      long_rad = ((double) rad)/((double)pow(2,rad_shf));
//      long_ang = ((double) ang)/((double)pow(2,ang_shf));
      memcpy(&long_rad,&rad,sizeof(double));
      memcpy(&long_ang,&ang,sizeof(double));
      long_n   = ((double) n-1);
      pol_arr_idx = 0;
      max_n   =  n-1;
//      printf("rad %g ang %g n %g maxN %d\n",long_rad,long_ang,long_n, max_n);
      stored_r = long_rad * cos(long_ang);
      x_scaling = long_n/(long_rad + 1 - (long_rad * cos(long_ang))); 
      y_scaling = long_n/((long_rad + 1) * sin(long_ang));
//      printf("x_scaling %g y_scaling %g \n",x_scaling, y_scaling);
      inverse = 0;
      for (i = 0; i < n; ++i)
      {
         row[i] = long_rad + inverse;
         col[i] = long_ang * inverse;
         //printf("x_%d %g y_%d %g inverse %g \n",i,row[i],i,col[i],inverse);
         inverse += 1/long_n;
      }
   }
   x = row[r_idx] * cos(col[c_idx]);
   x -= stored_r;
   x *= x_scaling;
   chk_x = ((ceil(x) != ceil(x + PRECISION)) || (floor(x) != floor(x - PRECISION))) ? 2 : 0; 
//   printf("pos %d: PRECISION %g, chk_x %d, ceil(x) %g, ceil(x + PRECISION) %g, floor(x) %g, floor(x - PRECISION) %g x %g\n",pol_arr_idx,PRECISION,chk_x,ceil(x),ceil(x+PRECISION),floor(x),floor(x-PRECISION),x);
//   if (chk_x)
//   {
//      printf("pos %d x %g chk_x %d\n",pol_arr_idx,x,chk_x);
//   }
   x_int = (unsigned int)x;
   if ((x_int == max_n) && (max_n != 0))
   {
      printf("C:X = %d, Reached max index, Being reduced by one\n", max_n);
      x_int -= 1; 
   }
   out_x = (x_int)<<10;
   y = row[r_idx] * sin(col[c_idx]);
   y *= y_scaling; 
   chk_y = ((ceil(y) != ceil(y + PRECISION)) || (floor(y) != floor(y - PRECISION))) ? 1 : 0; 
//   printf("pos %d: PRECISION %g, chk_y %d, ceil(y) %g, ceil(y + PRECISION) %g, floor(y) %g, floor(y - PRECISION) %g y %g\n",pol_arr_idx,PRECISION,chk_y,ceil(y),ceil(y+PRECISION),floor(y),floor(y-PRECISION),y);
//   if (chk_y)
//   {
//      printf("pos %d y %g chk_y %d\n",pol_arr_idx,y,chk_y);
//   }
   chk_array[pol_arr_idx] = chk_x + chk_y; 
   y_int = (unsigned int)y;
   if ((y_int == max_n) && (max_n != 0))
   {
      printf("C:Y = %d, Reached max index, Being reduced by one\n", max_n);
      y_int -= 1; 
   }
   out_y = (y_int)%(1<<10);
   // printf("r_idx %d, c_idx %d, x %g y %g C\n", r_idx, c_idx, x,y);
   return_val = out_x + out_y;
//   printf("C: xxxx %d %d\n",x_int,y_int);
   x0y0 = readCartArray(x_int,y_int);
   x0y1 = readCartArray(x_int,y_int+1);
   x1y0 = readCartArray(x_int+1,y_int);
   x1y1 = readCartArray(x_int+1,y_int+1);
   x0y0_rel = signExtend(x0y0/65536);
   x0y0_img = signExtend(x0y0%65536);
   x0y1_rel = signExtend(x0y1/65536);
   x0y1_img = signExtend(x0y1%65536);
   x1y0_rel = signExtend(x1y0/65536);
   x1y0_img = signExtend(x1y0%65536);
   x1y1_rel = signExtend(x1y1/65536);
   x1y1_img = signExtend(x1y1%65536);
   pol_array_val_x = ((x0y0_rel + x0y1_rel + x1y0_rel + x1y1_rel) / 4) * 65536; // (/4 * 65536)
   pol_array_val_y = ((x0y0_img + x0y1_img + x1y0_img + x1y1_img) / 4) % 65536;
   //printf("xxxx Pol[%d][%d] = pol_array_val_x=%x, pol_array_val_y=%x\n",r_idx,c_idx,pol_array_val_x,pol_array_val_y);
   pol_array_val = pol_array_val_x + pol_array_val_y;
   pol_array[pol_arr_idx] = pol_array_val;
   //printf("xxxx readPolArray[%d] returns %x\n",pol_arr_idx,pol_array[pol_arr_idx]);
   pol_arr_idx++;
   return return_val;
}

/*
int main()
{
   unsigned int i, j;
   unsigned int n = 1000;
   double long_rad = 10;
   unsigned char rad_shf = 40; // fractional decision of long_rad
   unsigned long long rad = (unsigned long long) (long_rad * pow(2,rad_shf));
   double long_ang = PI/6;
   unsigned char ang_shf = 50; // fractional decision of long_ang
   unsigned long long ang = (unsigned long long) (long_ang * pow(2,ang_shf));
   unsigned int coord, coord_x, coord_y;
   unsigned int res, res_rel, res_img;
   for (i = 0; i < n; ++i)
      for (j = 0; j < n; ++j)
         if (i == 0 && j == 0) // initialization
         {
            coord = cGetPos(1,rad,rad_shf,ang,ang_shf,n,0,0);
            coord_x = coord>>10;
            coord_y = coord%(1<<10);
            res = readPolArray(i,j);
            res_rel = res>>16;
            res_img = res%(1<<16);
            if (res_rel != coord_x*4+2 || res_img != coord_y*4+2)
            {
               printf("Error! Wrong output!");
               return -1;
            }
//            printf("Pol[%d][%d] averages Cart[%d][%d] and gets rel = %d img = %d \n",i,j,coord_x,coord_y,res_rel,res_img);            
         }
         else
         {
            coord = cGetPos(0,rad,rad_shf,ang,ang_shf,n,i,j);
            coord_x = coord>>10;
            coord_y = coord%(1<<10);
            res = readPolArray(i,j);
            res_rel = res>>16;
            res_img = res%(1<<16);
            if (res_rel != coord_x*4+2 || res_img != coord_y*4+2)
            {
               printf("Error! Wrong output!");
               return -1;
            }
//            printf("Pol[%d][%d] averages Cart[%d][%d] and gets rel = %d img = %d \n",i,j,coord_x,coord_y,res_rel,res_img);            
         }
   printf("Testbranch passes\n");
   return 0;
}
*/
