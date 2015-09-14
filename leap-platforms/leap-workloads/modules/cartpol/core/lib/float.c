#include <math.h>
#include <stdlib.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

unsigned long long cAdd(unsigned long long x, 
                        unsigned long long y)
{
   double double_x, double_y, double_res;
   unsigned long long res;
   memcpy(&double_x,&x,sizeof(double));
   memcpy(&double_y,&y,sizeof(double));
   double_res = double_x + double_y;
   memcpy(&res,&double_res,sizeof(double));
   return res;
}

unsigned long long cSub(unsigned long long x, 
                        unsigned long long y)
{
   double double_x, double_y, double_res;
   unsigned long long res;
   memcpy(&double_x,&x,sizeof(double));
   memcpy(&double_y,&y,sizeof(double));
   double_res = double_x - double_y;
   memcpy(&res,&double_res,sizeof(double));
   return res;
}

unsigned long long cMul(unsigned long long x, 
                        unsigned long long y)
{
   double double_x, double_y, double_res;
   unsigned long long res;
   memcpy(&double_x,&x,sizeof(double));
   memcpy(&double_y,&y,sizeof(double));
   double_res = double_x * double_y;
   memcpy(&res,&double_res,sizeof(double));
   return res;
}

unsigned char cIsSmaller(unsigned long long x,
                         unsigned long long y)
{
   double double_x, double_y;
   memcpy(&double_x,&x,sizeof(double));
   memcpy(&double_y,&y,sizeof(double));
   return (double_x < double_y) ? 1 : 0;
}

unsigned char cIsLarger(unsigned long long x,
                        unsigned long long y)
{
   double double_x, double_y;
   memcpy(&double_x,&x,sizeof(double));
   memcpy(&double_y,&y,sizeof(double));
   return (double_x > double_y) ? 1 : 0;
}
