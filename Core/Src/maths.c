/*
 * maths.c
 *
 *  Created on: Feb 11, 2023
 *      Author: rob
 */

#include <stdio.h>
#include "maths.h"


uint32_t sqrt_fp(uint32_t in)
{
  // Newton Rhaphson formula
  // x1 = 1/2(x0 + xin/x0)
  // start with x0 a fraction of xin
  // seems to work with integers, in spreadsheet
  uint32_t in_mul = in << 8;
  uint32_t diff;
  uint32_t x0 = in_mul >> 9;
  uint32_t x1;
  uint16_t n = 0;
  do
  {
    uint32_t t = (in_mul + (x0>>1))/x0;
    t += (1 + x0);
    x1 = t >> 1;
    diff = x0 - x1;
    x0 = x1;
    n++;
  }
  while (diff != 0);
  printf("Converged in %u loops\n", n);
  x0 += 8;

  return x0 >> 4;
}
