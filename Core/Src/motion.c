/**
 * @file motion.c
 *
 *  Created on: Feb 8, 2023
 * @author rob
 */


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "arm_math.h"
#include "maths.h"
#include "motion.h"

#define F_CLOCK     72000000UL
#define ARR         4UL
#define SPEED_MIN   30UL

// Timer tick will be expressed as a fraction of 2UL^31
#define T_TICK_DIV  (F_CLOCK)/(ARR+1)
#define T_TICK_NUM  (((1UL<<31)+(T_TICK_DIV)/2)/(T_TICK_DIV))

#define SPEED_MIN_TICKS \
        ((F_CLOCK)/((ARR+1)*(SPEED_MIN)))


static void *p_motion_buff;

// PSC =
/**
 * @brief Works out a motion plan, using sinusoidal profile
 *
 * For a sinusoidal speed profile, with no contant speed portion, the acceleration is given by\n
 * \f$a(t)=a_{max}sin(\omega t)\f$\n\n
 * Integrating for speed\n
 * \f$
 * \eta(t) = \frac{a_{max}}{\omega }\left ( 1-cos(\omega t) \right )
 * \f$\n\n
 * Maximum speed will occur when
 * \f$\omega t = 2\pi \f$\n
 * hence\n
 * \f$
 * \eta _{max} = \frac{2a_{max}}{\omega}\f$\n
 * and thus \f$
 * \omega = \frac{2a_{max}}{\eta_{max}}
 * \f$\n\n
 * Integrating again for speed, we get\n
 * \f$
 * s(t)=\frac{a_{max}}{\omega}\left ( t-\frac{sin(\omega t)}{\omega} \right )
 * \f$\n\n
 * Subsituting \f$t = \frac{2\pi}{\omega}\f$ the mininum disance to reach specified max speed and accel (\f$\eta_{max}, a_{max}\f$) \n
 * \f$s_{min=}\pi\frac{\eta_{max}^{2}}{2a_{max}}\f$\n\n
 * For movements less than \f$s_{min}\f$, the target speed will need to be reduced below \f$\eta_{max}\f$, in order for a complete sinusoidal \n profile to be acheived
 *
 *
 */
double* motion_plan(uint16_t s_target, uint16_t max_accel, uint16_t max_speed)
{
  // For a sinusoidal speed profile, the accel is given by
  // a(t) = a*sin(w*t)
  // the speed is given by
  // n(t) = a/w(1-cos(w*t))
  // nmax = 2a/w, hence w = 2a/nmax
  // Assuming no constant speed portion, then the distance is given by
  // s(t) = k/w(t - 1/(w*sin(w*t)))
  // s = (pi*nmax*nmax)/(2a)
  uint16_t s_idx = 0;
  uint16_t s_accel = 0;
  // caclulate s_min, the distance required to carry out a motion reaching
  // the specified accel and decel
  double s_min = (pow((double)max_speed, 2) * M_PI_2)/(double)max_accel;
  printf("Moving %u steps, %u required for max speed\n",
                          s_target, (uint16_t)s_min);

  double n_max;
  if ((double)s_target < s_min)
  {
    n_max = sqrt(M_2_PI * (double)max_accel * (double)s_target);
  }
  else
  {
    n_max = (double)max_speed;
  }
  double w = 2 * (double)max_accel/(n_max - (double)SPEED_MIN);
  double a = (double)max_accel;
  double n = (double)SPEED_MIN;
  double t = 0;
  double dt = 0;
  p_motion_buff = malloc(s_target*sizeof(double));
  double *p_buff = (double*)p_motion_buff;
  double n_prev = 0;

  // We only ever need to calculate the accel phase, since decel will simply be the reverse
  while(s_idx < s_target)
  {
    *p_buff++ = n;
    dt = 1/n;
    t += dt;
    n = (double)SPEED_MIN+(a*(1-cos(w*t))/w);
    if (n < n_prev)
    {
      // Peak has been detected, discard last sample
      s_accel = s_idx+1;
      n = n_prev;
      printf("Peak speed %.2f detected at %u steps\n", n, s_accel);
      break;
    }
    s_idx++;
    n_prev = n;
  }

  // Check if constant speed section is required
  int16_t s_cv = s_target - (2 * s_accel);
  if (s_cv > 0)
  {
    printf("%d steps at constant speed of %.2f\n", s_cv, n);
    while(s_cv-- > 0)
    {
      s_idx++;
      *p_buff++ = n;
    }
  }

  // Finally, write out the decel
  for (int16_t d_step = s_accel-1; d_step >= 0; d_step--)
  {
    s_idx++;
    *p_buff++ = ((double*)p_motion_buff)[d_step];
  }

  return (double*)p_motion_buff;
}


/**
 * @brief Motion planner using fixed point arithmetic
 *
 * The arm maths library includes fixed point implementations for various trig functions
 * The input is scaled so that @c0.99999 = \f$2\pi\f$\n
 * This means for the q31 implementation that @c0x7ffffff=\f$2\pi\f$\n
 *
 * Run times  steps             accel       speed          us
 *             160              2000        500            368
 *             600              2000        600            472
 *             600              2500        500            436
 *             120              2200        500            336
 *
 * Based on the above run times, the speed profile could be calculated on the fly
 * inside the ISR. The only thing to watch, could be rounding errors causing a different
 * number of decel pulses, to accel. Thus the motion would not come to rest at minimum speed
 * or too many steps at minimum speed.
 *
 * @param s_target
 * @param max_accel
 * @param max_speed
 * @return
 */
uint32_t* motion_plan_fp(uint16_t s_target, uint16_t max_accel, uint16_t max_speed)
{

  p_motion_buff = malloc(s_target*sizeof(uint32_t));
  uint32_t *p_buff = (uint32_t*)p_motion_buff;

  // See if are enough steps to acheive the specified speed (113/710 is pi/2))
  uint32_t temp = (uint32_t)max_speed;
  temp *= (uint32_t)temp;

  uint32_t s_min = (temp * 355)/
                    ((uint32_t)max_accel * 226UL);

  uint32_t nmax;
  if (s_target < s_min)
  {
#if 0
    uint32_t temp = (226UL * (uint32_t)max_accel * (uint32_t)s_target)/355UL;
    nmax = sqrt_fp(temp) - SPEED_MIN;
#else
    // *2 so q31_sqrt can have result right shifted by 16 places
    temp = (452UL * (uint32_t)max_accel * (uint32_t)s_target)/355UL;
    arm_sqrt_q31(temp, (q31_t*)&nmax);
    nmax >>= 16;
#endif
  }
  else
  {
    nmax = (uint32_t)max_speed - SPEED_MIN;
  }


  // Use timer ticks of
  // n = (nmax/2)*(1-cos(wt))
  // ARM lib trig functions are scaled so that 2^31 input is 2pi
  // So our multiplier is 2^31/2pi
  // overall multipler is (w*2^31*T_TICK_NUM)
  //                      -------------------
  //                       2pi * 2^31
  uint32_t t_mult = (uint32_t)max_accel*T_TICK_NUM*113;       // pi == 355/113
  t_mult += (nmax * 355)/2;
  t_mult /= (nmax * 355);

  // say max accel = 20000, so t_mult(max) = ((20000*149*113)+355/2)/355
  // So tMult(max) = 948,563

  uint32_t ticks = SPEED_MIN_TICKS;
  uint32_t n_prev = 0;
  uint32_t s_idx = 0;
  uint32_t s_accel = 0;
  uint32_t n = SPEED_MIN<<8;                // q8
  bool accel_done = false;

  while(s_idx < s_target)
  {
    // Loop takes 1 multiple, 1 divide plus whatever is in the trig function
    // (apparently a table lookup + linear interp)
    // Interesting to see what is the run time - if quick enough the pulses could
    // be generated in real time inside the timer OC interrupt, for this to be feasible
    // calculation wants to be < 40us (based on say 5000Hz max stepper speed)

    // ticks * tmult must not exceed 2^31, however greater accel will have less accel/decel
    // ticks so everything should look after itself
    *p_buff++ = n;
    uint32_t trig_operand = ticks * t_mult;
    if (trig_operand > 0x7fffffff)
    {
      accel_done = true;
    }
    uint32_t temp = (uint32_t)arm_cos_q31((q31_t)trig_operand);
    temp = 0x7fffffff-temp;
    // This should always be positive, therefore change to q14 by doing a right shift 17 places
    temp >>= 15;      // Change scaling to q16 to prevent overflow
    // Work out the speed
    n = nmax/2 * temp;         // q16
    n >>= 8;                           // q8
    n += (SPEED_MIN<<8);
    if ((n < n_prev) || (accel_done))
    {
      s_accel = s_idx+1;
      n = n_prev;
      printf("Peak speed %lu detected at %u steps\n", n>>8, s_accel);
      break;
    }
    // We want to go from speed to pulse, so its 1/speed
    // dticks is used to schedule the next pulse, PSC, ARR and CCR1 will needed to be
    // adjusted to that PSC is <= 65535
    uint32_t dticks = (((T_TICK_DIV)<<8) + (n>>1))/n;
    s_idx++;
    ticks += dticks;
    n_prev = n;
  }

  // Check if constant speed section is required
  int16_t s_cv = s_target - (2 * s_accel);
  if (s_cv > 0)
  {
    printf("%d steps at constant speed of %lu\n", s_cv, n>>8);
    while(s_cv-- > 0)
    {
      s_idx++;
      *p_buff++ = n;
    }
  }

  // Finally, write out the decel
  for (int16_t d_step = s_accel-1; d_step >= 0; d_step--)
  {
    s_idx++;
    *p_buff++ = ((uint32_t*)p_motion_buff)[d_step];
  }

  return (uint32_t*)p_motion_buff;

}

void motion_complete(void)
{
  if (p_motion_buff != NULL)
  {
    free(p_motion_buff);
  }
}



