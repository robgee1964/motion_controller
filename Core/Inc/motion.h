 /*
 * motion.h
 *
 *  Created on: Feb 8, 2023
 *      Author: rob
 */

#ifndef MOTION_H_
#define MOTION_H_

#include <stdint.h>

typedef struct t_motion_ctx* pt_motion_ctx;


double* motion_plan_df(uint16_t n_steps, uint16_t max_accel, uint16_t max_speed);
float* motion_plan_sf(uint16_t s_target, uint16_t max_accel, uint16_t max_speed);
uint32_t motion_start_fp(uint16_t s_target, uint16_t max_accel, uint16_t max_speed);
uint32_t motion_step(void);


void motion_complete(void);




#endif /* MOTION_H_ */
