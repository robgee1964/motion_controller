 /*
 * motion.h
 *
 *  Created on: Feb 8, 2023
 *      Author: rob
 */

#ifndef MOTION_H_
#define MOTION_H_

#include <stdint.h>


double* motion_plan(uint16_t n_steps, uint16_t max_accel, uint16_t max_speed);
uint32_t* motion_plan_fp(uint16_t s_target, uint16_t max_accel, uint16_t max_speed);


void motion_complete(void);




#endif /* MOTION_H_ */
