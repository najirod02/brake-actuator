#ifndef ENCODER_H
#define ENCODER_H

#include "tim.h"

/**
 * given that the phase angle of the actuator is 1.8°. 200 steps are needed for a full rotation
 * with the given information of the encoder we can derive the relation tick/mm
 * 
 * total encoder counts per revolution = 16384
 * steps per revolution = 200
 * encoder counts per actuator step ~ 82 counts/steps (81.92)
 * distance per encoder count 0.122 µm/step
 * 
 */

#define MM_STEP (0.01f) // mm/step actuator
#define ENC_MM_TICK (1.220703125e-4f) // mm/tick encoder

#define ENC_BRAKE_FREQ_HZ   200                      /* 200Hz */
#define ENC_BRAKE_PERIOD_MS 1000 / ENC_BRAKE_FREQ_HZ /* 5ms */

#define PPR 65536
#define CPR 16384

/**
 * relative counter with reference to the actual rotation
 */
uint32_t get_relative_counter(TIM_HandleTypeDef *htim);

/**
 * returns the number of ticks read by the timer
 * unsigned int
 */
uint32_t get_absolute_counter(TIM_HandleTypeDef *htim);

/**
 * allows to obtain also non positive counter values
 */
int32_t get_signed_counter(TIM_HandleTypeDef *htim);

/**
 * given the number of ticks that is the counter, returns
 * the distance in mm
 */
float get_distance(int32_t counter);

/**
 * read from timer the counter and convert
 * it into distance mm
 */
float get_distance_from_counter(TIM_HandleTypeDef *htim);

#endif