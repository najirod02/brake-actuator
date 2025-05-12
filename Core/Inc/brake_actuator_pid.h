#ifndef BRAKE_ACTUATOR_H
#define BRAKE_ACTUATOR_H

#define PID_ERRORS_VECTOR
#define N_PID_PREV_ERRORS 5

#include <stdbool.h>
#include "pid.h"
#include "tim.h"
#include "encoder.h"

#define BRAKE_ACTUATOR_SPEED_LIMIT 5.0
#define BRAKE_ACTUATOR_BRAKING_LIMIT 100.0

void brake_actuator_update_set_point(float setPoint);

bool brake_actuator_is_enabled();

void brake_actuator_enable();

void brake_actuator_disable();

void brake_actuator_set_speed(float speed);

void brake_actuator_pid_init(float kp, float ki, float kd, float sample_time, float anti_windup);

void brake_actuator_update_pid();

void brake_actuator_update_speed();

#endif
