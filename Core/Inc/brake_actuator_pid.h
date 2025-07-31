#ifndef BRAKE_ACTUATOR_H
#define BRAKE_ACTUATOR_H

#define PID_ERRORS_VECTOR
#define N_PID_PREV_ERRORS 5

#include <stdbool.h>
#include "pid.h"
#include "tim.h"
#include "encoder.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "usart.h"

#define BRAKE_ACTUATOR_SPEED_LIMIT 30.0 //mm/s - maximum speed of the actuator

#define EXTEND GPIO_PIN_RESET // the actutor "extends" (cw rotation w.r.t. back of actuator)
#define RETRACT GPIO_PIN_SET // the actuator "retracts" (ccw rotation w.r.t. back of actuator)

#define MOTOR_GO GPIO_PIN_RESET // the driver is enabled, a step command will be accepted
#define MOTOR_STOP GPIO_PIN_SET // the driver is disabled, any step command will be discarded

#define MAX_PRESSURE 10.0f //bar - how much pressure the brake pedal can "generate" at most

#define DEADBAND 0.2f //bar - how much we need to be near the target point before stopping the pwm

#define TIMER_CLOCK 84e6 // the clock of the timer, needed to compute the AAR, CCR

#define UART_TIMEOUT_MS 500 // ms - stop the actuator if not receiving any mesasge
#define UART_LINE_MAX 32

extern uint32_t last_uart_msg_time;

void brake_actuator_update_set_point(float setPoint);

bool brake_actuator_is_enabled();

void brake_actuator_enable();

void brake_actuator_disable();

void brake_actuator_set_speed(float speed);

void brake_actuator_pid_init(float kp, float ki, float kd, float sample_time, float anti_windup);

void brake_actuator_update_pid();

void brake_actuator_update_speed();

/*!
 * \brief keep reeading until an unknown command is read or a line feed '\n' is read
 * 
 * \warning in case an unknown command is received or it cannot be parse, the actuator will be disabled
 * for security in order to avoid damages to the brake pedal and/or actuator
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif
