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

#define MAX_PRESSURE 0.8f //bar - how much pressure the actuator can "generate"

#define UART_LINE_MAX 32

typedef enum {
    UART_WAIT_TYPE,
    UART_WAIT_PRESSURE,
    UART_WAIT_COMMAND
} UartRxState_t;

void brake_actuator_update_set_point(float setPoint);

bool brake_actuator_is_enabled();

void brake_actuator_enable();

void brake_actuator_disable();

void brake_actuator_set_speed(float speed);

void brake_actuator_pid_init(float kp, float ki, float kd, float sample_time, float anti_windup);

void brake_actuator_update_pid();

void brake_actuator_update_speed();

/**
 * keep reeading until an unknown command is read or a line feed '\n' is read
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif
