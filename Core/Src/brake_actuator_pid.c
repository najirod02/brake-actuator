#include "brake_actuator_pid.h"

#include <math.h>
#include <stdbool.h>

float pid_prev_errors[N_PID_PREV_ERRORS];
PidController_t pid_controller;

bool brake_actuator_enabled = false;

void brake_actuator_update_set_point(float set_point) {
    pid_controller.set_point = set_point;
}

bool brake_actuator_is_enabled() {
    return brake_actuator_enabled;
}

void brake_actuator_enable() {
    pid_reset(&pid_controller);
    brake_actuator_enabled = true;
}

void brake_actuator_disable() {
    brake_actuator_enabled = false;
    brake_actuator_set_speed(0.0);
}

void brake_actuator_set_speed(float speed)
{   
    float brake_distance = get_distance_from_counter(&htim2);

    if (fabs(brake_distance) > BRAKE_ACTUATOR_BRAKING_LIMIT) speed = 0.0;

    if (fabs(speed) > BRAKE_ACTUATOR_SPEED_LIMIT){
        if (speed > 0.0) speed = BRAKE_ACTUATOR_SPEED_LIMIT;
        else speed = -BRAKE_ACTUATOR_SPEED_LIMIT;
    }
    
    HAL_GPIO_WritePin(ActuatorDir_GPIO_Port, ActuatorDir_Pin, speed < 0.0 ? 0 : 1);

    // set new pwm frequency leaving constant the duty cycle ~ 50% of the new arr
    uint32_t arr = (uint32_t)((84e6 / (fabs(speed) / MM_STEP)) - 1);
    if (arr > 0xFFFF) arr = 0xFFFF; // maximum 16 bit value
    if (arr < 10) arr = 10; // prevent high frequencies
    TIM3->ARR = arr;
    TIM3->CCR1 = arr / 2; // to set 50% duty cycle
}

void brake_actuator_pid_init(float kp, float ki, float kd, float sample_time, float anti_windup) {
    pid_init(&pid_controller, kp, kd, ki, sample_time, anti_windup, pid_prev_errors, N_PID_PREV_ERRORS);
}

void brake_actuator_update_pid() {
    if (brake_actuator_enabled) {
        pid_update(&pid_controller, get_distance_from_counter(&htim2));
    }
}

void brake_actuator_update_speed() {
    if (brake_actuator_enabled) {
        float speed = pid_compute(&pid_controller);
        brake_actuator_set_speed(speed);
    }
}