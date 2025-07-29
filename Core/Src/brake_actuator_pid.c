#include "brake_actuator_pid.h"

float pid_prev_errors[N_PID_PREV_ERRORS];
PidController_t pid_controller;

uint8_t msg[100] = {'\0'};
float actual_pressure = 0.0f;
uint8_t uart_line[UART_LINE_MAX];
uint8_t uart_index = 0;

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
    HAL_GPIO_WritePin(ActuatorEnable_GPIO_Port, ActuatorEnable_Pin, MOTOR_GO);
}

void brake_actuator_disable() {
    brake_actuator_enabled = false;
    HAL_GPIO_WritePin(ActuatorEnable_GPIO_Port, ActuatorEnable_Pin, MOTOR_STOP);
    brake_actuator_set_speed(0.0);
}

void brake_actuator_set_speed(float speed)
{   
    // safety stop if system is outside operating range
    if (fabs(actual_pressure) > MAX_PRESSURE) {
        speed = 0.0f;
    }

    if (fabs(speed) > BRAKE_ACTUATOR_SPEED_LIMIT){
        if (speed > 0.0) speed = BRAKE_ACTUATOR_SPEED_LIMIT;
        else speed = -BRAKE_ACTUATOR_SPEED_LIMIT;
    }
    
    HAL_GPIO_WritePin(ActuatorDir_GPIO_Port, ActuatorDir_Pin, speed < 0.0 ? RETRACT : EXTEND);

    // set new pwm frequency leaving constant the duty cycle ~ 50% of the new arr
    float steps_per_sec = fabs(speed) / MM_STEP;

    if (steps_per_sec > 0.0f) {
        //84e6 is the timer clock, need to be changed based on the ioc file
        uint32_t timer_clk = 84e6 / (TIM3->PSC + 1);
        uint16_t arr = (uint16_t)((timer_clk / steps_per_sec) - 1);
        // limit ARR to working range found by testing
        if (arr > 4999) arr = 4999; // ~ 200 Hz
        if (arr < 1249) arr = 1249; // ~ 600 Hz

        TIM3->ARR = arr;
        TIM3->CCR1 = arr / 2; // 50% duty cycle
    } else {
        TIM3->CCR1 = 0; // stop pulses
    }
}

void brake_actuator_pid_init(float kp, float ki, float kd, float sample_time, float anti_windup) {
    pid_init(&pid_controller, kp, kd, ki, sample_time, anti_windup, pid_prev_errors, N_PID_PREV_ERRORS);
    // wait for pressure value
    HAL_UART_Receive_IT(&huart2, uart_line, 1);
}

void brake_actuator_update_pid() {
    if (brake_actuator_enabled) {
        pid_update(&pid_controller, actual_pressure);
    }
}

void brake_actuator_update_speed() {
    if (brake_actuator_enabled) {
        float speed = pid_compute(&pid_controller);
        brake_actuator_set_speed(speed);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) return;

    uint8_t c = uart_line[uart_index];

    //terminated line, convertion
    if (c == '\n') {
        uart_line[uart_index] = '\0';
        uart_index = 0;

        //NEW PRESSURE
        if (uart_line[0] == 'p') {
            char *endptr;
            float val = strtof((char *)&uart_line[1], &endptr);

            //check if conversion happened
            if (endptr == (char *)&uart_line[1]) {
                HAL_UART_Transmit(&huart2, (uint8_t*)"[UART] ERROR: Invalid new pressure value\r\n", 39, HAL_MAX_DELAY);
            } else {
                actual_pressure = val;
                brake_actuator_update_set_point(actual_pressure);
                sprintf((char *)msg, "[UART] New PRESSURE: %.2f bar\r\n", actual_pressure);
                //HAL_UART_Transmit(&huart2, msg, strlen((char *)msg), HAL_MAX_DELAY);
            }
        }
        //SET PRESSURE
        else if (uart_line[0] == 's'){
            char *endptr;
            float val = strtof((char *)&uart_line[1], &endptr);

            //check if conversion happened
            if (endptr == (char *)&uart_line[1]) {
                HAL_UART_Transmit(&huart2, (uint8_t*)"[UART] ERROR: Invalid set pressure value\r\n", 39, HAL_MAX_DELAY);
            } else {
                actual_pressure = val;
                brake_actuator_update_set_point(actual_pressure);
                sprintf((char *)msg, "[UART] Set PRESSURE: %.2f bar\r\n", actual_pressure);
                //HAL_UART_Transmit(&huart2, msg, strlen((char *)msg), HAL_MAX_DELAY);
            }
        }
        //ENABLE/DISABLE ACTUATOR
        else if (uart_line[0] == 'c') {
            if (uart_line[1] == '0') {
                HAL_GPIO_WritePin(ActuatorEnable_GPIO_Port, ActuatorEnable_Pin, MOTOR_STOP);
                brake_actuator_disable();
                //HAL_UART_Transmit(&huart2, (uint8_t*)"[UART] DISABLED\r\n", 18, HAL_MAX_DELAY);
            } else if (uart_line[1] == '1'){
                HAL_GPIO_WritePin(ActuatorEnable_GPIO_Port, ActuatorEnable_Pin, MOTOR_GO);
                brake_actuator_enable();
                //HAL_UART_Transmit(&huart2, (uint8_t*)"[UART] ENABLED\r\n", 17, HAL_MAX_DELAY);
            } else {
                HAL_UART_Transmit(&huart2, (uint8_t*)"[UART] UNKNOWN COMMAND\r\n", 25, HAL_MAX_DELAY);    
            }
        }
        else {
            HAL_UART_Transmit(&huart2, (uint8_t*)"[UART] UNKNOWN COMMAND\r\n", 25, HAL_MAX_DELAY);
        }
    }
    else {
        if (uart_index < UART_LINE_MAX - 1) uart_index++;
    }

    HAL_UART_Receive_IT(&huart2, &uart_line[uart_index], 1);
}