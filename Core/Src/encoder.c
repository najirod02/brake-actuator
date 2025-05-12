#include "encoder.h"

uint32_t last_counter = 0;

uint32_t get_relative_counter(TIM_HandleTypeDef *htim){
    return __HAL_TIM_GET_COUNTER(htim) % CPR;
}

uint32_t get_absolute_counter(TIM_HandleTypeDef *htim){
    return __HAL_TIM_GET_COUNTER(htim);
}

int32_t get_signed_counter(TIM_HandleTypeDef *htim){
    int32_t current = __HAL_TIM_GET_COUNTER(htim);
    int32_t delta = current - last_counter;
    last_counter = current;
    return delta;
}

float get_distance(int32_t counter){
    return counter * ENC_MM_TICK;
}

float get_distance_from_counter(TIM_HandleTypeDef *htim){
    return __HAL_TIM_GET_COUNTER(htim) * ENC_MM_TICK;
}