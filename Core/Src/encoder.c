#include "encoder.h"

uint32_t get_relative_counter(TIM_HandleTypeDef *htim){
    return __HAL_TIM_GET_COUNTER(htim) % CPR;
}

uint32_t get_absolute_counter(TIM_HandleTypeDef *htim){
    return __HAL_TIM_GET_COUNTER(htim);
}

int32_t get_signed_counter(TIM_HandleTypeDef *htim){
    return (int32_t)__HAL_TIM_GET_COUNTER(htim);
}

float get_distance(int32_t counter){
    return counter * ENC_MM_TICK;
}

float get_distance_from_counter(TIM_HandleTypeDef *htim){
    return __HAL_TIM_GET_COUNTER(htim) * ENC_MM_TICK;
}