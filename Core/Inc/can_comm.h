#ifndef CAN_COMM_H
#define CAN_COMM_H

#include "imu.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_can.h"
#include <stdio.h>

void initCAN(CAN_HandleTypeDef *hcan1);
static int16_t to_i16(float v);
void SendCANData(CAN_HandleTypeDef *hcan1, const float *corrected_distance, const FusedOrientation *fusedOrientation, const float *acc);
void GPS_ProcessCAN(CAN_HandleTypeDef *hcan);

#endif