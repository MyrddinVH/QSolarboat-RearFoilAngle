#include "can_comm.h"
#include "stm32l4xx_hal_can.h"
#include <stdio.h>
#include <string.h>
#include "imu.h"

CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;
    
#ifdef DEBUG_PRINTS
    extern UART_HandleTypeDef huart2;
#endif

void initCAN(CAN_HandleTypeDef *hcan1)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(hcan1, &filter) != HAL_OK)
    {
        // Error_Handler();
    }

    if (HAL_CAN_Start(hcan1) != HAL_OK)
    {
        // Error_Handler();
    }

    TxHeader.StdId = 0x123;
    TxHeader.ExtId = 0;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
}

void GPS_ProcessCAN(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
        {
            continue;
        }

        // Alleen standaard dataframes met GPS-ID 0x783
        if (rxHeader.IDE != CAN_ID_STD ||
            rxHeader.RTR != CAN_RTR_DATA ||
            rxHeader.StdId != 0x783)
        {
            continue;
        }

        if (rxHeader.DLC != 8)
        {
            continue;
        }

        uint8_t gps_fix = rxData[0];

        // Data[1..4] bevat de float, LSB eerst
        float gps_speed_kmh;
        memcpy(&gps_speed_kmh, &rxData[1], sizeof(gps_speed_kmh));

        char msg[100];

        if (gps_fix == 1)
        {
            float gps_speed_ms = gps_speed_kmh / 3.6f;
        
            // GPS overschrijft direct de IMU-snelheid
            IMU_SetSpeed(gps_speed_ms);
        
        #ifdef DEBUG_PRINTS
            int len = snprintf(msg, sizeof(msg),"GPS fix: YES | GPS: %.2f km/h | IMU reset: %.2f km/h\r\n",(double)gps_speed_kmh,(double)Speed_GetKmh());
        
            if (len > 0 && len < (int)sizeof(msg))
            {
                HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)len, 100);
            }
        #endif
        }
        else
        {
            #ifdef DEBUG_PRINTS
            int len = snprintf(msg, sizeof(msg),"GPS fix: NO | Raw speed: %.2f km/h\r\n",(double)gps_speed_kmh);

            if (len > 0 && len < (int)sizeof(msg))
            {
                HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)len, 100);
            }
            #endif
        }
    }
}

static int16_t to_i16(float v){
    if(v > 32767.0f) return 32767;
    if(v< -32768.0f) return 32767;
    return (int16_t) v;
}

void SendCANData(CAN_HandleTypeDef *hcan1, const float *corrected_distance, const FusedOrientation *fusedOrientation, const float *acc)
{
    float left_height   = corrected_distance[0];
    float right_height  = corrected_distance[1];
    float global_height = (left_height + right_height) / 2.0f;

    int16_t h_left   = (int16_t)(left_height   * 100.0f);
    int16_t h_right  = (int16_t)(right_height  * 100.0f);
    int16_t h_global = (int16_t)(global_height * 100.0f);
    
#ifdef DEBUG_PRINTS
    char debug_msg[256];
    int len = snprintf(debug_msg, sizeof(debug_msg),
                       "HEIGHTS -> L: %.2f mm (%d) | R: %.2f mm (%d) | G: %.2f mm (%d)\r\n",
                       left_height, h_left, right_height, h_right, global_height, h_global);
    //! uncomment for height debug data
    // HAL_UART_Transmit(&huart2,(uint8_t*)debug_msg,len,100);
#endif
    uint8_t can_data1[6];

    can_data1[0] = h_left >> 8;
    can_data1[1] = h_left & 0xFF;
    can_data1[2] = h_right >> 8;
    can_data1[3] = h_right & 0xFF;
    can_data1[4] = h_global >> 8;
    can_data1[5] = h_global & 0xFF;

    TxHeader.StdId = 0x100;
    TxHeader.DLC   = 6;

    // while (HAL_CAN_GetTxMailboxesFreeLevel(hcan1) == 0);
    // HAL_CAN_AddTxMessage(hcan1, &TxHeader, can_data1, &TxMailbox);

    float roll_f  = fusedOrientation->global_roll;
    float pitch_f = fusedOrientation->global_pitch;
    float yaw_f   = fusedOrientation->global_yaw;

    int16_t roll  = (int16_t)(roll_f  * 100.0f);
    int16_t pitch = (int16_t)(pitch_f * 100.0f);
    int16_t yaw   = (int16_t)(yaw_f   * 100.0f);

#ifdef DEBUG_PRINTS
    len = snprintf(debug_msg, sizeof(debug_msg),
                   "ANGLES -> Roll: %.2f (%d) | Pitch: %.2f (%d) | Yaw: %.2f (%d)\r\n",
                   roll_f, roll, pitch_f, pitch, yaw_f, yaw);

    //!angle debug msg
    // HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, len, 100);
#endif

    uint8_t can_data2[6];

    can_data2[0] = roll >> 8;
    can_data2[1] = roll & 0xFF;
    can_data2[2] = pitch >> 8;
    can_data2[3] = pitch & 0xFF;
    can_data2[4] = yaw >> 8;
    can_data2[5] = yaw & 0xFF;

    TxHeader.StdId = 0x7E3;
    TxHeader.DLC   = 6;

    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan1) == 0);
    HAL_CAN_AddTxMessage(hcan1, &TxHeader, can_data2, &TxMailbox);


    // //accelerometer frame (0x7E4)
    // int16_t ax = to_i16(acc[0] * 1000.0f);
    // int16_t ay = to_i16(acc[1] * 1000.0f);
    // int16_t az = to_i16(acc[2] * 1000.0f);


    // uint8_t can_data3[6];
    // can_data3[0] = ax >> 8;
    // can_data3[1] = ax & 0xFF;
    // can_data3[2] = ay >> 8;
    // can_data3[3] = ay & 0xFF;
    // can_data3[4] = az >> 8;
    // can_data3[5] = az & 0xFF;

    // TxHeader.StdId = 0x7E4;
    // TxHeader.DLC = 6;

    // while(HAL_CAN_GetTxMailboxesFreeLevel(hcan1) == 0);
    // HAL_CAN_AddTxMessage(hcan1, &TxHeader, can_data3, &TxMailbox);
}