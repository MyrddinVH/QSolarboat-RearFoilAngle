#include "can_comm.h"
#include "stm32l4xx_hal_can.h"

CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;
    
#ifdef DEBUG_PRINTS
    extern UART_HandleTypeDef huart2;
#endif

void initCAN(CAN_HandleTypeDef *hcan1)
{
    HAL_CAN_Start(hcan1);

    TxHeader.StdId = 0x123;
    TxHeader.ExtId = 0;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 2;
    TxHeader.TransmitGlobalTime = DISABLE;
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
    HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, len, 100);
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


    //accelerometer frame (0x7E4)
    int16_t ax = to_i16(acc[0] * 1000.0f);
    int16_t ay = to_i16(acc[1] * 1000.0f);
    int16_t az = to_i16(acc[2] * 1000.0f);


    uint8_t can_data3[6];
    can_data3[0] = ax >> 8;
    can_data3[1] = ax & 0xFF;
    can_data3[2] = ay >> 8;
    can_data3[3] = ay & 0xFF;
    can_data3[4] = az >> 8;
    can_data3[5] = az & 0xFF;

    TxHeader.StdId = 0x7E4;
    TxHeader.DLC = 6;

    while(HAL_CAN_GetTxMailboxesFreeLevel(hcan1) == 0);
    HAL_CAN_AddTxMessage(hcan1, &TxHeader, can_data3, &TxMailbox);
}