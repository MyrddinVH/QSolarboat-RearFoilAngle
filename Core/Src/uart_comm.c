#include "uart_comm.h"

#ifdef DEBUG_PRINTS
    extern UART_HandleTypeDef huart2;
#endif

FusedOrientation fusedOrientation;
IMU_Data imu_data[3];   // 0x50, 0x51, 0x52
float distance[2];      // 0x01, 0x02
float filtered_distance[2]; // after applying filters

static uint8_t tx_data[8];
uint8_t uart1_rx_buffer[UART_RX_BUFFER_SIZE];
uint8_t rx_accum[ACCUM_BUFFER_SIZE];
uint16_t rx_len = 0;
extern uint8_t imu_data_fresh[3];  // index 0=0x50, 1=0x51, 2=0x52
extern uint8_t filtered_distance_fresh[2];  // index 0=0x01, 1=0x02

/**
* \brief Maps an IMU address to its corresponding index in the imu_data array.
 * \param addr The address of the IMU (0x50, 0x51, or 0x52).
 * \return The index of the IMU in the imu_data array, or -1 if the address is invalid.
*/
static int imu_index(uint8_t addr)
{
    if (addr >= 0x50 && addr <= 0x52)
        return addr - 0x50;
    return -1;
}

/**
* \brief Maps a distance sensor address to its corresponding index in the distance array.
 * \param addr The address of the distance sensor (0x01 or 0x02).
 * \return The index of the distance sensor in the distance array, or -1 if the address is invalid.
 */
 
static int dist_index(uint8_t addr)
{
    if (addr == 0x01) return 0;
    if (addr == 0x02) return 1;
    return -1;
}

#ifdef DEBUG_PRINTS
    // Debug function to print byte arrays in hex format.
    static void debug_print_bytes(const char *label, uint8_t *buf, uint16_t len)
    {
        char msg[256];
        int pos = snprintf(msg, sizeof(msg), "%s [%d]: ", label, len);

        for (uint16_t i = 0; i < len && pos < sizeof(msg) - 4; i++)
            pos += snprintf(msg + pos, sizeof(msg) - pos, "%02X ", buf[i]);

        pos += snprintf(msg + pos, sizeof(msg) - pos, "\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, pos, 100);
    }
#endif

/**
* \brief Restarts the UART DMA reception by re-enabling the receive-to-idle mode.
 * \param huart1 Pointer to the UART handle for USART1.
*/
void UART_Restart_DMA(UART_HandleTypeDef *huart1)
{
    HAL_UARTEx_ReceiveToIdle_DMA(huart1, uart1_rx_buffer, UART_RX_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart1->hdmarx, DMA_IT_HT);
}

/**
* \brief Sends a Modbus request frame to the specified sensor address over UART.
 * \param huart1 Pointer to the UART handle for USART1.
 * \param addr The address of the sensor to request data from (0x01, 0x02 for ultrasonic sensors, 0x50-0x52 for IMUs).
*/
void Modbus_Send_Request(UART_HandleTypeDef *huart1, uint8_t addr)
{
    tx_data[0] = addr;
    tx_data[1] = 0x03;

    uint16_t reg = 0, count = 0;

    if (addr == 0x01 || addr == 0x02)
    {
        reg = 0x0222;
        count = 0x0001;
    }
    else if (addr >= 0x50 && addr <= 0x52)
    {
        reg = 0x0034;
        count = 0x000C;
    }

    tx_data[2] = reg >> 8;
    tx_data[3] = reg & 0xFF;
    tx_data[4] = count >> 8;
    tx_data[5] = count & 0xFF;

    uint16_t crc = crc16(tx_data, 6);
    tx_data[6] = crc & 0xFF;
    tx_data[7] = crc >> 8;

    HAL_UART_Transmit(huart1, tx_data, 8, 1000);
}

#ifdef DEBUG_PRINTS
    // Print the parsed data for a given address.
    void print_data(uint8_t addr)
    {
        char msg[256];
        int len = 0;

        if (addr == 0x01 || addr == 0x02)
        {
            int idx = dist_index(addr);
            len = snprintf(msg, sizeof(msg),
                "[0x%02X] DIST: %.2f\r\n",
                addr, filtered_distance[idx]);
        }
        else if (addr >= 0x50 && addr <= 0x52)
        {
            int idx = imu_index(addr);
            IMU_Data *d = &imu_data[idx];

            len = snprintf(msg, sizeof(msg),
                "[0x%02X] ACC: %.2f %.2f %.2f | GYRO: %.2f %.2f %.2f | ANG: %.2f %.2f %.2f\r\n",
                addr,
                d->acc[0], d->acc[1], d->acc[2],
                d->gyro[0], d->gyro[1], d->gyro[2],
                d->angle[0], d->angle[1], d->angle[2]);
        }
        //!uncomment for accelerometer data
        // HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
    }
#endif

/**
* \brief Consumes a specified number of bytes from the accumulated UART buffer, shifting the remaining data to the front.
 * \param len The number of bytes to consume from the buffer.
*/
void buffer_consume(uint16_t len)
{
    if (len >= rx_len) { rx_len = 0; return; }

    memmove(rx_accum, rx_accum + len, rx_len - len);
    rx_len -= len;
}

/**
* \brief Processes the accumulated UART data to extract valid frames and update the global data structures.
 * \param huart1 Pointer to the UART handle for USART1, used for potential debug transmissions.
*/
void process_uart_stream(UART_HandleTypeDef *huart1)
{
    while (rx_len >= 5)
    {
        // Check for valid address and function code (0x03)
        if (!(
            (rx_accum[0] == 0x01 || rx_accum[0] == 0x02 ||
             rx_accum[0] == 0x50 || rx_accum[0] == 0x51 || rx_accum[0] == 0x52)
            && rx_accum[1] == 0x03))
        {
            buffer_consume(1);
            continue;
        }

        // Check if we have received the full frame based on the byte count.
        uint8_t byte_count = rx_accum[2];
        uint16_t frame_size = 3 + byte_count + 2;

        if (rx_len < frame_size)
            return;

        // Validate CRC
        uint16_t crc_rx   = rx_accum[frame_size - 2] | (rx_accum[frame_size - 1] << 8);
        uint16_t crc_calc = crc16(rx_accum, frame_size - 2);

        if (crc_rx != crc_calc)
        {   
            #ifdef DEBUG_PRINTS
                debug_print_bytes("CRC FAIL", rx_accum, frame_size);
            #endif
            buffer_consume(1);
            continue;
        }

        // Valid frame received, toggle LED
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

        uint8_t addr = rx_accum[0];

        // Update distance or IMU data based on the address.
        int d_idx = dist_index(addr);
        if (d_idx >= 0)
        {
            uint16_t value = (rx_accum[3] << 8) | rx_accum[4];
            distance[d_idx] = (float)value;
            filtered_distance[d_idx] = apply_filters(d_idx, distance[d_idx]);
            filtered_distance_fresh[d_idx] = 1;
            #ifdef DEBUG_PRINTS
                print_data(addr);
            #endif
        }

        int i_idx = imu_index(addr);
        if (i_idx >= 0)
        {
            IMU_Parse(rx_accum, frame_size, &imu_data[i_idx]);
            imu_data_fresh[i_idx] = 1;
            #ifdef DEBUG_PRINTS
                print_data(addr);
            #endif
        }

        buffer_consume(frame_size);
    }
}

/**
* \brief UART receive complete callback function, called by the HAL when a UART reception is completed.
 * \param huart1 Pointer to the UART handle for USART1, used to identify which UART triggered the callback.
 * \param Size The number of bytes received in the current DMA transfer.
*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart1, uint16_t Size)
{
    if (huart1->Instance != USART1) return;

    if ((rx_len + Size) > ACCUM_BUFFER_SIZE)
        rx_len = 0;

    memcpy(&rx_accum[rx_len], uart1_rx_buffer, Size);
    rx_len += Size;
    
    process_uart_stream(huart1);
    UART_Restart_DMA(huart1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart1)
{
    if (huart1->Instance != USART1) return;

    __HAL_UART_CLEAR_OREFLAG(huart1);
    __HAL_UART_CLEAR_FEFLAG(huart1);
    __HAL_UART_CLEAR_NEFLAG(huart1);

    UART_Restart_DMA(huart1);
}

/**
* \brief Initializes the UART communication by starting the DMA reception for USART1.
 * \param huart1 Pointer to the UART handle for USART1, used to set up the DMA reception.
*/
void UART_Comm_Init(UART_HandleTypeDef *huart1)
{
    UART_Restart_DMA(huart1);
}