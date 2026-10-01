#include "imu.h"

//TODO ADD IMU CALIBRATION

/**
* \brief Parses the raw data received from an IMU sensor over UART and fills the IMU_Data structure with the parsed values.
 * \param rx Pointer to the received byte array containing the IMU data frame.
 * \param len The length of the received byte array.
 * \param out Pointer to the IMU_Data structure where the parsed data will be stored.
 * \return 0 on success, negative value on error (e.g., invalid frame format).
*/
int IMU_Parse(uint8_t *rx, uint16_t len, IMU_Data *out)
{
    if(len < 29) return -1;
    if(rx[1] != 0x03) return -2;
    if(rx[2] != 24) return -3;

    int16_t raw[12];
    uint8_t *data = &rx[3];

    for(int i = 0; i < 12; i++)
    {
        raw[i] = (int16_t)((data[2*i] << 8) | data[2*i + 1]);
    }

    for(int i = 0; i < 3; i++)
    {
        out->acc[i]   = raw[i]     / 32768.0f * 16.0f;
        out->gyro[i]  = raw[i+3]   / 32768.0f * 2000.0f;
        out->mag[i]   = raw[i+6];
        out->angle[i] = raw[i+9]   / 32768.0f * 180.0f;
    }

    return 0;
}

/**
* \brief Fuses the orientation data from multiple IMUs by taking the angles from the middle IMU as the global orientation.
 * \param imu_fl Pointer to the IMU data structure for the front-left IMU.
 * \param imu_mid Pointer to the IMU data structure for the middle IMU.
 * \param imu_fr Pointer to the IMU data structure for the front-right IMU.
 * \param out Pointer to the FusedOrientation structure where the fused orientation will be stored.
*/
void fuse_orientation (IMU_Data *imu_fl, IMU_Data *imu_mid, IMU_Data *imu_fr, FusedOrientation *out)
{
    #ifdef SINGLE_IMU
        out->global_yaw   = imu_fl->angle[2];
        out->global_roll  = imu_fl->angle[1];
        out->global_pitch = imu_fl->angle[0];
        return;
    #else
        float x_yaw = cosf(imu_fl->angle[2]) + cosf(imu_mid->angle[2]) + cosf(imu_fr->angle[2]);
        float y_yaw = sinf(imu_fl->angle[2]) + sinf(imu_mid->angle[2]) + sinf(imu_fr->angle[2]);
        float yaw = atan2(y_yaw, x_yaw);
        if (yaw < 0.0f)
        {
            yaw += 2.0f * M_PI;
        }
        out->global_yaw = yaw;


        out->global_roll = (imu_fl->angle[1] + imu_mid->angle[1] + imu_fr->angle[1]) / 3.0f;
        out->global_pitch  = (imu_fl->angle[0] + imu_mid->angle[0] + imu_fr->angle[0]) / 3.0f;
    #endif
}

/**
* \brief Fuses the roll angle from the IMU and the roll angle calculated from the ultrasonic sensors using a complementary filter.
 Alpha 0.7 means 70% trust in the IMU and 30% trust in the ultrasonic sensors.
 \brief Not used in final code, but could be useful for testing or future improvements.
 * \param roll_imu The roll angle measured by the IMU in degrees.
 * \param roll_ultra The roll angle calculated from the ultrasonic sensors in degrees.
 * \return The fused roll angle in degrees.Not used in final code, but could be useful for testing or future improvements
 */
float fuse_roll(float roll_imu, float roll_ultra) {
    return FUSE_ALPHA * roll_imu + (1.0f - FUSE_ALPHA) * roll_ultra;
}
