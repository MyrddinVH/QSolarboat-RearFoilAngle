#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

#define DEBUG_PRINTS // Comment out to disable debug prints over UART2
#define SINGLE_IMU // Comment out to use all 3 IMUs for orientation fusion, uncomment to use only middle IMU and ignore left and right IMU

// Sensor physical setup (use 0x00 for unused sensors)
static const uint8_t imu_addresses[] = { 0x50, 0x51, 0x52 };
static const uint8_t us_addresses[]  = { 0x00, 0x00, 0x00 };
// static const uint8_t us_addresses[]  = { 0x01, 0x02, 0x00 };

#define SENSOR_PAIR_COUNT       1
#define SENSOR_DISTANCE_MM      150       // Distance between left and right sensors in mm

// Timeouts
#define IMU_TIMEOUT_MS          20
#define ULTRASONIC_TIMEOUT_MS   50

// Filter tuning
#define MEDIAN_SIZE             5         // Must stay odd
#define OUTLIERTHRESHOLD        200.0f    // Maximum allowed change in distance (mm) between consecutive measurements before considering it an outlier
#define LOWPASSALPHA            0.4f      // Higher alpha means more trust in current measurement, lower alpha means more trust in previous filtered value
#define FUSE_ALPHA              0.85f     // Higher alpha means more trust in IMU roll, lower alpha means more trust in ultrasonic roll

#endif