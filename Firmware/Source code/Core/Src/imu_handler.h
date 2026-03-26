/**
 * @file imu_handler.h
 * @brief IIM-46234 stream packet unpacker + Madgwick orientation pipeline
 *
 * Packet layout (actual_stream = raw_spi_buf + 1, after stripping echo byte):
 *
 *  Byte  0-1  : Header  0x23 0x23
 *  Byte  2    : Length (low byte)
 *  Byte  3    : Command type  0xAB (stream data)
 *  Byte  4    : Sensor status
 *  Byte  5    : Sample counter
 *  Byte  6-13 : Timestamp (2 x 32-bit big-endian, microseconds)
 *  Byte 14-17 : Accel X  (float32 big-endian, m/s2)
 *  Byte 18-21 : Accel Y  (float32 big-endian, m/s2)
 *  Byte 22-25 : Accel Z  (float32 big-endian, m/s2)
 *  Byte 26-29 : Gyro  X  (float32 big-endian, rad/s)
 *  Byte 30-33 : Gyro  Y  (float32 big-endian, rad/s)
 *  Byte 34-37 : Gyro  Z  (float32 big-endian, rad/s)
 *  Byte 38-41 : Temperature (float32 big-endian, deg C)
 *  Byte 42-43 : Checksum (uint16 big-endian, sum of bytes 3..41)
 *  Byte 44    : Footer 0x0D
 *  Byte 45    : Footer 0x0A
 */

#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "imu_madgwick.h"

/* -------------------------------------------------------------------------- */
/* Constants                                                                   */
/* -------------------------------------------------------------------------- */

/** Expected packet size (bytes 0-45 in actual_stream)                        */
#define IMU_STREAM_PACKET_SIZE   46u

/** Byte offset of first data field inside actual_stream                      */
#define IMU_ACCEL_X_OFFSET       14u
#define IMU_ACCEL_Y_OFFSET       18u
#define IMU_ACCEL_Z_OFFSET       22u
#define IMU_GYRO_X_OFFSET        26u
#define IMU_GYRO_Y_OFFSET        30u
#define IMU_GYRO_Z_OFFSET        34u
#define IMU_TEMP_OFFSET          38u

/* -------------------------------------------------------------------------- */
/* Data structures                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Decoded IMU sample with computed orientation
 */
typedef struct {
    /* Raw sensor data */
    float accel_x;      /**< m/s2  */
    float accel_y;
    float accel_z;
    float gyro_x;       /**< rad/s */
    float gyro_y;
    float gyro_z;
    float temperature;  /**< deg C */
    uint8_t  sample_counter;
    uint32_t timestamp_high; /**< Upper 32 bits of 64-bit us timestamp */
    uint32_t timestamp_low;  /**< Lower 32 bits of 64-bit us timestamp */

    /* Computed orientation */
    EulerAngles_t euler;
    Quaternion_t  quaternion;

    /* Status */
    uint8_t  checksum_ok;
} IMU_Sample_t;

/* -------------------------------------------------------------------------- */
/* Parse error codes                                                           */
/* -------------------------------------------------------------------------- */

#define IMU_ERR_OK            ( 1)   /**< Packet parsed and Madgwick updated  */
#define IMU_ERR_NULL          (-1)   /**< NULL pointer argument               */
#define IMU_ERR_TOO_SHORT     (-2)   /**< Buffer too short for a stream pkt   */
#define IMU_ERR_BAD_HEADER    (-3)   /**< Header bytes != 0x23 0x23           */
#define IMU_ERR_BAD_CMD       (-4)   /**< Command type byte != 0xAB           */
#define IMU_ERR_NO_FOOTER     (-5)   /**< 0x0D 0x0A footer not found          */
#define IMU_ERR_BAD_CHECKSUM  (-6)   /**< Checksum mismatch                   */

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialise the IMU handler (also resets the Madgwick filter).
 *
 * @param sample_freq  IMU output data rate in Hz (e.g. 100.0f).
 * @param beta         Madgwick filter gain (e.g. 0.1f).
 */
void IMU_Handler_Init(float sample_freq, float beta);

/**
 * @brief Process one stream packet from a buffer that has NO echo byte.
 *
 * Use this in mcu_on_imu_drdy where HAL_SPI_Receive() is called directly
 * and the buffer contains the raw payload starting at byte 0:
 *   buf[0] = 0x24, buf[1] = 0x24, buf[2] = length, buf[3] = 0xAB, ...
 *
 * Validates: header (0x24 0x24), command type (0xAB), footer (0x0D 0x0A),
 * and checksum.  On success runs the Madgwick update and fills *out.
 *
 * @param stream_buf  Pointer to the SPI RX buffer (payload, no echo byte).
 * @param buf_len     Number of bytes in the buffer (>= IMU_STREAM_PACKET_SIZE).
 * @param out         Pointer to output sample struct.
 * @return            IMU_ERR_OK (1) on success, negative IMU_ERR_* on failure.
 */
int IMU_Handler_ProcessStreamPacket(const uint8_t *stream_buf,
                                    uint32_t       buf_len,
                                    IMU_Sample_t  *out);

/**
 * @brief Process one raw SPI buffer that INCLUDES the leading echo byte.
 *
 * Legacy function kept for completeness.  The echo byte at index 0 is
 * stripped internally before parsing.
 *
 * @param raw_spi_buf  Pointer to the full SPI receive buffer (echo + payload).
 * @param buf_len      Length of the buffer in bytes.
 * @param out          Pointer to output sample struct.
 * @return             IMU_ERR_OK (1) on success, negative IMU_ERR_* on failure.
 */
int IMU_Handler_ProcessPacket(const uint8_t *raw_spi_buf,
                               uint32_t       buf_len,
                               IMU_Sample_t  *out);

/**
 * @brief Update Madgwick sample frequency (Hz) without resetting the filter state.
 */
void IMU_Handler_SetSampleFreq(float sample_freq);

/**
 * @brief Override the gyro bias values used in orientation computation.
 *
 * Biases are measured in deg/s with the board stationary.
 * They are subtracted from the raw gyro reading before the Madgwick update.
 * Defaults are the values measured 2026-02-26 at ±250 dps FSR:
 *   X=+0.421587  Y=-0.133538  Z=+0.064359 (deg/s)
 *
 * @param bias_x_dps  Gyro X bias in deg/s
 * @param bias_y_dps  Gyro Y bias in deg/s
 * @param bias_z_dps  Gyro Z bias in deg/s
 */
void IMU_Handler_SetGyroBias(float bias_x_dps, float bias_y_dps, float bias_z_dps);

/**
 * @brief Get the current Madgwick filter state (quaternion).
 */
const MadgwickFilter_t *IMU_Handler_GetFilter(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_HANDLER_H */