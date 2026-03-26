/**
 * @file imu_handler.c
 * @brief IIM-46234 stream packet unpacker + Madgwick orientation pipeline
 */

#include "imu_handler.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Private constants                                                           */
/* -------------------------------------------------------------------------- */

#define BYTE_HEADER        0x23u   /* IIM-46234 stream packets use 0x23, not 0x24 */
#define BYTE_FOOTER_1      0x0Du
#define BYTE_FOOTER_2      0x0Au
#define CMD_STREAM_DATA    0xABu
/* Sustain 1 kHz packet ingest on STM32G0 by running the float-heavy fusion
 * path at a lower effective rate. Packet parsing and sample timestamps remain
 * at full rate; only the Madgwick update is decimated. */
#define IMU_FUSION_DECIM   8u

/* -------------------------------------------------------------------------- */
/* Private state                                                               */
/* -------------------------------------------------------------------------- */

static MadgwickFilter_t s_filter;
static float s_packet_sample_freq = 1000.0f;
static float s_accum_gx = 0.0f;
static float s_accum_gy = 0.0f;
static float s_accum_gz = 0.0f;
static float s_accum_ax = 0.0f;
static float s_accum_ay = 0.0f;
static float s_accum_az = 0.0f;
static uint8_t s_fusion_accum_count = 0u;

/* Gyro bias in deg/s ??subtracted from raw gyro before Madgwick update.
 * Measured 2026-02-26 at ±250 dps FSR, board stationary.
 * Call IMU_Handler_SetGyroBias() to override at runtime.             */
static float s_gyro_bias_x =  0.421587f;   /* deg/s */
static float s_gyro_bias_y = -0.133538f;   /* deg/s */
static float s_gyro_bias_z =  0.064359f;   /* deg/s */

/* -------------------------------------------------------------------------- */
/* Private helpers                                                             */
/* -------------------------------------------------------------------------- */

/** Decode a big-endian IEEE 754 float from 4 consecutive bytes. */
static float decode_float_be(const uint8_t *b)
{
    union { float f; uint32_t i; } u;
    u.i = ((uint32_t)b[0] << 24) |
          ((uint32_t)b[1] << 16) |
          ((uint32_t)b[2] <<  8) |
           (uint32_t)b[3];
    return u.f;
}

/** Simple byte-sum checksum over [buf, buf+len). */
static uint16_t calc_checksum(const uint8_t *buf, uint32_t len)
{
    uint16_t sum = 0;
    for (uint32_t i = 0; i < len; i++) sum += (uint16_t)buf[i];
    return sum;
}

static float imu_get_fusion_sample_freq(float packet_sample_freq)
{
    if (packet_sample_freq <= 0.0f) return 1000.0f;
    if (IMU_FUSION_DECIM <= 1u) return packet_sample_freq;

    float fusion_freq = packet_sample_freq / (float)IMU_FUSION_DECIM;
    if (fusion_freq < 1.0f) fusion_freq = packet_sample_freq;
    return fusion_freq;
}

/**
 * @brief Core parser ??operates on a buffer that starts at byte 0 of the
 *        IIM-46234 stream packet (i.e. 0x23 0x23 ...), NO echo byte.
 *
 * Validates header, cmd type, footer, and checksum, then decodes all
 * sensor fields and runs a Madgwick update.
 *
 * @return IMU_ERR_OK on success, negative IMU_ERR_* code on any failure.
 */
static int decode_stream_pkt(const uint8_t *pkt, uint32_t pkt_len,
                              IMU_Sample_t  *out)
{
    /* ---- Minimum length guard ---------------------------------------- */
    /* Need at least: header(2) + len(1) + cmd(1) + status(1) + counter(1)
       + timestamp(8) + accel(12) + gyro(12) + temp(4) + chk(2) + footer(2) = 46 */
    if (pkt_len < IMU_STREAM_PACKET_SIZE) {
        return IMU_ERR_TOO_SHORT;
    }

    /* ---- Header validation ------------------------------------------- */
    if (pkt[0] != BYTE_HEADER || pkt[1] != BYTE_HEADER) {
        return IMU_ERR_BAD_HEADER;
    }

    /* ---- Command type validation ------------------------------------- */
    if (pkt[3] != CMD_STREAM_DATA) {
        return IMU_ERR_BAD_CMD;
    }

    /* ---- Footer at fixed position ------------------------------------ */
    /* The IIM-46234 stream packet is always exactly IMU_STREAM_PACKET_SIZE
       (46) bytes.  The footer 0x0D 0x0A is always at the last two bytes.
       DO NOT search for it ??sensor float data can contain 0x0D or 0x0A,
       causing a false early match and an IMU_ERR_TOO_SHORT failure.      */
    const int packet_end = (int)IMU_STREAM_PACKET_SIZE - 2;  /* = 44 */

    if (pkt[packet_end] != BYTE_FOOTER_1 || pkt[packet_end + 1u] != BYTE_FOOTER_2) {
        return IMU_ERR_NO_FOOTER;
    }

    /* ---- Checksum verification --------------------------------------- */
    /* Verified against raw packets: checksum is uint16 BIG-ENDIAN.
       Occupies bytes [42][43] (packet_end-2, packet_end-1).
       Data range summed: bytes [3..41] inclusive = 39 bytes.             */
    uint16_t chk_recv =
        ((uint16_t)pkt[packet_end - 2] << 8) |
         (uint16_t)pkt[packet_end - 1];          /* BE */

    /* Sum bytes [3..41] = 39 bytes (packet_end=44, so [3..packet_end-3]) */
    uint16_t chk_calc = calc_checksum(&pkt[3], (uint32_t)(packet_end - 3 - 3 + 1));

    if (chk_calc != chk_recv) {
        out->checksum_ok = 0u;
        return IMU_ERR_BAD_CHECKSUM;
    }
    out->checksum_ok = 1u;

    /* ---- Decode fields ----------------------------------------------- */
    out->sample_counter = pkt[5];

    out->timestamp_high = ((uint32_t)pkt[ 6] << 24) |
                          ((uint32_t)pkt[ 7] << 16) |
                          ((uint32_t)pkt[ 8] <<  8) |
                           (uint32_t)pkt[ 9];

    out->timestamp_low  = ((uint32_t)pkt[10] << 24) |
                          ((uint32_t)pkt[11] << 16) |
                          ((uint32_t)pkt[12] <<  8) |
                           (uint32_t)pkt[13];

    out->accel_x     = decode_float_be(&pkt[IMU_ACCEL_X_OFFSET]);
    out->accel_y     = decode_float_be(&pkt[IMU_ACCEL_Y_OFFSET]);
    out->accel_z     = decode_float_be(&pkt[IMU_ACCEL_Z_OFFSET]);
    out->gyro_x      = decode_float_be(&pkt[IMU_GYRO_X_OFFSET]);
    out->gyro_y      = decode_float_be(&pkt[IMU_GYRO_Y_OFFSET]);
    out->gyro_z      = decode_float_be(&pkt[IMU_GYRO_Z_OFFSET]);
    out->temperature = decode_float_be(&pkt[IMU_TEMP_OFFSET]);

    /* ---- Madgwick update --------------------------------------------- */
    /* The STM32G0 cannot sustain full 1 kHz stream service if we run the
     * Madgwick step on every packet. Keep 1 kHz packet ingest, but fuse at a
     * lower effective rate using a short average window. */
    const float DEG2RAD = 3.14159265358979f / 180.0f;
    s_accum_gx += (out->gyro_x - s_gyro_bias_x) * DEG2RAD;
    s_accum_gy += (out->gyro_y - s_gyro_bias_y) * DEG2RAD;
    s_accum_gz += (out->gyro_z - s_gyro_bias_z) * DEG2RAD;
    s_accum_ax += out->accel_x;
    s_accum_ay += out->accel_y;
    s_accum_az += out->accel_z;
    s_fusion_accum_count++;

    if (s_fusion_accum_count >= IMU_FUSION_DECIM) {
        const float inv_count = 1.0f / (float)s_fusion_accum_count;
        Madgwick_Update(&s_filter,
                        s_accum_gx * inv_count,
                        s_accum_gy * inv_count,
                        s_accum_gz * inv_count,
                        s_accum_ax * inv_count,
                        s_accum_ay * inv_count,
                        s_accum_az * inv_count);
        s_accum_gx = 0.0f;
        s_accum_gy = 0.0f;
        s_accum_gz = 0.0f;
        s_accum_ax = 0.0f;
        s_accum_ay = 0.0f;
        s_accum_az = 0.0f;
        s_fusion_accum_count = 0u;
    }

    Madgwick_GetQuaternion(&s_filter, &out->quaternion);

    return IMU_ERR_OK;
}

/* -------------------------------------------------------------------------- */
/* Public implementation                                                       */
/* -------------------------------------------------------------------------- */

void IMU_Handler_Init(float sample_freq, float beta)
{
    s_packet_sample_freq = sample_freq;
    s_accum_gx = 0.0f;
    s_accum_gy = 0.0f;
    s_accum_gz = 0.0f;
    s_accum_ax = 0.0f;
    s_accum_ay = 0.0f;
    s_accum_az = 0.0f;
    s_fusion_accum_count = 0u;
    Madgwick_Init(&s_filter, imu_get_fusion_sample_freq(sample_freq), beta);
}

void IMU_Handler_SetSampleFreq(float sample_freq)
{
    s_packet_sample_freq = sample_freq;
    Madgwick_SetSampleFreq(&s_filter, imu_get_fusion_sample_freq(sample_freq));
}

void IMU_Handler_SetGyroBias(float bias_x_dps, float bias_y_dps, float bias_z_dps)
{
    s_gyro_bias_x = bias_x_dps;
    s_gyro_bias_y = bias_y_dps;
    s_gyro_bias_z = bias_z_dps;
}

/*
 * Use this in mcu_on_imu_drdy.
 * The buffer must start at byte 0 of the stream packet (0x23 0x23 ...),
 * with NO leading echo byte ??exactly what HAL_SPI_Receive gives you when
 * you call it directly to read the stream data.
 */
int IMU_Handler_ProcessStreamPacket(const uint8_t *stream_buf,
                                    uint32_t       buf_len,
                                    IMU_Sample_t  *out)
{
    if (stream_buf == NULL || out == NULL) return IMU_ERR_NULL;
    return decode_stream_pkt(stream_buf, buf_len, out);
}

/*
 * Legacy function: buffer includes the echo byte at index 0.
 * Strips it then delegates to the same parser.
 */
int IMU_Handler_ProcessPacket(const uint8_t *raw_spi_buf,
                               uint32_t       buf_len,
                               IMU_Sample_t  *out)
{
    if (raw_spi_buf == NULL || out == NULL || buf_len < 2u) return IMU_ERR_NULL;
    /* Skip the echo byte */
    return decode_stream_pkt(raw_spi_buf + 1u, buf_len - 1u, out);
}

const MadgwickFilter_t *IMU_Handler_GetFilter(void)
{
    return &s_filter;
}




