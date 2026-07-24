/**
 * @file imu_config.h
 * @brief IIM-46234 ODR and LPF configuration via WRITE_REGISTER commands.
 *
 * All values taken directly from the official InvenSense Atmel reference
 * implementation in IIM4623x_ATMEL/iim4623x/iim4623x.h and iim4623x.c.
 *
 * ----- WRITE_REGISTER command format (from IIM4623x_SetCMD_WriteRegister) -----
 *  [0]     0x24           header
 *  [1]     0x24           header
 *  [2]     total_size     SIZE_CMD_WRITE_REGS_BASE(12) + data_length
 *  [3]     0x12           CMD_TYPE_WRITE_REG
 *  [4]     0x00           reserved
 *  [5]     data_length    number of bytes to write (1 or 2 or 4)
 *  [6]     reg_addr       register first address
 *  [7]     page_id        register page (0 = config, 1 = output)
 *  [8...]  value bytes    big-endian
 *  [8+len] chk_hi         checksum high byte (sum of bytes [3..7+len])
 *  [9+len] chk_lo         checksum low byte
 *  [10+len] 0x0D          footer
 *  [11+len] 0x0A          footer
 *  Total  = 12 + data_length bytes
 *
 * ----- ACK response (SIZE_RESP_ACK = 10 bytes) -----
 *  The sensor responds to every WRITE_REG with a 10-byte ACK.
 *  A DRDY pulse IS generated (same as all other commands).
 *  send_imu_command_ex waits for DRDY then reads exactly SIZE_RESP_ACK bytes.
 *
 * ----- ODR configuration -----
 *  ODR is controlled via SAMPLE_RATE_DIV register (page 0, addr 0x1A, 2 bytes).
 *  The sensor base clock is 1 kHz. ODR = 1000 / divisor.
 *  Examples:  divisor=1 ??1000 Hz,  divisor=2 ??500 Hz,  divisor=8 ??125 Hz
 *
 * ----- LPF (BW_CONFIG register, page 0, addr 0x30, 1 byte) -----
 *  Upper nibble = Accel LPF BW:  0x40=BW4, 0x50=BW5, 0x60=BW6, 0x70=BW7
 *  Lower nibble = Gyro  LPF BW:  0x04=BW4, 0x05=BW5, 0x06=BW6, 0x07=BW7
 *  Combined default: 0x44 (both BW4)
 */

#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ---- Command type (from iim4623x.h) ---- */
#define IMU_CMD_TYPE_WRITE_REG   0x12u   /* NOT 0x14 ??that was wrong */
#define IMU_CMD_TYPE_READ_REG    0x11u

/* ---- Base packet sizes (from iim4623x.h) ---- */
#define IMU_SIZE_CMD_WRITE_REGS_BASE  12u   /* overhead bytes for write cmd */
#define IMU_SIZE_RESP_ACK             10u   /* ACK response to any write cmd (SIZE_RESP_ACK) */
#define IMU_SIZE_RESP_READ_BASE       16u   /* base overhead for read-reg response */

/* ---- Register definitions (page 0) ??from iim4623x.c reg map ---- */
#define IMU_PAGE0                0x00u

/* SAMPLE_RATE_DIV: addr=0x1A, length=2, page=0
 * ODR = 1000Hz / divisor
 * Stored big-endian in the packet (high byte first). */
#define IMU_REG_SAMPLE_RATE_DIV_ADDR  0x1Au
#define IMU_REG_SAMPLE_RATE_DIV_LEN   2u

/* BW_CONFIG: addr=0x30, length=1, page=0
 * [7:4] Accel LPF BW,  [3:0] Gyro LPF BW */
#define IMU_REG_BW_CONFIG_ADDR        0x30u
#define IMU_REG_BW_CONFIG_LEN         1u

/* SELECT_OUT_DATA: addr=0x1C, length=1, page=0
 * BIT0=accel BIT1=gyro BIT2=temp BIT3=delta_vel BIT4=delta_ang
 * Packet size = 18 + 4 × (number of set bits) bytes.
 * 0x07 (accel+gyro+temp)  = 46 bytes ??supports 1 kHz
 * 0x1F (all five outputs) = 62 bytes ??sensor throttles DRDY to ~48 Hz */
#define IMU_REG_SELECT_OUT_DATA_ADDR  0x1Cu
#define IMU_SELECT_OUT_ACC_GYRO_TEMP  0x07u   /* recommended for 1 kHz */

/* SAVE_ALL_CONFIG: addr=0x2F, length=1, page=0 ??write 0x50 to trigger */
#define IMU_REG_SAVE_ALL_CONFIG_ADDR  0x2Fu
#define IMU_SAVE_ALL_CONFIG_CMD       0x50u

/* RESET_ALL_CONFIG: addr=0xA8, length=1, page=0 - write 0xA0 to restore
 * the factory-backed flash profile (SAMPLE_RATE_DIV=0x0001,
 * SELECT_OUT_DATA=0x07, BW_CONFIG=0x44). */
#define IMU_REG_RESET_ALL_CONFIG_ADDR 0xA8u
#define IMU_RESET_ALL_CONFIG_CMD      0xA0u

/* ---- SAMPLE_RATE_DIV values (divisors of 1 kHz) ---- */
#define IMU_ODR_DIV_1000HZ   1u    /* 1000 Hz */
#define IMU_ODR_DIV_500HZ    2u    /*  500 Hz */
#define IMU_ODR_DIV_250HZ    4u    /*  250 Hz */
#define IMU_ODR_DIV_200HZ    5u    /*  200 Hz */
#define IMU_ODR_DIV_125HZ    8u    /*  125 Hz */
#define IMU_ODR_DIV_100HZ    10u   /*  100 Hz */
#define IMU_ODR_DIV_50HZ     20u    /*   50 Hz */

/* ---- BW_CONFIG combined values ---- */
/* Accel nibble (upper) */
#define IMU_ACC_LPF_BW4   0x40u
#define IMU_ACC_LPF_BW5   0x50u
#define IMU_ACC_LPF_BW6   0x60u
#define IMU_ACC_LPF_BW7   0x70u
/* Gyro nibble (lower) */
#define IMU_GYRO_LPF_BW4  0x04u
#define IMU_GYRO_LPF_BW5  0x05u
#define IMU_GYRO_LPF_BW6  0x06u
#define IMU_GYRO_LPF_BW7  0x07u
/* Combined defaults (BW4 for both) */
#define IMU_BW_DEFAULT    (IMU_ACC_LPF_BW4 | IMU_GYRO_LPF_BW4)  /* 0x44 */

/**
 * @brief Configure sensor ODR and LPF bandwidth.
 *
 * Sends two WRITE_REG commands (no DRDY wait for either):
 *   1. SAMPLE_RATE_DIV ??sets ODR = 1000 / odr_divisor
 *   2. BW_CONFIG       ??sets accel + gyro LPF bandwidth
 *
 * Call this AFTER reset + WHO_AM_I, BEFORE START_STREAMING.
 *
 * Example for 500 Hz, BW4 filter:
 *   imu_configure_odr_lpf(IMU_ODR_DIV_500HZ, IMU_BW_DEFAULT);
 *   IMU_Handler_Init(500.0f, 0.1f);
 *
 * @param odr_divisor  Divisor for 1kHz base: use IMU_ODR_DIV_* defines
 * @param bw_config    Combined accel+gyro LPF byte: use IMU_BW_DEFAULT or combine nibbles
 * @return 0 on success, non-zero on SPI error
 */
/**
 * @brief Read back key IMU configuration registers and log them via UART.
 *
 * Reads SAMPLE_RATE_DIV, BW_CONFIG, and OUT_DATA_FORM, logs each value
 * with the given tag string so you can distinguish pre/post-write calls.
 *
 * Example:
 *   imu_read_config("PRE");
 *   imu_configure_odr_lpf(IMU_ODR_DIV_1000HZ, IMU_BW_DEFAULT);
 *   imu_read_config("POST");
 *
 * @param tag  Short label printed in every log line (e.g. "PRE", "POST")
 * @return 0 if all reads succeeded, first non-zero error code on failure
 */
int imu_read_config(const char *tag);

int imu_configure_odr_lpf(uint16_t odr_divisor, uint8_t bw_config);
int imu_ensure_default_profile(void);

/** Helper: actual ODR in Hz from divisor */
static inline float imu_odr_hz_from_div(uint16_t div)
{
    return (div > 0u) ? (1000.0f / (float)div) : 1000.0f;
}

#ifdef __cplusplus
}
#endif

#endif /* IMU_CONFIG_H */

