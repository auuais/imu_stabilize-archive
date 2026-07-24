/**
 * @file imu_config.c
 * @brief IIM-46234 ODR and LPF configuration.
 *
 * Each WRITE_REG command needs the sensor in a clean command-mode state.
 * IMU_SoftReset() sends STOP_STREAMING (0x28) between commands to return
 * the sensor to command mode WITHOUT a hardware reset ??config survives.
 */

#include "imu_config.h"
#include "main.h"
#include <string.h>

extern int send_imu_command_ex(uint8_t *tx_packet, uint32_t tx_size,
                               uint32_t rx_size, const char *cmd_name,
                               uint8_t *rx_out);
extern void Send_UART3_Debug(const char *format, ...);
extern void IMU_Reset(void);

static uint16_t calc_checksum(const uint8_t *buf, uint32_t len)
{
    uint16_t sum = 0;
    for (uint32_t i = 0; i < len; i++) sum += (uint16_t)buf[i];
    return sum;
}

/* -------------------------------------------------------------------------- */
/* imu_read_reg ??issue a READ_REG command and return the data bytes.        */
/* response = SIZE_RESP_READ_REGS_BASE(16) + data_len bytes.                 */
/* data is at pkt[12 .. 12+data_len-1] (after hdr2+len1+type1+res1+len1+    */
/* addr1+page1+status1+sample1+res2+data_len_echo1 = offset 12).            */
/* -------------------------------------------------------------------------- */
static int imu_read_reg(uint8_t reg_addr, uint8_t page_id,
                        uint8_t data_len, uint8_t *data_out,
                        const char *name)
{
    /* SIZE_CMD_READ_REGS = 12 */
    uint8_t pkt[12] = {0};
    pkt[0] = 0x24u;
    pkt[1] = 0x24u;
    pkt[2] = 0x0Cu;                  /* total_size = 12 */
    pkt[3] = IMU_CMD_TYPE_READ_REG;  /* 0x11 */
    pkt[4] = 0x00u;                  /* reserved */
    pkt[5] = data_len;
    pkt[6] = reg_addr;
    pkt[7] = page_id;
    uint16_t chk = calc_checksum(&pkt[3], 5u);
    pkt[8]  = (uint8_t)(chk >> 8);
    pkt[9]  = (uint8_t)(chk & 0xFFu);
    pkt[10] = 0x0Du;
    pkt[11] = 0x0Au;

    /* Response = 16 base + data_len bytes total */
    uint32_t resp_len = (uint32_t)IMU_SIZE_RESP_READ_BASE + (uint32_t)data_len;
    uint8_t  resp[26] = {0};   /* max: 16 + 10 bytes of data */
    int r = send_imu_command_ex(pkt, sizeof(pkt), resp_len, name, resp);
    if (r != 0) return r;

    /* Data starts at byte 12 in the response (0-indexed from packet start) */
    if (data_out && resp_len >= 13u) {
        for (uint8_t i = 0; i < data_len && (12u + i) < resp_len; i++)
            data_out[i] = resp[12u + i];
    }
    return 0;
}

/**
 * @brief Build a WRITE_REG packet and send it via send_imu_command_ex().
 *
 * Packet format mirrors IIM4623x_SetCMD_WriteRegister() exactly.
 * Like all commands, WRITE_REG generates a DRDY pulse; send_imu_command_ex
 * waits for it, then reads the 10-byte ACK in a new CS transaction.
 */
static int imu_write_reg(uint8_t reg_addr, uint8_t page_id,
                         const uint8_t *data, uint8_t data_len,
                         const char *name)
{
    uint8_t pkt[20] = {0};
    uint8_t total_size = (uint8_t)(IMU_SIZE_CMD_WRITE_REGS_BASE + data_len);

    pkt[0] = 0x24u;
    pkt[1] = 0x24u;
    pkt[2] = total_size;
    pkt[3] = IMU_CMD_TYPE_WRITE_REG;   /* 0x12 */
    pkt[4] = 0x00u;                    /* reserved */
    pkt[5] = data_len;
    pkt[6] = reg_addr;
    pkt[7] = page_id;
    for (uint8_t i = 0; i < data_len; i++) pkt[8 + i] = data[i];

    uint16_t chk = calc_checksum(&pkt[3], (uint32_t)(5u + data_len));
    pkt[8 + data_len]  = (uint8_t)(chk >> 8);
    pkt[9 + data_len]  = (uint8_t)(chk & 0xFFu);
    pkt[10 + data_len] = 0x0Du;   /* footer */
    pkt[11 + data_len] = 0x0Au;

    /* send_imu_command_ex will:
     *   - pad to 20 bytes via build_cmd20
     *   - skip DRDY for WRITE_REG (cmd_type 0x12): ACK arrives immediately
     *   - scan for 0x23 0x23 header, read length, read remaining bytes
     *   - validate footer + checksum
     *   - toggle CS correctly with proper NSS timing
     *   - enforce ?? ms post-response inter-command gap
     */
    uint8_t ack_buf[IMU_SIZE_RESP_ACK] = {0};
    return send_imu_command_ex(pkt, total_size, IMU_SIZE_RESP_ACK, name, ack_buf);
}

/* -------------------------------------------------------------------------- */
/* imu_read_config ??read back the key configuration registers and log them. */
/*                                                                            */
/* Registers read:                                                            */
/*   SAMPLE_RATE_DIV  page=0, addr=0x1A, len=2  ??ODR = 1000 / divisor     */
/*   BW_CONFIG        page=0, addr=0x30, len=1  ??LPF bandwidth             */
/*   OUT_DATA_FORM    page=0, addr=0x19, len=1  ??float(0) or fixed(1)      */
/*                                                                            */
/* Call BEFORE and AFTER imu_configure_odr_lpf() to confirm writes.         */
/* -------------------------------------------------------------------------- */
int imu_read_config(const char *tag)
{
    int any_fail = 0;

    /* ---- SAMPLE_RATE_DIV (2 bytes, big-endian) ---- */
    {
        uint8_t rb[2] = {0};
        int r = imu_read_reg(IMU_REG_SAMPLE_RATE_DIV_ADDR, IMU_PAGE0,
                             IMU_REG_SAMPLE_RATE_DIV_LEN, rb,
                             "RD_SAMPLE_RATE_DIV");
        if (r == 0) {
            uint16_t div = ((uint16_t)rb[0] << 8) | rb[1];
            uint32_t odr = (div > 0u) ? (1000u / (uint32_t)div) : 0u;
            Send_UART3_Debug("[CFG %s] SAMPLE_RATE_DIV=0x%04X  ODR=%lu Hz\r\n",
                             tag, (unsigned)div, (unsigned long)odr);
        } else {
            Send_UART3_Debug("[CFG %s] SAMPLE_RATE_DIV read FAILED r=%d\r\n", tag, r);
            any_fail = r;
        }
    }

    /* ---- BW_CONFIG (1 byte) ---- */
    {
        uint8_t rb = 0;
        int r = imu_read_reg(IMU_REG_BW_CONFIG_ADDR, IMU_PAGE0,
                             IMU_REG_BW_CONFIG_LEN, &rb,
                             "RD_BW_CONFIG");
        if (r == 0) {
            uint8_t acc_bw  = (rb >> 4) & 0x0Fu;
            uint8_t gyro_bw = rb & 0x0Fu;
            Send_UART3_Debug("[CFG %s] BW_CONFIG=0x%02X  accel_bw=%u gyro_bw=%u\r\n",
                             tag, (unsigned)rb,
                             (unsigned)acc_bw, (unsigned)gyro_bw);
        } else {
            Send_UART3_Debug("[CFG %s] BW_CONFIG read FAILED r=%d\r\n", tag, r);
            any_fail = r;
        }
    }

    /* ---- OUT_DATA_FORM (1 byte, addr=0x19) ---- */
    {
        uint8_t rb = 0;
        int r = imu_read_reg(0x19u, IMU_PAGE0, 1u, &rb, "RD_OUT_DATA_FORM");
        if (r == 0) {
            Send_UART3_Debug("[CFG %s] OUT_DATA_FORM=0x%02X  (%s)\r\n",
                             tag, (unsigned)rb,
                             (rb == 0u) ? "float IEEE754" : "fixed 2s-comp");
        } else {
            Send_UART3_Debug("[CFG %s] OUT_DATA_FORM read FAILED r=%d\r\n", tag, r);
            any_fail = r;
        }
    }

    /* ---- SELECT_OUT_DATA (1 byte, addr=0x1C) ---- */
    /* Controls which sensor outputs are included in the stream packet.
     * BIT0=accel BIT1=gyro BIT2=temp BIT3=delta_vel BIT4=delta_ang
     * Default ATMEL session sets 0x1F (all five). That makes packet=62 bytes
     * and the sensor throttles DRDY far below 1kHz. Must be 0x07 for 1kHz. */
    {
        uint8_t rb = 0;
        int r = imu_read_reg(0x1Cu, IMU_PAGE0, 1u, &rb, "RD_SELECT_OUT_DATA");
        if (r == 0) {
            Send_UART3_Debug("[CFG %s] SELECT_OUT_DATA=0x%02X  "
                             "(acc=%u gyro=%u temp=%u dvel=%u dang=%u)  "
                             "pkt=%u bytes\r\n",
                             tag,
                             (unsigned)rb,
                             (unsigned)(rb & 0x01u),
                             (unsigned)((rb >> 1u) & 0x01u),
                             (unsigned)((rb >> 2u) & 0x01u),
                             (unsigned)((rb >> 3u) & 0x01u),
                             (unsigned)((rb >> 4u) & 0x01u),
                             (unsigned)(18u + 4u * __builtin_popcount(rb)));
        } else {
            Send_UART3_Debug("[CFG %s] SELECT_OUT_DATA read FAILED r=%d\r\n", tag, r);
            any_fail = r;
        }
    }

    return any_fail;
}

static int imu_read_reg_after_reset(uint8_t reg_addr, uint8_t page_id,
                                    uint8_t data_len, uint8_t *data_out,
                                    const char *name)
{
    IMU_Reset();
    HAL_Delay(10u);
    return imu_read_reg(reg_addr, page_id, data_len, data_out, name);
}

static int imu_write_reg_after_reset(uint8_t reg_addr, uint8_t page_id,
                                     const uint8_t *data, uint8_t data_len,
                                     const char *name)
{
    IMU_Reset();
    HAL_Delay(10u);
    return imu_write_reg(reg_addr, page_id, data, data_len, name);
}

int imu_ensure_default_profile(void)
{
    uint8_t sample_div_rb[2] = {0};
    uint8_t select_out_rb = 0u;
    uint8_t bw_config_rb = 0u;
    int r;

    IMU_Reset();
    HAL_Delay(10u);

    r = imu_read_reg(IMU_REG_SAMPLE_RATE_DIV_ADDR, IMU_PAGE0,
                     IMU_REG_SAMPLE_RATE_DIV_LEN, sample_div_rb,
                     "RD_DEFAULT_SAMPLE_RATE_DIV");
    if (r == 0) {
        r = imu_read_reg(IMU_REG_SELECT_OUT_DATA_ADDR, IMU_PAGE0,
                         1u, &select_out_rb,
                         "RD_DEFAULT_SELECT_OUT_DATA");
    }
    if (r == 0) {
        r = imu_read_reg(IMU_REG_BW_CONFIG_ADDR, IMU_PAGE0,
                         IMU_REG_BW_CONFIG_LEN, &bw_config_rb,
                         "RD_DEFAULT_BW_CONFIG");
    }

    if (r == 0) {
        Send_UART3_Debug("[ODR] Back-to-back command probe OK without reset\r\n");
    } else {
        Send_UART3_Debug("[ODR] Back-to-back command probe FAILED r=%d, retrying with reset-per-command\r\n", r);

        r = imu_read_reg_after_reset(IMU_REG_SAMPLE_RATE_DIV_ADDR, IMU_PAGE0,
                                     IMU_REG_SAMPLE_RATE_DIV_LEN, sample_div_rb,
                                     "RD_DEFAULT_SAMPLE_RATE_DIV");
        if (r != 0) {
            Send_UART3_Debug("[ODR] SAMPLE_RATE_DIV probe FAILED r=%d\r\n", r);
            return r;
        }

        r = imu_read_reg_after_reset(IMU_REG_SELECT_OUT_DATA_ADDR, IMU_PAGE0,
                                     1u, &select_out_rb,
                                     "RD_DEFAULT_SELECT_OUT_DATA");
        if (r != 0) {
            Send_UART3_Debug("[ODR] SELECT_OUT_DATA probe FAILED r=%d\r\n", r);
            return r;
        }

        r = imu_read_reg_after_reset(IMU_REG_BW_CONFIG_ADDR, IMU_PAGE0,
                                     IMU_REG_BW_CONFIG_LEN, &bw_config_rb,
                                     "RD_DEFAULT_BW_CONFIG");
        if (r != 0) {
            Send_UART3_Debug("[ODR] BW_CONFIG probe FAILED r=%d\r\n", r);
            return r;
        }
    }

    {
        const uint16_t sample_div =
            (uint16_t)(((uint16_t)sample_div_rb[0] << 8) | sample_div_rb[1]);
        const int need_restore =
            (sample_div != IMU_ODR_DIV_1000HZ) ||
            (select_out_rb != IMU_SELECT_OUT_ACC_GYRO_TEMP) ||
            (bw_config_rb != IMU_BW_DEFAULT);

        Send_UART3_Debug("[ODR] Flash profile: SAMPLE_RATE_DIV=0x%04X SELECT_OUT_DATA=0x%02X BW_CONFIG=0x%02X\r\n",
                         (unsigned)sample_div,
                         (unsigned)select_out_rb,
                         (unsigned)bw_config_rb);

        if (!need_restore) {
            Send_UART3_Debug("[ODR] Flash profile already at factory 1 kHz defaults\r\n");
            return 0;
        }
    }

    {
        uint8_t reset_cmd = IMU_RESET_ALL_CONFIG_CMD;
        r = imu_write_reg_after_reset(IMU_REG_RESET_ALL_CONFIG_ADDR, IMU_PAGE0,
                                      &reset_cmd, 1u, "RESET_ALL_CONFIG");
        if (r != 0) {
            Send_UART3_Debug("[ODR] RESET_ALL_CONFIG FAILED r=%d\r\n", r);
            return r;
        }
        Send_UART3_Debug("[ODR] RESET_ALL_CONFIG=0x%02X ACK, waiting for flash save\r\n",
                         (unsigned)reset_cmd);
        HAL_Delay(250u);
    }

    r = imu_read_reg_after_reset(IMU_REG_SAMPLE_RATE_DIV_ADDR, IMU_PAGE0,
                                 IMU_REG_SAMPLE_RATE_DIV_LEN, sample_div_rb,
                                 "RD_RESTORED_SAMPLE_RATE_DIV");
    if (r != 0) {
        Send_UART3_Debug("[ODR] SAMPLE_RATE_DIV verify FAILED r=%d\r\n", r);
        return r;
    }

    r = imu_read_reg_after_reset(IMU_REG_SELECT_OUT_DATA_ADDR, IMU_PAGE0,
                                 1u, &select_out_rb,
                                 "RD_RESTORED_SELECT_OUT_DATA");
    if (r != 0) {
        Send_UART3_Debug("[ODR] SELECT_OUT_DATA verify FAILED r=%d\r\n", r);
        return r;
    }

    r = imu_read_reg_after_reset(IMU_REG_BW_CONFIG_ADDR, IMU_PAGE0,
                                 IMU_REG_BW_CONFIG_LEN, &bw_config_rb,
                                 "RD_RESTORED_BW_CONFIG");
    if (r != 0) {
        Send_UART3_Debug("[ODR] BW_CONFIG verify FAILED r=%d\r\n", r);
        return r;
    }

    {
        const uint16_t sample_div =
            (uint16_t)(((uint16_t)sample_div_rb[0] << 8) | sample_div_rb[1]);
        if ((sample_div != IMU_ODR_DIV_1000HZ) ||
            (select_out_rb != IMU_SELECT_OUT_ACC_GYRO_TEMP) ||
            (bw_config_rb != IMU_BW_DEFAULT)) {
            Send_UART3_Debug("[ODR] Flash restore verify mismatch: SAMPLE_RATE_DIV=0x%04X SELECT_OUT_DATA=0x%02X BW_CONFIG=0x%02X\r\n",
                             (unsigned)sample_div,
                             (unsigned)select_out_rb,
                             (unsigned)bw_config_rb);
            return -1;
        }
    }

    Send_UART3_Debug("[ODR] Flash profile restored to factory 1 kHz defaults\r\n");
    return 0;
}

int imu_configure_odr_lpf(uint16_t odr_divisor, uint8_t bw_config)
{
    int r;

    /* Keep a short idle gap between commands so the next falling-edge DRDY
     * belongs to the command we just sent. */
    extern volatile int drdy_interrupt;
#define IMU_INTER_CMD_CLEAR() do { drdy_interrupt = 0; HAL_Delay(10u); } while(0)

    /* 1) SAMPLE_RATE_DIV: 2 bytes, big-endian. */
    uint8_t div_bytes[2];
    div_bytes[0] = (uint8_t)(odr_divisor >> 8);
    div_bytes[1] = (uint8_t)(odr_divisor & 0xFFu);
    r = imu_write_reg(IMU_REG_SAMPLE_RATE_DIV_ADDR, IMU_PAGE0,
                      div_bytes, IMU_REG_SAMPLE_RATE_DIV_LEN,
                      "SAMPLE_RATE_DIV");
    if (r != 0) {
        Send_UART3_Debug("[ODR] SAMPLE_RATE_DIV write FAILED r=%d\r\n", r);
        return r;
    }
    Send_UART3_Debug("[ODR] SAMPLE_RATE_DIV=0x%04X (ODR=%u Hz) OK\r\n",
                     (unsigned)odr_divisor,
                     (unsigned)((odr_divisor > 0u) ? (1000u / odr_divisor) : 0u));
    IMU_INTER_CMD_CLEAR();
    HAL_Delay(100u);
    Send_UART3_Debug("[ODR] Leaving SELECT_OUT_DATA/BW_CONFIG unchanged in single-write path\r\n");

#undef IMU_INTER_CMD_CLEAR
    return 0;
}

