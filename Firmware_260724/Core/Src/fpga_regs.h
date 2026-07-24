// fpga_regs.h
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// FPGA Register Map (MCU <-> FPGA) 8-bit register addressing
// Byte order: little-endian for multi-byte values
// ============================================================

#define FPGA_I2C_ADDR7              0x36u

// ---------- CTRL / MODE ----------
#define FPGA_REG_MODE_CURRENT       0x00u // u8
#define FPGA_REG_MODE_FUTURE        0x01u // reserved
#define FPGA_REG_CTRL_DEBUG         0x02u // u8: 0/1
#define FPGA_REG_CTRL_APPLY         0x03u // u8: write 0xA5 to latch (optional)

// ---------- ORIENTATION ----------
// roll/pitch sent in centi-degrees (deg * 100), signed int16
#define FPGA_REG_ORIENT_ROLL_CD8    0x10u // i16 LE at 0x10..0x11
#define FPGA_REG_ORIENT_PITCH_CD8   0x12u // i16 LE at 0x12..0x13
#define FPGA_REG_ORIENT_RATE_HZ8    0x14u // u16 LE at 0x14..0x15 (currently not updated in fast VSYNC path)
#define FPGA_REG_ORIENT_SEQ8        0x16u // u16 LE at 0x16..0x17 (currently not updated in fast VSYNC path)

// ---------- PARAM BANKS ----------
// Per-bank runtime configuration used by the panorama algorithm.
// Layout per bank:
//   yaw_cd[6]    : uint16 LE, centi-degrees
//   pitch_cd[6]  : int16 LE, centi-degrees
//   roll_cd[6]   : int16 LE, centi-degrees
//   overlap_px   : uint16 LE, pixels
//   feather_q16  : int32 LE, Q16.16
#define FPGA_CFG_BANK_EO            0x00u
#define FPGA_CFG_BANK_IR            0x01u

#define FPGA_REG_BASE_CFG_EO        0x20u
#define FPGA_REG_BASE_CFG_IR        0x50u

#define FPGA_REG_CFG_YAW_CD(idx)    ((uint8_t)(0x00u + ((idx) * 2u))) // u16
#define FPGA_REG_CFG_PITCH_CD(idx)  ((uint8_t)(0x0Cu + ((idx) * 2u))) // i16
#define FPGA_REG_CFG_ROLL_CD(idx)   ((uint8_t)(0x18u + ((idx) * 2u))) // i16
#define FPGA_REG_CFG_OVERLAP_U16    0x24u // u16
#define FPGA_REG_CFG_FEATHER_Q16    0x28u // i32

#define FPGA_REG_CFG_BANK_STRIDE    0x30u

static inline uint8_t fpga_cfg_bank_base(uint8_t bank)
{
    return (bank == FPGA_CFG_BANK_IR) ? FPGA_REG_BASE_CFG_IR : FPGA_REG_BASE_CFG_EO;
}

#ifdef __cplusplus
}
#endif
