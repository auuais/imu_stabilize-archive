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
// All float parameters are quantized as Q16.16 int32 before transfer.
// overlap_px is sent as signed int32.
#define FPGA_CFG_BANK_EO            0x00u
#define FPGA_CFG_BANK_IR            0x01u

#define FPGA_REG_BASE_CFG_EO        0x20u
#define FPGA_REG_BASE_CFG_IR        0x50u

#define FPGA_REG_CFG_CYL_H_FOV_Q16  0x00u // i32
#define FPGA_REG_CFG_CYL_V_FOV_Q16  0x04u // i32
#define FPGA_REG_CFG_CROP_H_Q16     0x08u // i32
#define FPGA_REG_CFG_CROP_W_Q16     0x0Cu // i32
#define FPGA_REG_CFG_PITCH_TR_Q16   0x10u // i32 (effective pitch factor)
#define FPGA_REG_CFG_YAW_TR_Q16     0x14u // i32
#define FPGA_REG_CFG_OVERLAP_I32    0x18u // i32
#define FPGA_REG_CFG_FEATHER_Q16    0x1Cu // i32

#define FPGA_REG_CFG_BANK_STRIDE    0x30u

static inline uint8_t fpga_cfg_bank_base(uint8_t bank)
{
    return (bank == FPGA_CFG_BANK_IR) ? FPGA_REG_BASE_CFG_IR : FPGA_REG_BASE_CFG_EO;
}

#ifdef __cplusplus
}
#endif
