// fpga_i2c.h
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "stm32g0xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  I2C_HandleTypeDef* hi2c;
  uint8_t addr7;   // 7-bit (not shifted)
} fpga_i2c_t;

void fpga_i2c_init(fpga_i2c_t* dev, I2C_HandleTypeDef* hi2c, uint8_t addr7);

HAL_StatusTypeDef fpga_is_ready(fpga_i2c_t* dev, uint32_t trials, uint32_t timeout_ms);

HAL_StatusTypeDef fpga_write_reg8(fpga_i2c_t* dev, uint8_t reg, uint8_t value, uint32_t timeout_ms);
HAL_StatusTypeDef fpga_write_regs(fpga_i2c_t* dev, uint8_t reg, const uint8_t* data, uint16_t len, uint32_t timeout_ms);
HAL_StatusTypeDef fpga_read_reg8 (fpga_i2c_t* dev, uint8_t reg, uint8_t* out,  uint32_t timeout_ms);

// Convenience for your smoke test (write then read same reg)
HAL_StatusTypeDef fpga_write_read_reg8(fpga_i2c_t* dev, uint8_t reg, uint8_t wr, uint8_t* rd, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
