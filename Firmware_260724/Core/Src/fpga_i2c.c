// fpga_i2c.c
#include "fpga_i2c.h"

static inline uint16_t addr8(uint8_t addr7) { return (uint16_t)(addr7 << 1); }

void fpga_i2c_init(fpga_i2c_t* dev, I2C_HandleTypeDef* hi2c, uint8_t addr7)
{
  if (!dev) return;
  dev->hi2c  = hi2c;
  dev->addr7 = addr7;
}

HAL_StatusTypeDef fpga_is_ready(fpga_i2c_t* dev, uint32_t trials, uint32_t timeout_ms)
{
  if (!dev || !dev->hi2c) return HAL_ERROR;
  return HAL_I2C_IsDeviceReady(dev->hi2c, addr8(dev->addr7), trials, timeout_ms);
}

HAL_StatusTypeDef fpga_write_reg8(fpga_i2c_t* dev, uint8_t reg, uint8_t value, uint32_t timeout_ms)
{
  if (!dev || !dev->hi2c) return HAL_ERROR;

  return HAL_I2C_Mem_Write(dev->hi2c,
                           addr8(dev->addr7),
                           reg,
                           I2C_MEMADD_SIZE_8BIT,
                           &value,
                           1,
                           timeout_ms);
}

HAL_StatusTypeDef fpga_write_regs(fpga_i2c_t* dev, uint8_t reg, const uint8_t* data, uint16_t len, uint32_t timeout_ms)
{
  if (!dev || !dev->hi2c || !data || (len == 0u)) return HAL_ERROR;

  return HAL_I2C_Mem_Write(dev->hi2c,
                           addr8(dev->addr7),
                           reg,
                           I2C_MEMADD_SIZE_8BIT,
                           (uint8_t*)data,
                           len,
                           timeout_ms);
}

HAL_StatusTypeDef fpga_read_reg8(fpga_i2c_t* dev, uint8_t reg, uint8_t* out, uint32_t timeout_ms)
{
  if (!dev || !dev->hi2c || !out) return HAL_ERROR;

  return HAL_I2C_Mem_Read(dev->hi2c,
                          addr8(dev->addr7),
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          out,
                          1,
                          timeout_ms);
}

HAL_StatusTypeDef fpga_write_read_reg8(fpga_i2c_t* dev, uint8_t reg, uint8_t wr, uint8_t* rd, uint32_t timeout_ms)
{
  HAL_StatusTypeDef st = fpga_write_reg8(dev, reg, wr, timeout_ms);
  if (st != HAL_OK) return st;
  return fpga_read_reg8(dev, reg, rd, timeout_ms);
}
