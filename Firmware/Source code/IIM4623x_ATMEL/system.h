/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include <asf.h>

void spi_master_initialize(void);
uint32_t spi_master_write(uint8_t *p_wBuf, uint32_t len);
uint32_t spi_master_read(uint8_t *p_rBuf, uint32_t len);
uint32_t spi_master_write_read(uint8_t *p_wBuf, uint32_t w_len,  uint8_t *p_rBuf, uint32_t r_len);
void configure_console(void);
void configure_uart(void);
void reconfigure_uart_baudrate(uint32_t baud_rate);
uint32_t uart_transmit(uint8_t *p_wBuf, uint32_t len);
uint32_t uart_receive(uint8_t *p_rBuf, uint32_t len);
uint32_t uart_rx_tx(uint8_t *p_wBuf, uint32_t w_len,  uint8_t *p_rBuf, uint32_t r_len);
void ext_int_initialize(void (*handler_function)(void));
void enable_pck(void);
void device_reset(void);

#endif /* SYSTEM_H_ */