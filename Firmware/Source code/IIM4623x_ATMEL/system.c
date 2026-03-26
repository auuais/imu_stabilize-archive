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

#include <string.h>
#include <stdarg.h>
#include "stdio_serial.h"
#include "conf_board.h"
#include "system.h"

/* Chip select. */
#define SPI_CHIP_SEL 3
#define SPI_CHIP_PCS spi_get_pcs(SPI_CHIP_SEL)

/* Clock polarity. */
#define SPI_CLK_POLARITY 1

/* Clock phase. */
#define SPI_CLK_PHASE 0

/* Delay before SPCK. */
#define SPI_DLYBS 0x40

/* Delay between consecutive transfers. */
#define SPI_DLYBCT 0x20

/* SPI clock setting (Hz). */
static uint32_t spi_clock = 6000000;

#if !defined(PMC_PCK_PRES_CLK_1)
#define PMC_PCK_PRES_CLK_1   PMC_PCK_PRES(0)
#define PMC_PCK_PRES_CLK_2   PMC_PCK_PRES(1)
#define PMC_PCK_PRES_CLK_4   PMC_PCK_PRES(2)
#define PMC_PCK_PRES_CLK_8   PMC_PCK_PRES(3)
#define PMC_PCK_PRES_CLK_16  PMC_PCK_PRES(4)
#define PMC_PCK_PRES_CLK_32  PMC_PCK_PRES(5)
#define PMC_PCK_PRES_CLK_64  PMC_PCK_PRES(6)
#endif

extern volatile uint32_t ul_ticks;

void spi_master_initialize(void)
{
	/* Configure an SPI peripheral. */
	spi_enable_clock(SPI_MASTER_BASE);

	spi_disable(SPI_MASTER_BASE);
	spi_reset(SPI_MASTER_BASE);
	spi_set_lastxfer(SPI_MASTER_BASE);
	spi_set_master_mode(SPI_MASTER_BASE);
	spi_disable_mode_fault_detect(SPI_MASTER_BASE);
	spi_set_peripheral_chip_select_value(SPI_MASTER_BASE, SPI_CHIP_PCS);
	spi_set_clock_polarity(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI_MASTER_BASE, SPI_CHIP_SEL,
			SPI_CSR_BITS_8_BIT);
	spi_set_baudrate_div(SPI_MASTER_BASE, SPI_CHIP_SEL,	(sysclk_get_peripheral_hz() / spi_clock));
	spi_set_transfer_delay(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_DLYBS,
			SPI_DLYBCT);
	spi_enable(SPI_MASTER_BASE);
}

uint32_t spi_master_write(uint8_t *p_wBuf, uint32_t len)
{
	uint8_t *p_buf = p_wBuf; 
	uint32_t size = len;
	uint32_t i;
	uint8_t uc_pcs = 0;
	uint16_t data = 0;
	uint32_t rv = 0;
	
	for (i = 0; i < size; i++) {
		rv += spi_write(SPI_MASTER_BASE, (uint16_t)p_buf[i], 0, 0);
		/* Wait transfer done. */
		while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_RDRF) == 0);
		rv += spi_read(SPI_MASTER_BASE, &data, &uc_pcs);
	}

	return rv;
}

uint32_t spi_master_read(uint8_t *p_rBuf, uint32_t len)
{
	uint8_t *p_buf = p_rBuf; 
	uint32_t size = len;
	uint32_t i;
	uint8_t uc_pcs = 0;
	uint16_t data = 0;
	uint32_t rv = 0;
	
	for (i = 0; i < size; i++) {
		rv += spi_write(SPI_MASTER_BASE, 0x0, 0, 0);
		/* Wait transfer done. */
		while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_RDRF) == 0);
		rv += spi_read(SPI_MASTER_BASE, &data, &uc_pcs);
		p_buf[i] = (uint8_t)data;
	}

	return rv;
}

uint32_t spi_master_write_read(uint8_t *p_wBuf, uint32_t w_len,  uint8_t *p_rBuf, uint32_t r_len)
{
	uint8_t *p_buf1 = p_wBuf;
	uint8_t *p_buf2 = p_rBuf; 
	uint32_t size;
	uint32_t i;
	uint8_t uc_pcs = 0;
	uint16_t data = 0;
	uint32_t rv = 0;

	size = w_len;
	for (i = 0; i < size; i++) {
		rv += spi_write(SPI_MASTER_BASE, (uint16_t)p_buf1[i], 0, 0);
		/* Wait transfer done. */
		while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_RDRF) == 0);
		rv += spi_read(SPI_MASTER_BASE, &data, &uc_pcs);
		p_buf2[i] = (uint8_t)data;
	}

	size = r_len - w_len;
	for (i = 0; i < size; i++) {
		rv += spi_write(SPI_MASTER_BASE, 0x0, 0, 0);
		/* Wait transfer done. */
		while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_RDRF) == 0);
		rv += spi_read(SPI_MASTER_BASE, &data, &uc_pcs);
		p_buf2[i] = (uint8_t)data;
	}

	return rv;
}

/**
 *  \brief Configure the Console UART.
 */
void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#ifdef CONF_UART_CHAR_LENGTH
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#ifdef CONF_UART_STOP_BITS
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

extern uint8_t rbuf_resp[26];
extern uint8_t rbuf_data[46];

/** Pointer to size of the receive buffer */
uint32_t g_ul_buff_size;

/** Reception is done. */
volatile uint32_t g_ul_recv_done = false;

/** count number for received data. */
volatile uint32_t g_ulcount = 0;

/** Pointer to receive buffer base. */
volatile uint8_t *p_revdata = &rbuf_resp[0];

void configure_uart(void)
{
	usart_serial_options_t uart_serial_options = {
		.baudrate = 921600,
		.charlength = US_MR_CHRL_8_BIT,
		.paritytype = UART_MR_PAR_NO,
		.stopbits = US_MR_NBSTOP_1_BIT,
	};

	/* Configure console UART. */
	usart_serial_init((Usart *)UART4, &uart_serial_options);

	NVIC_ClearPendingIRQ(UART4_IRQn);
	NVIC_DisableIRQ(UART4_IRQn);
	NVIC_SetPriority(UART4_IRQn, 0);
	NVIC_EnableIRQ(UART4_IRQn);
}

void reconfigure_uart_baudrate(uint32_t baud_rate)
{
	NVIC_DisableIRQ(UART4_IRQn);
	sysclk_disable_peripheral_clock(ID_UART4);

	usart_serial_options_t uart_serial_options = {
		.baudrate = baud_rate,
		.charlength = US_MR_CHRL_8_BIT,
		.paritytype = UART_MR_PAR_NO,
		.stopbits = US_MR_NBSTOP_1_BIT,
	};

	usart_serial_init((Usart *)UART4, &uart_serial_options);
	NVIC_EnableIRQ(UART4_IRQn);	
}

uint32_t uart_transmit(uint8_t *p_wBuf, uint32_t len)
{
	uint8_t *p_buf = p_wBuf; 
	uint32_t size = len;

	for (uint32_t i = 0; i < size; i++)
		usart_serial_putchar((Usart *)UART4, p_buf[i]);

	while(!uart_is_tx_empty(UART4)) {} /*waiting for transmit over*/

	uart_enable_interrupt(UART4, UART_IER_RXRDY);

	return 0;
}

uint32_t uart_receive(uint8_t *p_rBuf, uint32_t len)
{
	while (!g_ul_recv_done) {	
	}
	if (g_ul_recv_done) {
		g_ul_recv_done = false;
		g_ulcount = 0;
	}

	return 0;
}

uint32_t uart_rx_tx(uint8_t *p_wBuf, uint32_t w_len,  uint8_t *p_rBuf, uint32_t r_len)
{
	uint8_t *p_buf = p_wBuf; 
	uint32_t size = w_len;

	while (!g_ul_recv_done) {
	}
	if (g_ul_recv_done) {
		g_ul_recv_done = false;
		g_ulcount = 0;
	}

	for (uint32_t i = 0; i < size; i++)
		usart_serial_putchar((Usart *)UART4, p_buf[i]);

	while(!uart_is_tx_empty(UART4)) {} /*waiting for transmit over*/	

	return 0;
}

/**
 * \brief USART IRQ handler.
 *
 * Interrupt handler for USART. After reception is done, set g_ul_recv_done to true.
 *
 */
void UART4_Handler(void)
{
	uint8_t uc_char;

	/* Receive register is full. */
	if(uart_read(UART4, &uc_char) == 0) {
		*p_revdata++ = uc_char;
		g_ulcount++;
		if(g_ulcount >= g_ul_buff_size) {
			uart_disable_interrupt(UART4, UART_IER_RXRDY);
			g_ul_recv_done = true;
		}
	}
}

void ext_int_initialize(void (*handler_function)(void))
{
	/* Enable the peripheral clock for board interrupt pin. */
	pmc_enable_periph_clk(PIN_EXT_INTERRUPT_ID);

	/* Configure PIOs as input pins. */
	pio_configure(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_TYPE, PIN_EXT_INTERRUPT_MASK, PIN_EXT_INTERRUPT_ATTR);

	/* Initialize PIO interrupt handler, interrupt on rising edge. */
	pio_handler_set(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_ID, PIN_EXT_INTERRUPT_MASK, PIN_EXT_INTERRUPT_ATTR, (void (*) (uint32_t, uint32_t))handler_function);

	/* Initialize and enable interrupt. */
	pio_handler_set_priority(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_IRQn, 0);
	pio_enable_interrupt(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_MASK);
}

void enable_pck(void)
{
	pmc_disable_pck(PMC_PCK_4);
	//uint32_t main_clk = sysclk_get_main_hz();
	//uint32_t peripheral_clk = sysclk_get_peripheral_hz();
	//printf("main_clk: %ld peripheral_clk: %ld\n\r", main_clk, peripheral_clk);
	pmc_switch_pck_to_pllack(PMC_PCK_4, PMC_PCK_PRES(11));
	pmc_enable_pck(PMC_PCK_4);	
}

void device_reset(void)
{
	ioport_set_pin_dir(FLA_RESET_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(FLA_RESET_GPIO, FLA_RESET_INACTIVE_LEVEL);
	ioport_set_pin_level(FLA_RESET_GPIO, FLA_RESET_ACTIVE_LEVEL);
	delay_ms(10);
	ioport_set_pin_level(FLA_RESET_GPIO, FLA_RESET_INACTIVE_LEVEL);
}

void SysTick_Handler(void)
{
	ul_ticks++;
}

