
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
 
#ifndef MAIN_H_
#define MAIN_H_

void ext_int_data_ready_cb(void);
void display_main_menu(void);
void char_input_handler(uint8_t input);
void select_data_output_form(uint8_t input);
void select_uart_baud_rate(uint8_t input);
void select_sync_config(uint8_t input);
void select_lpf_bw_accel(uint8_t input);
void select_lpf_bw_gyro(uint8_t input);
void select_fsr_accel(uint8_t input);
void select_fsr_gyro(uint8_t input);
void select_ext_calib_config(uint8_t input);
void set_bias(uint8_t input);
void read_bias(uint8_t input);
void set_sens_mat_accel(uint8_t input);
void set_sens_mat_gyro(uint8_t input);
void read_sens_mat_accel(uint8_t input);
void read_sens_mat_gyro(uint8_t input);
void select_data_output(uint8_t input);
void select_odr(uint8_t input);
void set_sensorft(uint8_t input);

#endif /* MAIN_H_ */
