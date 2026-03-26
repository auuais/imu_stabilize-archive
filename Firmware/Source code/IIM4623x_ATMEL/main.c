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
 
#include <asf.h>
#include <string.h>
#include "conf_clock.h"
#include "system.h"
#include "main.h"
#include "../iim4623x/iim4623x.h"

#define STRING_EOL    "\r"
#define STRING_HEADER "-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

#define SERIF_TYPE_SPI	1
#define SERIF_TYPE_UART	0
#define DATA_STREAM_LOG	1

#if SERIF_TYPE_SPI
	int bus_type = 0;  // SPI
#elif SERIF_TYPE_UART
	int bus_type = 1;  // UART
#else
	int bus_type = -1; // error
#endif

#if DATA_STREAM_LOG
	int enable_log = 1;
#else
	int enable_log = 0;
#endif

#define MICROSECONDS_PER_SECOND	1000000

volatile int drdy_interrupt = 0;
volatile uint32_t ul_ticks = 0;
volatile uint64_t timestamp;

void ext_int_data_ready_cb(void)
{
	drdy_interrupt = 1;
	timestamp = ul_ticks;
}

/**
 * \brief Display the user menu on the terminal.
 */
void display_main_menu(void)
{
	printf("\r\n");
	printf("============ Command Mode Menu ============\r\n");
	printf(" Press 'r' to reset IIM4623x             \r\n");
	printf(" Press 'w' to read WHO_AM_I              \r\n");
	printf(" Press 'i' to read Serial Number         \r\n");
	printf(" Press 'v' to get Version                \r\n");
	printf(" Press 't' to invoke Self Test           \r\n");
	printf(" Press 'f' to select Data Output Form    \r\n");
	printf(" Press 'b' to select UART Baud Rate      \r\n");
	printf(" Press 'y' to select Sync Config         \r\n");
	printf(" Press 'h' to set UTC time               \r\n");
	printf(" Press 'l' to select BW for Accel LPF    \r\n");
	printf(" Press 'm' to select BW for Gyro LPF     \r\n");
	printf(" Press 'a' to select FSR for Accel       \r\n");
	printf(" Press 'g' to select FSR for Gyro        \r\n");
	printf(" Press 'c' to configure Ext. Calib.      \r\n");
	printf(" Press 'e' to set Bias              	 \r\n");
	printf(" Press 'u' to read Bias              	 \r\n");	
	printf(" Press 'd' to select Data Output         \r\n");	
	printf(" Press 'o' to select ODR                 \r\n");
	printf(" Press 's' to start streaming            \r\n");
	printf(" Press 'j' to set SensorFT               \r\n");
	printf(" Press 'k' to set Sens. Mat for Accel    \r\n");
	printf(" Press 'n' to set Sens. Mat for Gyro     \r\n");
	printf(" Press 'q' to read Sens. Mat for Accel   \r\n");
	printf(" Press 'x' to read Sens. Mat for Gyro    \r\n");
	printf("=========== Streaming Mode Menu ===========\r\n");
	printf(" Press 'p' to stop streaming             \r\n");
	printf(" Press 'h' to set UTC time               \r\n");
	printf("============= Firmware Update =============\r\n");
	printf(" Press ',' to start firmware update      \r\n");
	printf("===========================================\r\n");
	printf("\r\n");
}

void char_input_handler(uint8_t input)
{
	if (Is_Command_Mode()) {
		switch (input) {
		case 'r':
			IIM4623x_Initialize();
			printf("Device was reset \n\r");
			break;
		case 'w':
			IIM4623x_Read_WhoAmI();
			printf("Sent command to read WHO_AM_I \n\r");			
			break;
		case 'i':
			IIM4623x_Get_SerialNum();
			printf("Sent command to get serial number \n\r");
			break;
		case 'v':
			IIM4623x_Get_Version();
			printf("Sent command to get firmware version \n\r");
			break;
		case 't':
			IIM4623x_Imu_SelfTest();
			printf("Sent command for IMU self-test (Wait for a while) \n\r");
			break;
		case 'f':
			printf("\r\n");
			printf(" Press '0' to select Data Output Form as Floating IEEE 754    \r\n");
			printf(" Press '1' to select Data Output Form as Fixed 2's Complement \r\n");
			printf("\r\n");
			break;
		case 'b':
			printf("\r\n");
			printf(" Press '0' to select UART Baud Rate as 921600  \r\n");
			printf(" Press '1' to select UART Baud Rate as 1500000 \r\n");
			printf(" Press '2' to select UART Baud Rate as 3000000 \r\n");
			printf("\r\n");			
			break;
		case 'y':
			printf("\r\n");
			printf(" Press '0' to select Sync Config as 'Disable Sync'  \r\n");
			printf(" Press '1' to select Sync Config as 'Sync with PPS' \r\n");
			printf("\r\n");			
			break;
		case 'h':
		{
			utc time_pps;
			printf("\r\n");
			printf("Setting UTC time begins.. \r\n");
			
			printf("Enter year (yyyy) and then press 'Enter': ");
			scanf("%hu", &time_pps.year);
			printf("%hu \r\n", time_pps.year);
			
			printf("Enter month (mm) and then press 'Enter': ");
			scanf("%hu", &time_pps.month);
			printf("%hu \r\n", time_pps.month);
			
			printf("Enter day (dd) and then press 'Enter': ");
			scanf("%hu", &time_pps.day);
			printf("%hu \r\n", time_pps.day);
			
			printf("Enter hour (hh) and then press 'Enter': ");
			scanf("%hu", &time_pps.hh);
			printf("%hu \r\n", time_pps.hh);
			
			printf("Enter minute (mm) and then press 'Enter': ");
			scanf("%hu", &time_pps.mm);
			printf("%hu \r\n", time_pps.mm);
			
			printf("Enter second (ss) and then press 'Enter': ");
			scanf("%hu", &time_pps.ss);
			printf("%hu \r\n", time_pps.ss);
			printf("\r\n");

			IIM4623x_Set_UtcTime(time_pps);
			break;
		}
		case 'l':
			printf("\r\n");
			printf(" Press '0' to select BW for Accel LPF as BW4 \r\n");
			printf(" Press '1' to select BW for Accel LPF as BW5 \r\n");
			printf(" Press '2' to select BW for Accel LPF as BW6 \r\n");
			printf(" Press '3' to select BW for Accel LPF as BW7 \r\n");
			printf("\r\n");				
			break;
		case 'm':
			printf("\r\n");
			printf(" Press '0' to select BW for Gyro LPF as BW4 \r\n");
			printf(" Press '1' to select BW for Gyro LPF as BW5 \r\n");
			printf(" Press '2' to select BW for Gyro LPF as BW6 \r\n");
			printf(" Press '3' to select BW for Gyro LPF as BW7 \r\n");
			printf("\r\n");				
			break;
		case 'a':
		{
			PartNum part = IIM4623x_Get_PartNum();
			printf("\r\n");
			if (part == IIM46230) {
				printf(" Press '0' to select FSR for Accel as +-16g \r\n");
				printf(" Press '1' to select FSR for Accel as +-8g \r\n");
				printf(" Press '2' to select FSR for Accel as +-4g \r\n");
				printf(" Press '3' to select FSR for Accel as +-2g \r\n");
			}
			else if (part == IIM46234) {
				printf(" Press '0' to select FSR for Accel as +-8g \r\n");
				printf(" Press '1' to select FSR for Accel as +-4g \r\n");
				printf(" Press '2' to select FSR for Accel as +-2g \r\n");				
			}
			printf("\r\n");				
			break;
		}
		case 'g':
		{
			PartNum part = IIM4623x_Get_PartNum();
			printf("\r\n");
			if (part == IIM46230) {
				printf(" Press '0' to select FSR for Gyro as +-2000dps \r\n");
				printf(" Press '1' to select FSR for Gyro as +-1000dps \r\n");
				printf(" Press '2' to select FSR for Gyro as +-500dps \r\n");
				printf(" Press '3' to select FSR for Gyro as +-250dps \r\n");			
			}
			else if (part == IIM46234) {
				printf(" Press '0' to select FSR for Gyro as +-480dps \r\n");
				printf(" Press '1' to select FSR for Gyro as +-250dps \r\n");
			}
			printf("\r\n");				
			break;
		}
		case 'c':
			printf("\r\n");
			printf(" Press '0' to toggle Bias Calibration for Accel  \r\n");
			printf(" Press '1' to toggle Bias Calibration for Gyro   \r\n");
			printf(" Press '2' to toggle Sens. Mat. Calib. for Accel \r\n");
			printf(" Press '3' to toggle Sens. Mat. Calib. for Gyro  \r\n");
			printf("\r\n");				
			break;
		case 'e':
			printf("\r\n");
			printf(" Press '0' to set Bias for Accel X-axis arbitrarily \r\n");
			printf(" Press '1' to set Bias for Accel Y-axis arbitrarily \r\n");
			printf(" Press '2' to set Bias for Accel Z-axis arbitrarily \r\n");
			printf(" Press '3' to set Bias for Gyro X-axis arbitrarily \r\n");
			printf(" Press '4' to set Bias for Gyro Y-axis arbitrarily \r\n");
			printf(" Press '5' to set Bias for Gyro Z-axis arbitrarily \r\n");			
			printf("\r\n");				
			break;
		case 'u':
			printf("\r\n");
			printf(" Press '0' to read Bias for Accel X-axis \r\n");
			printf(" Press '1' to read Bias for Accel Y-axis \r\n");
			printf(" Press '2' to read Bias for Accel Z-axis \r\n");
			printf(" Press '3' to read Bias for Gyro X-axis \r\n");
			printf(" Press '4' to read Bias for Gyro Y-axis \r\n");
			printf(" Press '5' to read Bias for Gyro Z-axis \r\n");			
			printf("\r\n");				
			break;				
		case 'd':
			printf("\r\n");
			printf(" Press '0' to toggle Accel  	    \r\n");
			printf(" Press '1' to toggle Gyro		    \r\n");
			printf(" Press '2' to toggle Temperature    \r\n");	
			printf(" Press '3' to toggle Delta Velocity \r\n");
			printf(" Press '4' to toggle Delta Angle	\r\n");
			printf("\r\n");				
			break;
		case 'o':
			printf("\r\n");
			printf(" Press '0' to set ODR @ 10Hz  \r\n");
			printf(" Press '1' to set ODR @ 20Hz  \r\n");
			printf(" Press '2' to set ODR @ 25Hz  \r\n");
			printf(" Press '3' to set ODR @ 50Hz  \r\n");
			printf(" Press '4' to set ODR @ 100Hz \r\n");
			printf(" Press '5' to set ODR @ 125Hz \r\n");
			printf(" Press '6' to set ODR @ 200Hz \r\n");
			printf(" Press '7' to set ODR @ 250Hz \r\n");
			printf(" Press '8' to set ODR @ 500Hz \r\n");
			printf(" Press '9' to set ODR @ 1kHz  \r\n");
			printf("\r\n");			
			break;			
		case 's':
			IIM4623x_Set_StartStreaming();
			IIM4623x_Start_Streaming();
			break;
		case 'j':
			printf("\r\n");
			printf(" Press '0' to disable SensorFT \r\n");
			printf(" Press '1' to enable SensorFT  \r\n");
			printf("\r\n");
			break;
		case 'k':
			printf("\r\n");
			printf(" Press '0' to set Sens. Mat. 11 for Accel arbitrarily \r\n");
			printf(" Press '1' to set Sens. Mat. 12 for Accel arbitrarily \r\n");
			printf(" Press '2' to set Sens. Mat. 13 for Accel arbitrarily \r\n");
			printf(" Press '3' to set Sens. Mat. 21 for Accel arbitrarily \r\n");
			printf(" Press '4' to set Sens. Mat. 22 for Accel arbitrarily \r\n");
			printf(" Press '5' to set Sens. Mat. 23 for Accel arbitrarily \r\n");
			printf(" Press '6' to set Sens. Mat. 31 for Accel arbitrarily \r\n");
			printf(" Press '7' to set Sens. Mat. 32 for Accel arbitrarily \r\n");
			printf(" Press '8' to set Sens. Mat. 33 for Accel arbitrarily \r\n");	
			printf("\r\n");				
			break;
		case 'n':
			printf("\r\n");
			printf(" Press '0' to set Sens. Mat. 11 for Gyro arbitrarily \r\n");
			printf(" Press '1' to set Sens. Mat. 12 for Gyro arbitrarily \r\n");
			printf(" Press '2' to set Sens. Mat. 13 for Gyro arbitrarily \r\n");
			printf(" Press '3' to set Sens. Mat. 21 for Gyro arbitrarily \r\n");
			printf(" Press '4' to set Sens. Mat. 22 for Gyro arbitrarily \r\n");
			printf(" Press '5' to set Sens. Mat. 23 for Gyro arbitrarily \r\n");
			printf(" Press '6' to set Sens. Mat. 31 for Gyro arbitrarily \r\n");
			printf(" Press '7' to set Sens. Mat. 32 for Gyro arbitrarily \r\n");
			printf(" Press '8' to set Sens. Mat. 33 for Gyro arbitrarily \r\n");
			printf("\r\n");				
			break;
		case 'q':
			printf("\r\n");
			printf(" Press '0' to read Sens. Mat. 11 for Accel arbitrarily \r\n");
			printf(" Press '1' to read Sens. Mat. 12 for Accel arbitrarily \r\n");
			printf(" Press '2' to read Sens. Mat. 13 for Accel arbitrarily \r\n");
			printf(" Press '3' to read Sens. Mat. 21 for Accel arbitrarily \r\n");
			printf(" Press '4' to read Sens. Mat. 22 for Accel arbitrarily \r\n");
			printf(" Press '5' to read Sens. Mat. 23 for Accel arbitrarily \r\n");
			printf(" Press '6' to read Sens. Mat. 31 for Accel arbitrarily \r\n");
			printf(" Press '7' to read Sens. Mat. 32 for Accel arbitrarily \r\n");
			printf(" Press '8' to read Sens. Mat. 33 for Accel arbitrarily \r\n");			
			printf("\r\n");				
			break;
		case 'x':
			printf("\r\n");
			printf(" Press '0' to read Sens. Mat. 11 for Gyro arbitrarily \r\n");
			printf(" Press '1' to read Sens. Mat. 12 for Gyro arbitrarily \r\n");
			printf(" Press '2' to read Sens. Mat. 13 for Gyro arbitrarily \r\n");
			printf(" Press '3' to read Sens. Mat. 21 for Gyro arbitrarily \r\n");
			printf(" Press '4' to read Sens. Mat. 22 for Gyro arbitrarily \r\n");
			printf(" Press '5' to read Sens. Mat. 23 for Gyro arbitrarily \r\n");
			printf(" Press '6' to read Sens. Mat. 31 for Gyro arbitrarily \r\n");
			printf(" Press '7' to read Sens. Mat. 32 for Gyro arbitrarily \r\n");
			printf(" Press '8' to read Sens. Mat. 33 for Gyro arbitrarily \r\n");			
			printf("\r\n");				
			break;
		case ',':
			IIM4623x_Switch_to_Bootloader();
			printf("Sent SWITCH_TO_BOOTLOADER command \n\r");
			break;
		default:
			display_main_menu();
			break;
		}
	}
	else {
		switch (input) {
		case 'p':
			IIM4623x_Set_StopStreaming();
			break;
		case 'h':
			IIM4623x_Set_UtcTimeInStreaming();
			break;			
		default:
			break;			
		}
	}
}

void select_data_output_form(uint8_t input)
{
	if (Is_Command_Mode()) {
		switch (input) {
		case '0':
			IIM4623x_Set_OutDataForm(FLOATING);
			printf("Sent command to set OUT_DATA_FORM (Floating Point) \n\r");
			break;
		case '1':
			IIM4623x_Set_OutDataForm(FIXED);
			printf("Sent command to set OUT_DATA_FORM (FIXED Point) \n\r");
			break;	
		default:
			break;
		}
	}
}

void select_uart_baud_rate(uint8_t input)
{
	if (Is_Command_Mode()) {
		switch (input) {
		case '0':
			IIM4623x_Set_UartIfConfig(BAUD_921600);
			printf("Sent command to set UART_IF_CONFIG (921600) \n\r");
			break;
		case '1':
			IIM4623x_Set_UartIfConfig(BAUD_1500000);
			printf("Sent command to set UART_IF_CONFIG (1500000) \n\r");
			break;
		case '2':
			IIM4623x_Set_UartIfConfig(BAUD_3000000);
			printf("Sent command to set UART_IF_CONFIG (3000000) \n\r");
			break;
		default:
			break;
		}
	}
}

void select_sync_config(uint8_t input)
{
	if (Is_Command_Mode()) {
		switch (input) {
		case '0':
			IIM4623x_Set_SyncConfig(DISABLE_SYNC);
			printf("Sent command to set SYNC_CONFIG (Disable) \n\r");
			break;
		case '1':
			IIM4623x_Set_SyncConfig(SYNC_WITH_PPS);
			printf("Sent command to set SYNC_CONFIG (Sync with PPS) \n\r");
			break;	
		default:
			break;
		}
	}
}

void select_lpf_bw_accel(uint8_t input)
{
	if (Is_Command_Mode()) {
		switch (input) {
		case '0':
			IIM4623x_Set_BWConfig_Accel(ACC_LPF_BW4);
			printf("Sent command to set BW_CONFIG (BW4 for Accel) \n\r");
			break;
		case '1':
			IIM4623x_Set_BWConfig_Accel(ACC_LPF_BW5);
			printf("Sent command to set BW_CONFIG (BW5 for Accel) \n\r");
			break;
		case '2':
			IIM4623x_Set_BWConfig_Accel(ACC_LPF_BW6);
			printf("Sent command to set BW_CONFIG (BW6 for Accel) \n\r");
			break;
		case '3':
			IIM4623x_Set_BWConfig_Accel(ACC_LPF_BW7);
			printf("Sent command to set BW_CONFIG (BW7 for Accel) \n\r");
			break;				
		default:
			break;
		}
	}
}

void select_lpf_bw_gyro(uint8_t input)
{
	if (Is_Command_Mode()) {
		switch (input) {
		case '0':
			IIM4623x_Set_BWConfig_Gyro(GYRO_LPF_BW4);
			printf("Sent command to set BW_CONFIG (BW4 for Gyro) \n\r");
			break;
		case '1':
			IIM4623x_Set_BWConfig_Gyro(GYRO_LPF_BW5);
			printf("Sent command to set BW_CONFIG (BW5 for Gyro) \n\r");
			break;
		case '2':
			IIM4623x_Set_BWConfig_Gyro(GYRO_LPF_BW6);
			printf("Sent command to set BW_CONFIG (BW6 for Gyro) \n\r");
			break;
		case '3':
			IIM4623x_Set_BWConfig_Gyro(GYRO_LPF_BW7);
			printf("Sent command to set BW_CONFIG (BW7 for Gyro) \n\r");
			break;				
		default:
			break;
		}
	}
}

void select_fsr_accel(uint8_t input)
{
	if (Is_Command_Mode()) {
		PartNum part = IIM4623x_Get_PartNum();

		if (part == IIM46230) {
			switch (input) {
			case '0':
				IIM4623x_Set_AccelConfig0(ACC_FSR_16G);
				printf("Sent command to set FSR (+-16g for Accel) \n\r");
				break;				
			case '1':
				IIM4623x_Set_AccelConfig0(ACC_FSR_8G);
				printf("Sent command to set FSR (+-8g for Accel) \n\r");
				break;
			case '2':
				IIM4623x_Set_AccelConfig0(ACC_FSR_4G);
				printf("Sent command to set FSR (+-4g for Accel) \n\r");
				break;
			case '3':
				IIM4623x_Set_AccelConfig0(ACC_FSR_2G);
				printf("Sent command to set FSR (+-2g for Accel) \n\r");
				break;
			default:
				break;
			}
		}	
		else if (part == IIM46234) {
			switch (input) {
			case '0':
				IIM4623x_Set_AccelConfig0(ACC_FSR_8G);
				printf("Sent command to set FSR (+-8g for Accel) \n\r");
				break;
			case '1':
				IIM4623x_Set_AccelConfig0(ACC_FSR_4G);
				printf("Sent command to set FSR (+-4g for Accel) \n\r");
				break;
			case '2':
				IIM4623x_Set_AccelConfig0(ACC_FSR_2G);
				printf("Sent command to set FSR (+-2g for Accel) \n\r");
				break;
			default:
				break;
			}
		}
	}
}

void select_fsr_gyro(uint8_t input)
{
	if (Is_Command_Mode()) {
		PartNum part = IIM4623x_Get_PartNum();

		if (part == IIM46230) {
			switch (input) {
			case '0':
				IIM4623x_Set_GyroConfig0(GYRO_FSR_2000DPS);
				printf("Sent command to set FSR (+-2000dps for Gyro) \n\r");
				break;
			case '1':
				IIM4623x_Set_GyroConfig0(GYRO_FSR_1000DPS);
				printf("Sent command to set FSR (+-1000dps for Gyro) \n\r");
				break;				
			case '2':
				IIM4623x_Set_GyroConfig0(GYRO_FSR_500DPS);
				printf("Sent command to set FSR (+-500dps for Gyro) \n\r");
				break;
			case '3':
				IIM4623x_Set_GyroConfig0(GYRO_FSR_250DPS);
				printf("Sent command to set FSR (+-250dps for Gyro) \n\r");
				break;
			default:
				break;
			}
		}
		else if (part == IIM46234) {
			switch (input) {
			case '0':
				IIM4623x_Set_GyroConfig0(GYRO_FSR_480DPS);
				printf("Sent command to set FSR (+-480dps for Gyro) \n\r");
				break;
			case '1':
				IIM4623x_Set_GyroConfig0(GYRO_FSR_250DPS);
				printf("Sent command to set FSR (+-250dps for Gyro) \n\r");
				break;
			default:
				break;
			}
		}
	}
}

void select_ext_calib_config(uint8_t input)
{
	if (Is_Command_Mode()) {
		switch (input) {
		case '0':
			IIM4623x_Toggle_ExtCalibConfig(ACCEL_BIAS);
			printf("Sent command to toggle Accel Bias Config \n\r");
			break;
		case '1':
			IIM4623x_Toggle_ExtCalibConfig(GYRO_BIAS);
			printf("Sent command to toggle Gyro Bias Config \n\r");
			break;
		case '2':
			IIM4623x_Toggle_ExtCalibConfig(ACCEL_SENS);
			printf("Sent command to toggle Accel Sens. Mat. Config \n\r");
			break;
		case '3':
			IIM4623x_Toggle_ExtCalibConfig(GYRO_SENS);
			printf("Sent command to toggle Accel Sens. Mat. Config \n\r");
			break;			
		default:
			break;
		}
	}
}

void set_bias(uint8_t input)
{
	if (Is_Command_Mode()) {
		switch (input) {
		case '0':
			IIM4623x_Write_Bias(ACCEL_X, 1.5f);
			printf("Sent command to set Bias for Accel X-axis arbitrarily \n\r");
			break;
		case '1':
			IIM4623x_Write_Bias(ACCEL_Y, 1.5f);
			printf("Sent command to set Bias for Accel Y-axis arbitrarily \n\r");
			break;
		case '2':
			IIM4623x_Write_Bias(ACCEL_Z, 1.5f);
			printf("Sent command to set Bias for Accel Z-axis arbitrarily \n\r");
			break;
		case '3':
			IIM4623x_Write_Bias(GYRO_X, 1.5f);
			printf("Sent command to set Bias for Gyro X-axis arbitrarily \n\r");
			break;
		case '4':
			IIM4623x_Write_Bias(GYRO_Y, 1.5f);
			printf("Sent command to set Bias for Gyro Y-axis arbitrarily \n\r");
			break;
		case '5':
			IIM4623x_Write_Bias(GYRO_Z, 1.5f);
			printf("Sent command to set Bias for Gyro Z-axis arbitrarily \n\r");
			break;			
		default:
			break;
		}
	}
}

void read_bias(uint8_t input)
{
	if (Is_Command_Mode()) {
		switch (input) {
		case '0':
			IIM4623x_Read_Bias(ACCEL_X);
			printf("Sent command to read Bias for Accel X-axis \n\r");
			break;
		case '1':
			IIM4623x_Read_Bias(ACCEL_Y);
			printf("Sent command to read Bias for Accel Y-axis \n\r");
			break;
		case '2':
			IIM4623x_Read_Bias(ACCEL_Z);
			printf("Sent command to read Bias for Accel Z-axis \n\r");
			break;
		case '3':
			IIM4623x_Read_Bias(GYRO_X);
			printf("Sent command to read Bias for Gyro X-axis \n\r");
			break;
		case '4':
			IIM4623x_Read_Bias(GYRO_Y);
			printf("Sent command to read Bias for Gyro X-axis \n\r");
			break;
		case '5':
			IIM4623x_Read_Bias(GYRO_Z);
			printf("Sent command to read Bias for Gyro X-axis \n\r");
			break;			
		default:
			break;
		}
	}
}

void set_sens_mat_accel(uint8_t input)
{
	if (Is_Command_Mode()) {
		switch (input) {
		case '0':
			IIM4623x_Write_Sens_Mat_Accel(X_1, 0.0f);
			printf("Sent command to set Accel Sens. Mat. 11 arbitrarily \n\r");
			break;
		case '1':
			IIM4623x_Write_Sens_Mat_Accel(X_2, 1.0f);
			printf("Sent command to set Accel Sens. Mat. 12 arbitrarily \n\r");
			break;
		case '2':
			IIM4623x_Write_Sens_Mat_Accel(X_3, 1.0f);
			printf("Sent command to set Accel Sens. Mat. 13 arbitrarily \n\r");
			break;
		case '3':
			IIM4623x_Write_Sens_Mat_Accel(Y_1, 1.0f);
			printf("Sent command to set Accel Sens. Mat. 21 arbitrarily \n\r");
			break;
		case '4':
			IIM4623x_Write_Sens_Mat_Accel(Y_2, 0.0f);
			printf("Sent command to set Accel Sens. Mat. 22 arbitrarily \n\r");
			break;
		case '5':
			IIM4623x_Write_Sens_Mat_Accel(Y_3, 1.0f);
			printf("Sent command to set Accel Sens. Mat. 23 arbitrarily \n\r");
			break;
		case '6':
			IIM4623x_Write_Sens_Mat_Accel(Z_1, 1.0f);
			printf("Sent command to set Accel Sens. Mat. 31 arbitrarily \n\r");
			break;
		case '7':
			IIM4623x_Write_Sens_Mat_Accel(Z_2, 1.0f);
			printf("Sent command to set Accel Sens. Mat. 32 arbitrarily \n\r");
			break;
		case '8':
			IIM4623x_Write_Sens_Mat_Accel(Z_3, 0.0f);
			printf("Sent command to set Accel Sens. Mat. 33 arbitrarily \n\r");
			break;
		default:
			break;
		}
	}
}

void set_sens_mat_gyro(uint8_t input)
{
	if (Is_Command_Mode()) {
		switch (input) {
		case '0':
			IIM4623x_Write_Sens_Mat_Gyro(X_1, 0.0f);
			printf("Sent command to set Gyro Sens. Mat. 11 arbitrarily \n\r");
			break;
		case '1':
			IIM4623x_Write_Sens_Mat_Gyro(X_2, 1.0f);
			printf("Sent command to set Gyro Sens. Mat. 12 arbitrarily \n\r");
			break;
		case '2':
			IIM4623x_Write_Sens_Mat_Gyro(X_3, 2.0f);
			printf("Sent command to set Gyro Sens. Mat. 13 arbitrarily \n\r");
			break;
		case '3':
			IIM4623x_Write_Sens_Mat_Gyro(Y_1, 3.0f);
			printf("Sent command to set Gyro Sens. Mat. 21 arbitrarily \n\r");
			break;
		case '4':
			IIM4623x_Write_Sens_Mat_Gyro(Y_2, 4.0f);
			printf("Sent command to set Gyro Sens. Mat. 22 arbitrarily \n\r");
			break;
		case '5':
			IIM4623x_Write_Sens_Mat_Gyro(Y_3, 5.0f);
			printf("Sent command to set Gyro Sens. Mat. 23 arbitrarily \n\r");
			break;
		case '6':
			IIM4623x_Write_Sens_Mat_Gyro(Z_1, 6.0f);
			printf("Sent command to set Gyro Sens. Mat. 31 arbitrarily \n\r");
			break;
		case '7':
			IIM4623x_Write_Sens_Mat_Gyro(Z_2, 7.0f);
			printf("Sent command to set Gyro Sens. Mat. 32 arbitrarily \n\r");
			break;
		case '8':
			IIM4623x_Write_Sens_Mat_Gyro(Z_3, 8.0f);
			printf("Sent command to set Gyro Sens. Mat. 33 arbitrarily \n\r");
			break;
		default:
			break;
		}
	}
}

void read_sens_mat_accel(uint8_t input)
{
	if (Is_Command_Mode()) {
		switch (input) {
		case '0':
			IIM4623x_Read_Sens_Mat_Accel(X_1);
			printf("Sent command to read Accel Sens. Mat. 11 arbitrarily \n\r");
			break;
		case '1':
			IIM4623x_Read_Sens_Mat_Accel(X_2);
			printf("Sent command to read Accel Sens. Mat. 12 arbitrarily \n\r");
			break;
		case '2':
			IIM4623x_Read_Sens_Mat_Accel(X_3);
			printf("Sent command to read Accel Sens. Mat. 13 arbitrarily \n\r");
			break;
		case '3':
			IIM4623x_Read_Sens_Mat_Accel(Y_1);
			printf("Sent command to read Accel Sens. Mat. 21 arbitrarily \n\r");
			break;
		case '4':
			IIM4623x_Read_Sens_Mat_Accel(Y_2);
			printf("Sent command to read Accel Sens. Mat. 22 arbitrarily \n\r");
			break;
		case '5':
			IIM4623x_Read_Sens_Mat_Accel(Y_3);
			printf("Sent command to read Accel Sens. Mat. 23 arbitrarily \n\r");
			break;
		case '6':
			IIM4623x_Read_Sens_Mat_Accel(Z_1);
			printf("Sent command to read Accel Sens. Mat. 31 arbitrarily \n\r");
			break;
		case '7':
			IIM4623x_Read_Sens_Mat_Accel(Z_2);
			printf("Sent command to read Accel Sens. Mat. 32 arbitrarily \n\r");
			break;
		case '8':
			IIM4623x_Read_Sens_Mat_Accel(Z_3);
			printf("Sent command to read Accel Sens. Mat. 33 arbitrarily \n\r");
			break;
		default:
			break;
		}
	}
}

void read_sens_mat_gyro(uint8_t input)
{
	if (Is_Command_Mode()) {
		switch (input) {
		case '0':
			IIM4623x_Read_Sens_Mat_Gyro(X_1);
			printf("Sent command to read Gyro Sens. Mat. 11 arbitrarily \n\r");
			break;
		case '1':
			IIM4623x_Read_Sens_Mat_Gyro(X_2);
			printf("Sent command to read Gyro Sens. Mat. 12 arbitrarily \n\r");
			break;
		case '2':
			IIM4623x_Read_Sens_Mat_Gyro(X_3);
			printf("Sent command to read Gyro Sens. Mat. 13 arbitrarily \n\r");
			break;
		case '3':
			IIM4623x_Read_Sens_Mat_Gyro(Y_1);
			printf("Sent command to read Gyro Sens. Mat. 21 arbitrarily \n\r");
			break;
		case '4':
			IIM4623x_Read_Sens_Mat_Gyro(Y_2);
			printf("Sent command to read Gyro Sens. Mat. 22 arbitrarily \n\r");
			break;
		case '5':
			IIM4623x_Read_Sens_Mat_Gyro(Y_3);
			printf("Sent command to read Gyro Sens. Mat. 23 arbitrarily \n\r");
			break;
		case '6':
			IIM4623x_Read_Sens_Mat_Gyro(Z_1);
			printf("Sent command to read Gyro Sens. Mat. 31 arbitrarily \n\r");
			break;
		case '7':
			IIM4623x_Read_Sens_Mat_Gyro(Z_2);
			printf("Sent command to read Gyro Sens. Mat. 32 arbitrarily \n\r");
			break;
		case '8':
			IIM4623x_Read_Sens_Mat_Gyro(Z_3);
			printf("Sent command to read Gyro Sens. Mat. 33 arbitrarily \n\r");
			break;
		default:
			break;
		}
	}
}

void select_data_output(uint8_t input)
{
	if (Is_Command_Mode()) {
		switch (input) {
		case '0':
			IIM4623x_Toggle_DataOutput(ACCEL);
			printf("Sent command to toggle Accel \n\r");
			break;
		case '1':
			IIM4623x_Toggle_DataOutput(GYRO);
			printf("Sent command to toggle Gyro \n\r");
			break;
		case '2':
			IIM4623x_Toggle_DataOutput(TEMP);
			printf("Sent command to toggle Temperature \n\r");
			break;
		case '3':
			IIM4623x_Toggle_DataOutput(DELTA_VEL);
			printf("Sent command to toggle Delta Velocity \n\r");
			break;
		case '4':
			IIM4623x_Toggle_DataOutput(DELTA_ANG);
			printf("Sent command to toggle Delta Angle \n\r");
			break;
		default:
			break;
		}
	}
}

void select_odr(uint8_t input)
{
	if (Is_Command_Mode()) {
		switch (input) {
		case '0':
			IIM4623x_Set_SampleRateDiv(ODR_10HZ);
			printf("Sent command to set SAMPLE_RATE_DIV (10Hz) \n\r");
			break;
		case '1':
			IIM4623x_Set_SampleRateDiv(ODR_20HZ);
			printf("Sent command to set SAMPLE_RATE_DIV (20Hz) \n\r");
			break;
		case '2':
			IIM4623x_Set_SampleRateDiv(ODR_25HZ);
			printf("Sent command to set SAMPLE_RATE_DIV (25Hz) \n\r");
			break;
		case '3':
			IIM4623x_Set_SampleRateDiv(ODR_50HZ);
			printf("Sent command to set SAMPLE_RATE_DIV (50Hz) \n\r");
			break;
		case '4':
			IIM4623x_Set_SampleRateDiv(ODR_100HZ);
			printf("Sent command to set SAMPLE_RATE_DIV (100Hz) \n\r");
			break;
		case '5':
			IIM4623x_Set_SampleRateDiv(ODR_125HZ);
			printf("Sent command to set SAMPLE_RATE_DIV (125Hz) \n\r");
			break;
		case '6':
			IIM4623x_Set_SampleRateDiv(ODR_200HZ);
			printf("Sent command to set SAMPLE_RATE_DIV (200Hz) \n\r");
			break;
		case '7':
			IIM4623x_Set_SampleRateDiv(ODR_250HZ);
			printf("Sent command to set SAMPLE_RATE_DIV (250Hz) \n\r");
			break;
		case '8':
			IIM4623x_Set_SampleRateDiv(ODR_500HZ);
			printf("Sent command to set SAMPLE_RATE_DIV (500Hz) \n\r");
			break;
		case '9':
			IIM4623x_Set_SampleRateDiv(ODR_1KHZ);
			printf("Sent command to set SAMPLE_RATE_DIV (1kHz) \n\r");
			break;		
		default:
			break;
		}
	}
}

void set_sensorft(uint8_t input)
{
	if (Is_Command_Mode()) {
		switch (input) {
		case '0':
			IIM4623x_Set_SensorFT(false);
			printf("Sent command to disable SensorFT \n\r");
			break;
		case '1':
			IIM4623x_Set_SensorFT(true);
			printf("Sent command to enable SensorFT \n\r");
			break;	
		default:
			break;
		}
	}
}

/**
 * \brief Application entry point for SPI example.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	uint8_t uc_key = 0;
	uint8_t uc_pre_key = 0;

	/* Initialize the SAM system. */
	sysclk_init();
	board_init();

	/* Initialize the console UART. */
	configure_console();

	printf("\r\n");
	printf("============================================= \n\r");
	printf("=  IIM4623x Example for ATSAMV71Q21 (%s) = \n\r", ((bus_type==0)?"SPI":"UART"));
	printf("============================================= \n\r");

	/* Output example information. */
	puts(STRING_HEADER);
	printf("\r\n");

	ext_int_initialize(ext_int_data_ready_cb);
	SysTick_Config(sysclk_get_cpu_hz() / MICROSECONDS_PER_SECOND);

	//enable_pck();	

	drdy_interrupt = 0;
	if (bus_type == 0) {
		spi_master_initialize();
		IIM4623x_set_serif(spi_master_read, spi_master_write, spi_master_write_read);
	}
	else if (bus_type == 1) {
		configure_uart();
		IIM4623x_set_serif(uart_receive, uart_transmit, uart_rx_tx);
	}
	else {
		printf("Serial IF setting error \n\r");
	}
	IIM4623x_set_reset(device_reset);
	IIM4623x_Initialize();

	/* Display menu. */
	display_main_menu();

	while (1) {
		if (usart_serial_is_rx_ready(CONF_UART)) {
			usart_serial_getchar(CONF_UART, &uc_key);

			switch(uc_pre_key) {
			case 'f':
				select_data_output_form(uc_key);
				break;
			case 'b':
				select_uart_baud_rate(uc_key);
				break;
			case 'y':
				select_sync_config(uc_key);
				break;
			case 'l':
				select_lpf_bw_accel(uc_key);
				break;
			case 'm':
				select_lpf_bw_gyro(uc_key);
				break;
			case 'a':
				select_fsr_accel(uc_key);
				break;
			case 'g':
				select_fsr_gyro(uc_key);
				break;
			case 'c':
				select_ext_calib_config(uc_key);
				break;
			case 'e':
				set_bias(uc_key);
				break;
			case 'u':
				read_bias(uc_key);
				break;
			case 'd':
				select_data_output(uc_key);
				break;
			case 'o':
				select_odr(uc_key);
				break;
			case 'j':
				set_sensorft(uc_key);
				break;
			case 'k':
				set_sens_mat_accel(uc_key);
				break;
			case 'n':
				set_sens_mat_gyro(uc_key);
				break;
			case 'q':
				read_sens_mat_accel(uc_key);
				break;
			case 'x':
				read_sens_mat_gyro(uc_key);
				break;
			default:
				char_input_handler(uc_key);
				break;
			}
		}
		
		uc_pre_key = uc_key;

		if (drdy_interrupt == 1) {
			drdy_interrupt = 0;
			if (response_handler()) {
				break;
			}
		}
	}
}

