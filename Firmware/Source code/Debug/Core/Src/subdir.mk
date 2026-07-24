################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/fpga_i2c.c \
../Core/Src/imu_config.c \
../Core/Src/imu_handler.c \
../Core/Src/imu_madgwick.c \
../Core/Src/main.c \
../Core/Src/mcu_fsm.c \
../Core/Src/orientation_kalman.c \
../Core/Src/proto_uart.c \
../Core/Src/stm32g0xx_hal_msp.c \
../Core/Src/stm32g0xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g0xx.c 

OBJS += \
./Core/Src/fpga_i2c.o \
./Core/Src/imu_config.o \
./Core/Src/imu_handler.o \
./Core/Src/imu_madgwick.o \
./Core/Src/main.o \
./Core/Src/mcu_fsm.o \
./Core/Src/orientation_kalman.o \
./Core/Src/proto_uart.o \
./Core/Src/stm32g0xx_hal_msp.o \
./Core/Src/stm32g0xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g0xx.o 

C_DEPS += \
./Core/Src/fpga_i2c.d \
./Core/Src/imu_config.d \
./Core/Src/imu_handler.d \
./Core/Src/imu_madgwick.d \
./Core/Src/main.d \
./Core/Src/mcu_fsm.d \
./Core/Src/orientation_kalman.d \
./Core/Src/proto_uart.d \
./Core/Src/stm32g0xx_hal_msp.d \
./Core/Src/stm32g0xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G0C1xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/fpga_i2c.cyclo ./Core/Src/fpga_i2c.d ./Core/Src/fpga_i2c.o ./Core/Src/fpga_i2c.su ./Core/Src/imu_config.cyclo ./Core/Src/imu_config.d ./Core/Src/imu_config.o ./Core/Src/imu_config.su ./Core/Src/imu_handler.cyclo ./Core/Src/imu_handler.d ./Core/Src/imu_handler.o ./Core/Src/imu_handler.su ./Core/Src/imu_madgwick.cyclo ./Core/Src/imu_madgwick.d ./Core/Src/imu_madgwick.o ./Core/Src/imu_madgwick.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mcu_fsm.cyclo ./Core/Src/mcu_fsm.d ./Core/Src/mcu_fsm.o ./Core/Src/mcu_fsm.su ./Core/Src/orientation_kalman.cyclo ./Core/Src/orientation_kalman.d ./Core/Src/orientation_kalman.o ./Core/Src/orientation_kalman.su ./Core/Src/proto_uart.cyclo ./Core/Src/proto_uart.d ./Core/Src/proto_uart.o ./Core/Src/proto_uart.su ./Core/Src/stm32g0xx_hal_msp.cyclo ./Core/Src/stm32g0xx_hal_msp.d ./Core/Src/stm32g0xx_hal_msp.o ./Core/Src/stm32g0xx_hal_msp.su ./Core/Src/stm32g0xx_it.cyclo ./Core/Src/stm32g0xx_it.d ./Core/Src/stm32g0xx_it.o ./Core/Src/stm32g0xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g0xx.cyclo ./Core/Src/system_stm32g0xx.d ./Core/Src/system_stm32g0xx.o ./Core/Src/system_stm32g0xx.su

.PHONY: clean-Core-2f-Src

