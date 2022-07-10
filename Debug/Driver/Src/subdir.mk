################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Driver/Src/driver_gpio.c \
../Driver/Src/driver_i2c.c \
../Driver/Src/driver_rcc.c \
../Driver/Src/driver_spi.c \
../Driver/Src/driver_uart.c 

OBJS += \
./Driver/Src/driver_gpio.o \
./Driver/Src/driver_i2c.o \
./Driver/Src/driver_rcc.o \
./Driver/Src/driver_spi.o \
./Driver/Src/driver_uart.o 

C_DEPS += \
./Driver/Src/driver_gpio.d \
./Driver/Src/driver_i2c.d \
./Driver/Src/driver_rcc.d \
./Driver/Src/driver_spi.d \
./Driver/Src/driver_uart.d 


# Each subdirectory must supply rules for building sources it contributes
Driver/Src/%.o Driver/Src/%.su: ../Driver/Src/%.c Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F411VETx -DSTM32 -DSTM32F4 -DSTM32F411E_DISCO -c -I"D:/Embedded_C/My_workspace/target/STM32F411DISCO_DRIVER/Driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Driver-2f-Src

clean-Driver-2f-Src:
	-$(RM) ./Driver/Src/driver_gpio.d ./Driver/Src/driver_gpio.o ./Driver/Src/driver_gpio.su ./Driver/Src/driver_i2c.d ./Driver/Src/driver_i2c.o ./Driver/Src/driver_i2c.su ./Driver/Src/driver_rcc.d ./Driver/Src/driver_rcc.o ./Driver/Src/driver_rcc.su ./Driver/Src/driver_spi.d ./Driver/Src/driver_spi.o ./Driver/Src/driver_spi.su ./Driver/Src/driver_uart.d ./Driver/Src/driver_uart.o ./Driver/Src/driver_uart.su

.PHONY: clean-Driver-2f-Src

