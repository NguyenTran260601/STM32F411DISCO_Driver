################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Simple\ ASW/gpio_main.c \
../Simple\ ASW/syscalls.c \
../Simple\ ASW/sysmem.c 

OBJS += \
./Simple\ ASW/gpio_main.o \
./Simple\ ASW/syscalls.o \
./Simple\ ASW/sysmem.o 

C_DEPS += \
./Simple\ ASW/gpio_main.d \
./Simple\ ASW/syscalls.d \
./Simple\ ASW/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Simple\ ASW/gpio_main.o: ../Simple\ ASW/gpio_main.c Simple\ ASW/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F411VETx -DSTM32 -DSTM32F4 -DSTM32F411E_DISCO -c -I"D:/Embedded_C/My_workspace/target/STM32F411DISCO_DRIVER/Driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Simple ASW/gpio_main.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Simple\ ASW/syscalls.o: ../Simple\ ASW/syscalls.c Simple\ ASW/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F411VETx -DSTM32 -DSTM32F4 -DSTM32F411E_DISCO -c -I"D:/Embedded_C/My_workspace/target/STM32F411DISCO_DRIVER/Driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Simple ASW/syscalls.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Simple\ ASW/sysmem.o: ../Simple\ ASW/sysmem.c Simple\ ASW/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F411VETx -DSTM32 -DSTM32F4 -DSTM32F411E_DISCO -c -I"D:/Embedded_C/My_workspace/target/STM32F411DISCO_DRIVER/Driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Simple ASW/sysmem.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Simple-20-ASW

clean-Simple-20-ASW:
	-$(RM) ./Simple\ ASW/gpio_main.d ./Simple\ ASW/gpio_main.o ./Simple\ ASW/gpio_main.su ./Simple\ ASW/syscalls.d ./Simple\ ASW/syscalls.o ./Simple\ ASW/syscalls.su ./Simple\ ASW/sysmem.d ./Simple\ ASW/sysmem.o ./Simple\ ASW/sysmem.su

.PHONY: clean-Simple-20-ASW

