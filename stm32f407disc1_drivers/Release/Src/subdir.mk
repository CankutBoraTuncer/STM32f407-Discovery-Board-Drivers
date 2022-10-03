################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/ex1_toggleLed.c \
../Src/ex2_pushButton.c \
../Src/ex3_interrupt.c \
../Src/ex4_spiTest.c \
../Src/ex5_spiArduinoWrite.c \
../Src/ex6_spiArduinoRead.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/ex1_toggleLed.o \
./Src/ex2_pushButton.o \
./Src/ex3_interrupt.o \
./Src/ex4_spiTest.o \
./Src/ex5_spiArduinoWrite.o \
./Src/ex6_spiArduinoRead.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/ex1_toggleLed.d \
./Src/ex2_pushButton.d \
./Src/ex3_interrupt.d \
./Src/ex4_spiTest.d \
./Src/ex5_spiArduinoWrite.d \
./Src/ex6_spiArduinoRead.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/ex1_toggleLed.d ./Src/ex1_toggleLed.o ./Src/ex1_toggleLed.su ./Src/ex2_pushButton.d ./Src/ex2_pushButton.o ./Src/ex2_pushButton.su ./Src/ex3_interrupt.d ./Src/ex3_interrupt.o ./Src/ex3_interrupt.su ./Src/ex4_spiTest.d ./Src/ex4_spiTest.o ./Src/ex4_spiTest.su ./Src/ex5_spiArduinoWrite.d ./Src/ex5_spiArduinoWrite.o ./Src/ex5_spiArduinoWrite.su ./Src/ex6_spiArduinoRead.d ./Src/ex6_spiArduinoRead.o ./Src/ex6_spiArduinoRead.su ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

