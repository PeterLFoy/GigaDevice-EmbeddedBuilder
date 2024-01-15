################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../firmware/cmsis/src/syscalls.c \
../firmware/cmsis/src/system_gd32f3x0.c 

OBJS += \
./firmware/cmsis/src/syscalls.o \
./firmware/cmsis/src/system_gd32f3x0.o 

C_DEPS += \
./firmware/cmsis/src/syscalls.d \
./firmware/cmsis/src/system_gd32f3x0.d 


# Each subdirectory must supply rules for building sources it contributes
firmware/cmsis/src/%.o: ../firmware/cmsis/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GD ARM MCU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -std=gnu11 -DGD32F3X0 -DGD32F310 -I"../inc" -I"../firmware/cmsis/inc" -I"../firmware/GD32f3x0_hal_peripheral/Include" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -Wa,-adhlns=$@.lst   -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


