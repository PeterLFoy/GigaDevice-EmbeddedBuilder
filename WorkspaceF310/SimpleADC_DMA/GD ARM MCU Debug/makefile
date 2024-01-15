################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include firmware/gd32f3x0_hal_peripheral/Source/subdir.mk
-include firmware/cmsis/src/subdir.mk
-include firmware/cmsis/gcc_startup/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH_BIN += \
SimpleADC_DMA.bin \

SECONDARY_LIST += \
SimpleADC_DMA.lst \

SECONDARY_SIZE += \
SimpleADC_DMA.siz \


# All Target
all: SimpleADC_DMA.elf secondary-outputs

# Tool invocations
SimpleADC_DMA.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GD ARM MCU Linker'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -Wl,-Map,"SimpleADC_DMA.map" -T gd32f3x0_flash.ld -Xlinker --gc-sections -L"../ldscripts" --specs=nano.specs --specs=nosys.specs -o "SimpleADC_DMA.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

SimpleADC_DMA.bin: SimpleADC_DMA.elf
	@echo 'Invoking: GD ARM MCU Flash Image(Bin)'
	arm-none-eabi-objcopy -O binary  "SimpleADC_DMA.elf"  "SimpleADC_DMA.bin"
	@echo 'Finished building: $@'
	@echo ' '

SimpleADC_DMA.lst: SimpleADC_DMA.elf
	@echo 'Invoking: GD ARM MCU Listing'
	arm-none-eabi-objdump --source --all-headers --demangle --line-numbers --wide "SimpleADC_DMA.elf" > "SimpleADC_DMA.lst"
	@echo 'Finished building: $@'
	@echo ' '

SimpleADC_DMA.siz: SimpleADC_DMA.elf
	@echo 'Invoking: GD ARM MCU Print Size'
	arm-none-eabi-size --format=berkeley "SimpleADC_DMA.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(SECONDARY_FLASH_BIN)$(S_UPPER_DEPS)$(C_DEPS) SimpleADC_DMA.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH_BIN) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
