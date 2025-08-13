################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/app_log.c \
../Core/Src/fpga.c \
../Core/Src/main.c \
../Core/Src/max2850.c \
../Core/Src/max2851.c \
../Core/Src/modem_metrics.c \
../Core/Src/qec.c \
../Core/Src/qec_math.c \
../Core/Src/reg_rw.c \
../Core/Src/rx_capture.c \
../Core/Src/smoke.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c \
../Core/Src/telemetry.c 

OBJS += \
./Core/Src/app_log.o \
./Core/Src/fpga.o \
./Core/Src/main.o \
./Core/Src/max2850.o \
./Core/Src/max2851.o \
./Core/Src/modem_metrics.o \
./Core/Src/qec.o \
./Core/Src/qec_math.o \
./Core/Src/reg_rw.o \
./Core/Src/rx_capture.o \
./Core/Src/smoke.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o \
./Core/Src/telemetry.o 

C_DEPS += \
./Core/Src/app_log.d \
./Core/Src/fpga.d \
./Core/Src/main.d \
./Core/Src/max2850.d \
./Core/Src/max2851.d \
./Core/Src/modem_metrics.d \
./Core/Src/qec.d \
./Core/Src/qec_math.d \
./Core/Src/reg_rw.d \
./Core/Src/rx_capture.d \
./Core/Src/smoke.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d \
./Core/Src/telemetry.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/app_log.cyclo ./Core/Src/app_log.d ./Core/Src/app_log.o ./Core/Src/app_log.su ./Core/Src/fpga.cyclo ./Core/Src/fpga.d ./Core/Src/fpga.o ./Core/Src/fpga.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/max2850.cyclo ./Core/Src/max2850.d ./Core/Src/max2850.o ./Core/Src/max2850.su ./Core/Src/max2851.cyclo ./Core/Src/max2851.d ./Core/Src/max2851.o ./Core/Src/max2851.su ./Core/Src/modem_metrics.cyclo ./Core/Src/modem_metrics.d ./Core/Src/modem_metrics.o ./Core/Src/modem_metrics.su ./Core/Src/qec.cyclo ./Core/Src/qec.d ./Core/Src/qec.o ./Core/Src/qec.su ./Core/Src/qec_math.cyclo ./Core/Src/qec_math.d ./Core/Src/qec_math.o ./Core/Src/qec_math.su ./Core/Src/reg_rw.cyclo ./Core/Src/reg_rw.d ./Core/Src/reg_rw.o ./Core/Src/reg_rw.su ./Core/Src/rx_capture.cyclo ./Core/Src/rx_capture.d ./Core/Src/rx_capture.o ./Core/Src/rx_capture.su ./Core/Src/smoke.cyclo ./Core/Src/smoke.d ./Core/Src/smoke.o ./Core/Src/smoke.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su ./Core/Src/telemetry.cyclo ./Core/Src/telemetry.d ./Core/Src/telemetry.o ./Core/Src/telemetry.su

.PHONY: clean-Core-2f-Src

