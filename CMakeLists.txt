cmake_minimum_required(VERSION 3.16)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi.cmake)
endif()

project(diy_drone_fc C ASM)

set(CMAKE_C_STANDARD 11)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_C_FLAGS_RELEASE "-O2")

set(LIB ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(RTOS ${LIB}/FreeRTOS-Kernel)
set(TUSB ${LIB}/tinyusb/src)
set(GEN_DIR ${CMAKE_BINARY_DIR}/generated)

add_custom_target(fw_version
    COMMAND ${CMAKE_COMMAND} -DSRC_DIR=${CMAKE_CURRENT_SOURCE_DIR} -DOUT=${GEN_DIR}/fw_version.h
            -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/gen_version.cmake
    BYPRODUCTS ${GEN_DIR}/fw_version.h
)

add_executable(firmware.elf
    # Startup aus dem ST-CMSIS-Paket
    ${LIB}/cmsis_device_f7/Source/Templates/gcc/startup_stm32f722xx.s

    # Hardwareunabhaengiger Kern
    src/core/biquad.c
    src/core/blackbox.c
    src/core/crsf.c
    src/core/dshot.c
    src/core/flight.c
    src/core/logrec.c
    src/core/mahony.c
    src/core/mixer.c
    src/core/params.c
    src/core/pid.c
    src/core/pt1.c
    src/core/rates.c

    # Board GEPRC TAKER F722
    src/board/system.c
    src/board/spi.c
    src/board/flash_w25q.c
    src/board/icm42688.c
    src/board/rx_uart.c
    src/board/dshot_tim3.c
    src/board/peripherals.c
    src/board/syscalls.c

    # Anwendung
    src/app/main.c
    src/app/config_store.c
    src/app/usb_cli.c
    src/app/usb_descriptors.c

    # FreeRTOS
    ${RTOS}/tasks.c
    ${RTOS}/queue.c
    ${RTOS}/list.c
    ${RTOS}/portable/GCC/ARM_CM4F/port.c
    ${RTOS}/portable/MemMang/heap_4.c

    # TinyUSB (USB-Geraet, CDC)
    ${TUSB}/tusb.c
    ${TUSB}/common/tusb_fifo.c
    ${TUSB}/device/usbd.c
    ${TUSB}/device/usbd_control.c
    ${TUSB}/class/cdc/cdc_device.c
    ${TUSB}/portable/synopsys/dwc2/dcd_dwc2.c
    ${TUSB}/portable/synopsys/dwc2/dwc2_common.c
)
add_dependencies(firmware.elf fw_version)

target_compile_definitions(firmware.elf PRIVATE STM32F722xx)

target_include_directories(firmware.elf PRIVATE
    src/core
    src/board
    src/app
    ${LIB}/cmsis_device_f7/Include
    ${LIB}/cmsis_core/CMSIS/Core/Include
    ${RTOS}/include
    ${RTOS}/portable/GCC/ARM_CM4F
    ${TUSB}
    ${GEN_DIR}
)

# Strenge Warnungen nur fuer den eigenen Code
set_source_files_properties(
    src/core/biquad.c src/core/blackbox.c src/core/crsf.c src/core/dshot.c src/core/flight.c
    src/core/logrec.c src/core/mahony.c src/core/mixer.c src/core/params.c src/core/pid.c
    src/core/pt1.c src/core/rates.c src/app/config_store.c
    src/board/system.c src/board/spi.c src/board/flash_w25q.c src/board/icm42688.c
    src/board/rx_uart.c src/board/dshot_tim3.c src/board/peripherals.c
    src/app/main.c src/app/usb_cli.c src/app/usb_descriptors.c
    PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Werror;-Wdouble-promotion"
)

target_link_options(firmware.elf PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/ld/stm32f722re.ld
    -Wl,-Map=${CMAKE_BINARY_DIR}/firmware.map
    -Wl,--print-memory-usage
)
target_link_libraries(firmware.elf PRIVATE m)

add_custom_command(TARGET firmware.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary firmware.elf firmware.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex firmware.elf firmware.hex
    COMMAND ${CMAKE_SIZE} firmware.elf
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Bei Aenderungen am Linkerskript neu linken
set_target_properties(firmware.elf PROPERTIES LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/ld/stm32f722re.ld)
