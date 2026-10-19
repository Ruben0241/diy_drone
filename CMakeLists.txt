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

add_executable(firmware.elf
    # Startup aus dem ST-CMSIS-Paket
    ${LIB}/cmsis_device_f7/Source/Templates/gcc/startup_stm32f722xx.s

    # Hardwareunabhaengiger Kern
    src/core/crsf.c
    src/core/dshot.c
    src/core/flight.c
    src/core/mahony.c
    src/core/mixer.c
    src/core/pid.c
    src/core/pt1.c

    # Board GEPRC TAKER F722
    src/board/system.c
    src/board/icm42688.c
    src/board/rx_uart.c
    src/board/dshot_tim3.c
    src/board/peripherals.c
    src/board/syscalls.c

    # Anwendung
    src/app/main.c

    # FreeRTOS
    ${RTOS}/tasks.c
    ${RTOS}/queue.c
    ${RTOS}/list.c
    ${RTOS}/portable/GCC/ARM_CM4F/port.c
    ${RTOS}/portable/MemMang/heap_4.c
)

target_compile_definitions(firmware.elf PRIVATE STM32F722xx)

target_include_directories(firmware.elf PRIVATE
    src/core
    src/board
    src/app
    ${LIB}/cmsis_device_f7/Include
    ${LIB}/cmsis_core/CMSIS/Core/Include
    ${RTOS}/include
    ${RTOS}/portable/GCC/ARM_CM4F
)

# Strenge Warnungen nur fuer den eigenen Code
set_source_files_properties(
    src/core/crsf.c src/core/dshot.c src/core/flight.c src/core/mahony.c
    src/core/mixer.c src/core/pid.c src/core/pt1.c
    src/board/system.c src/board/icm42688.c src/board/rx_uart.c
    src/board/dshot_tim3.c src/board/peripherals.c src/app/main.c
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
