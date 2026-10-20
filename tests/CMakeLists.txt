cmake_minimum_required(VERSION 3.16)
project(diy_drone_fc_tests C)

set(CMAKE_C_STANDARD 11)
set(CORE ${CMAKE_CURRENT_SOURCE_DIR}/../src/core)

add_executable(test_core
    test_core.c
    ${CORE}/biquad.c
    ${CORE}/blackbox.c
    ${CORE}/params.c
    ${CORE}/rates.c
    ${CORE}/crsf.c
    ${CORE}/logrec.c
    ${CORE}/dshot.c
    ${CORE}/flight.c
    ${CORE}/mahony.c
    ${CORE}/mixer.c
    ${CORE}/pid.c
    ${CORE}/pt1.c
)
target_include_directories(test_core PRIVATE ${CORE})
target_compile_options(test_core PRIVATE -Wall -Wextra -Werror)
target_link_libraries(test_core m)

enable_testing()
add_test(NAME core COMMAND test_core ${CMAKE_BINARY_DIR}/sample_log.bin)

# Firmware-Kern als Bibliothek fuer die Physik-Simulation (tools/sim)
add_library(fc_core SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/../tools/sim/sim_api.c
    ${CORE}/biquad.c ${CORE}/crsf.c ${CORE}/dshot.c ${CORE}/flight.c ${CORE}/mahony.c
    ${CORE}/mixer.c ${CORE}/params.c ${CORE}/pid.c ${CORE}/pt1.c ${CORE}/rates.c
)
target_include_directories(fc_core PRIVATE ${CORE})
target_compile_options(fc_core PRIVATE -Wall -Wextra -Werror -O2)
target_link_libraries(fc_core m)
