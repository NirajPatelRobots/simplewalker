cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(simplewalker_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_ENABLE_CXX_EXCEPTIONS 1)
pico_sdk_init()

add_executable(hardware_test
hardwareTest.cpp
motors.cpp
sensorReader.cpp
)
# pull in common dependencies
target_link_libraries(hardware_test pico_stdlib hardware_pwm hardware_spi)
# enable usb output, disable uart output
pico_enable_stdio_usb(hardware_test 1)
pico_enable_stdio_uart(hardware_test 0)
# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(hardware_test)