cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(simplewalker_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_ENABLE_CXX_EXCEPTIONS 1)
pico_sdk_init()

set(simplewalker_interboard_comm "../mainsrc/walkertypes.h" "../communication/messages.h")

add_executable(controller_main
controller_main.cpp
mpu6050_i2c.c
mpu6050.cpp
pico_comm.cpp
${simplewalker_interboard_comm}
)
# pull in common dependencies
target_link_libraries(controller_main pico_stdlib hardware_i2c)
# enable usb output, disable uart output
pico_enable_stdio_usb(controller_main 1)
pico_enable_stdio_uart(controller_main 0)
# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(controller_main)

add_executable(hardware_test
hardwareTest.cpp
motors.cpp
        ADC_reader.cpp
        ${simplewalker_interboard_comm}
)
# pull in common dependencies
target_link_libraries(hardware_test pico_stdlib hardware_pwm hardware_spi)
# enable usb output, disable uart output
pico_enable_stdio_usb(hardware_test 1)
pico_enable_stdio_uart(hardware_test 0)
# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(hardware_test)

add_executable(test_motor
testMotor.cpp
motors.cpp
        ADC_reader.cpp
        ${simplewalker_interboard_comm}
)
# pull in common dependencies
target_link_libraries(test_motor pico_stdlib hardware_pwm hardware_spi)
# enable usb output, disable uart output
pico_enable_stdio_usb(test_motor 1)
pico_enable_stdio_uart(test_motor 0)
# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(test_motor)


