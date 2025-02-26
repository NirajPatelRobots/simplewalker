cmake_minimum_required(VERSION 3.13)
if (NOT DEFINED ENV{PICO_SDK_PATH})
    set(ENV{PICO_SDK_PATH} ~/pico/pico-sdk)
endif()
include("$ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake")
project(simplewalker_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_CXX_ENABLE_EXCEPTIONS 1)
set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_COMMAND} -E time")
pico_sdk_init()

set(pico_motor_IO_src "motor_output.cpp" "ADC_reader.cpp" "motor_IO.cpp")
set(IMU_src "sensors/IMU_errcheck.cpp" "sensors/IMUContext.cpp")


if (DEFINED ENV{MPU6050_SRC_DIR} AND DEFINED ENV{MPU6050_BUILD_DIR})
    add_executable(IMU_main
            IMU_main.cpp "pico_comm.cpp" ${IMU_src}
    )
    target_include_directories(IMU_main PUBLIC $ENV{MPU6050_SRC_DIR} $ENV{MPU6050_BUILD_DIR} $CMAKE_SRC_DIR sensors/*.h)
    target_link_directories(IMU_main PUBLIC $ENV{MPU6050_BUILD_DIR})
    target_link_libraries(IMU_main MPU6050_i2c_pico_cpp_lib pico_stdlib hardware_i2c)
    # enable usb output, disable uart output
    pico_enable_stdio_usb(IMU_main 1)
    pico_enable_stdio_uart(IMU_main 0)
    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(IMU_main)
else()
    message("No MPU6050_SRC_DIR and MPU6050_BUILD_DIR defined")
endif()

add_executable(controller_main
        controller_main.cpp
        motor_control.cpp
        ${pico_motor_IO_src}
        "pico_comm.cpp"
        )
target_link_libraries(controller_main pico_stdlib hardware_pwm hardware_spi)
# enable usb output, disable uart output
pico_enable_stdio_usb(controller_main 1)
pico_enable_stdio_uart(controller_main 0)
# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(controller_main)

add_executable(hardware_test
        hardwareTest.cpp
        ${pico_motor_IO_src}
        "pico_comm.cpp"
)
target_link_libraries(hardware_test pico_stdlib hardware_pwm hardware_spi)
# enable usb output, disable uart output
pico_enable_stdio_usb(hardware_test 1)
pico_enable_stdio_uart(hardware_test 0)
# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(hardware_test)

add_executable(calibrate_motor
        calibrateMotor.cpp
        ${pico_motor_IO_src}
        "pico_comm.cpp"
)
target_link_libraries(calibrate_motor pico_stdlib hardware_pwm hardware_spi)
# enable usb output, disable uart output
pico_enable_stdio_usb(calibrate_motor 1)
pico_enable_stdio_uart(calibrate_motor 0)
# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(calibrate_motor)


