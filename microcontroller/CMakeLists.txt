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

function(controller_exe_common exe_name)
    # enable usb output, disable uart output
    pico_enable_stdio_usb(${exe_name} 1)
    pico_enable_stdio_uart(${exe_name} 0)
    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(${exe_name})
endfunction()

add_subdirectory(sensors)
add_subdirectory(communication)
add_subdirectory(control)
add_subdirectory(motors)  # requires sensors

add_library(ControllerInfo)
target_sources(ControllerInfo
        PRIVATE "controller_info.cpp"
        PUBLIC
            FILE_SET HEADERS
            BASE_DIRS .
            FILES "controller_info.hpp"
        )
target_link_libraries(ControllerInfo PUBLIC hardware_adc)

if (HAS_IMU)
    add_executable(IMU_main main/IMU_main.cpp)
    target_link_libraries(IMU_main
             IMULib ControllerInfo MPU6050_i2c_pico_cpp_lib pico_comm)
    controller_exe_common(IMU_main)
endif()

add_executable(controller_main main/controller_main.cpp)
target_link_libraries(controller_main MotorControl pico_motor_IO ControllerInfo pico_comm)
controller_exe_common(controller_main)

add_executable(hardware_test main/hardwareTest.cpp)
target_link_libraries(hardware_test pico_motor_IO pico_comm)
controller_exe_common(hardware_test)

add_executable(calibrate_motor calibrate/calibrateMotor.cpp calibrate/signal_generator.cpp)
target_link_libraries(calibrate_motor pico_motor_IO pico_comm)
controller_exe_common(calibrate_motor)

