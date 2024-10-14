cmake_minimum_required(VERSION 3.16)
project(simplewalker)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O")

set(src_names state_estimation/state_estimation state_estimation/robot_state
 sensors/sensorBoss display/logger physics/leg_kinematics mainsrc/walkerUtils)
set(src_dirs mainsrc communication display physics sensors state_estimation test)


set(src_comm "communication/comm_serial.cpp")
set(src_comm_ext "communication/comm_tcp.cpp")
set(src_files "")
foreach(src_name ${src_names})
    set(src_files ${src_files} ${src_name}.cpp)
endforeach()

message("Source files ${src_files}")
message("Source dirs ${src_dirs}")

# TODO: compile shared objects together
# TODO: don't redo object building
# TODO: sort out which directory to run from / find settings file

# simplewalker main
include_directories(${src_dirs})
include_directories(../eigen-3.4.0)
include_directories(../rapidxml)
include_directories(/usr/include/rapidxml)
include_directories(/usr/include/eigen3)
add_executable(simplewalker ${src_files} ${src_comm} ${src_comm_ext} mainsrc/simplewalker.cpp)
target_link_libraries(simplewalker wiringPi)

# collect_sensor_cal_data
include_directories(${src_dirs})
add_executable(collect_sensor_cal_data ${src_files} ${src_comm} sensors/collect_sensor_cal_data.cpp)
target_link_libraries(collect_sensor_cal_data wiringPi stdc++fs)


# measure motor angle manually
add_executable(measure_motor_angle_manually ${src_comm} calibrate/measure_motor_angle_manually.cpp)
target_link_libraries(measure_motor_angle_manually wiringPi)



# test_localization
include_directories(${src_dirs})
add_executable(test_localization ${src_files} state_estimation/test_localization.cpp)

# **************************** manual tests ****************************
add_executable(jacobian_tests physics/leg_kinematics.cpp mainsrc/walkerUtils.cpp
                              test/test_utils.cpp display/logger.cpp test/jacobian_tests.cpp)


# **************************** unittests ****************************
add_subdirectory(/usr/src/googletest build)
enable_testing()
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
include(GoogleTest)


add_executable(unittest_leg_kinematics physics/leg_kinematics.cpp test/unittests/unittest_leg_kinematics.cpp)
target_link_libraries(unittest_leg_kinematics gtest gtest_main)
gtest_discover_tests(unittest_leg_kinematics)

add_executable(unittest_walkerUtils mainsrc/walkerUtils.cpp test/unittests/unittest_walkerUtils.cpp)
target_link_libraries(unittest_walkerUtils gtest gtest_main)
gtest_discover_tests(unittest_walkerUtils)

add_executable(unittest_basic_communication test/unittests/unittest_basic_communication.cpp)
target_link_libraries(unittest_basic_communication gtest gtest_main gmock)
gtest_discover_tests(unittest_basic_communication)
