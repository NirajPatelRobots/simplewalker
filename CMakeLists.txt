cmake_minimum_required(VERSION 3.16)
project(simplewalker)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O")

set(src_names state_estimation/state_estimation state_estimation/robot_state
 sensors/sensorBoss display/logger physics/leg_kinematics mainsrc/walkerUtils)
set(src_dirs mainsrc communication display physics sensors state_estimation test)


set(src_comm "communication/maincomp_comm.cpp")
set(src_files "")
foreach(src_name ${src_names})
    set(src_files ${src_files} ${src_name}.cpp)
endforeach()

message("Source files ${src_files}")
message("Source dirs ${src_dirs}")

# TODO: compile shared objects together
# TODO: don't redo object building

# simplewalker main
include_directories(${src_dirs})
include_directories(../eigen-3.4.0)
include_directories(../rapidxml)
include_directories(/usr/include/rapidxml)
include_directories(/usr/include/eigen3)
add_executable(simplewalker ${src_files} ${src_comm} mainsrc/simplewalker.cpp)
target_link_libraries(simplewalker wiringPi)

# collect_sensor_cal_data
include_directories(${src_dirs})
add_executable(collect_sensor_cal_data ${src_files} ${src_comm} sensors/collect_sensor_cal_data.cpp)
target_link_libraries(collect_sensor_cal_data wiringPi)


# test_localization
include_directories(${src_dirs})
add_executable(test_localization ${src_files} state_estimation/test_localization.cpp)


# unittests
add_subdirectory(/usr/src/googletest build)
enable_testing()
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})


add_executable(unittests ${src_files} test/unittests.cpp test/test_utils.cpp) # TODO use googletest
#TODO split unitttests into multiple files

add_executable(unittest_basic_communication test/unittest_basic_communication.cpp)
target_link_libraries(unittest_basic_communication gtest gtest_main gmock)

include(GoogleTest)
gtest_discover_tests(unittest_basic_communication)
