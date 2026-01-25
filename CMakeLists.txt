cmake_minimum_required(VERSION 3.23)
project(simplewalker)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O")
set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_COMMAND} -E time")
include(FetchContent)

# TODO: sort out which directory to run from / find settings file
# TODO better dependency includes
include_directories(../eigen-3.4.0)
include_directories(../rapidxml)
include_directories(/usr/include/rapidxml)
include_directories(/usr/include/eigen3)

include_directories(mainsrc display physics)
add_subdirectory(communication)
add_subdirectory(state_estimation)
add_subdirectory(display)


# simplewalker main
add_executable(simplewalker mainsrc/simplewalker.cpp)
target_link_libraries(simplewalker IO InterboardComm ExternalComm StateEstimationLib)

# collect_sensor_cal_data
add_executable(collect_sensor_cal_data sensors/collect_sensor_cal_data.cpp)
target_link_libraries(collect_sensor_cal_data wiringPi stdc++fs IO InterboardComm)

# measure motor angle manually
add_executable(measure_motor_angle_manually calibrate/measure_motor_angle_manually.cpp)
target_link_libraries(measure_motor_angle_manually InterboardComm)


add_subdirectory(test)
