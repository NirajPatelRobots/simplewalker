cmake_minimum_required(VERSION 3.23)
project(simplewalker)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O")
set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_COMMAND} -E time")
cmake_policy(SET CMP0135 NEW)
include(FetchContent)

# TODO: sort out which directory to run from / find settings file
# TODO better dependency includes
include_directories(../eigen-3.4.0)
include_directories(../rapidxml)
include_directories(/usr/include/rapidxml)
include_directories(/usr/include/eigen3)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz)
FetchContent_MakeAvailable(json)

include_directories(mainsrc display physics)
add_subdirectory(communication)
add_subdirectory(state_estimation)
add_subdirectory(display)
add_subdirectory(motor_model)


# simplewalker main
add_executable(simplewalker mainsrc/simplewalker.cpp)
target_link_libraries(simplewalker IO InterboardComm ExternalComm StateEstimationLib)

# monitor only
add_executable(monitor_only mainsrc/monitor_only.cpp)
target_link_libraries(monitor_only IO InterboardComm ExternalComm)

# collect_sensor_cal_data
add_executable(collect_sensor_cal_data sensors/collect_sensor_cal_data.cpp)
target_link_libraries(collect_sensor_cal_data wiringPi stdc++fs IO InterboardComm)

# measure motor angle manually
add_executable(monitor_controller mainsrc/monitor_controller.cpp)
target_link_libraries(monitor_controller IO InterboardComm)

# motor model user API
add_executable(motor_model_user_API mainsrc/motor_model_user_main.cpp)
target_link_libraries(motor_model_user_API MotorModelUser MotorModelFileIO InterboardComm)


add_subdirectory(test)
