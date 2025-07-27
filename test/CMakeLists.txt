# **************************** manual tests ****************************
add_executable(jacobian_tests ../physics/leg_kinematics.cpp ../mainsrc/walkerUtils.cpp
        test_utils.cpp ../display/logger.cpp jacobian_tests.cpp)


# **************************** unittests ****************************
add_subdirectory(/usr/src/googletest build)
enable_testing()
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
include(GoogleTest)


add_executable(unittest_leg_kinematics ../physics/leg_kinematics.cpp unittests/unittest_leg_kinematics.cpp)
target_link_libraries(unittest_leg_kinematics gtest gtest_main)
gtest_discover_tests(unittest_leg_kinematics)

add_executable(unittest_walkerUtils ../mainsrc/walkerUtils.cpp unittests/unittest_walkerUtils.cpp)
target_link_libraries(unittest_walkerUtils gtest gtest_main)
gtest_discover_tests(unittest_walkerUtils)

add_executable(unittest_basic_communication
        unittests/unittest_basic_communication.cpp ../communication/communication.cpp)
target_link_libraries(unittest_basic_communication gtest gtest_main gmock)
gtest_discover_tests(unittest_basic_communication)

add_executable(unittest_state_prediction
        unittests/unittest_state_prediction.cpp ../state_estimation/state_prediction.cpp ../state_estimation/robot_state.cpp)
target_link_libraries(unittest_state_prediction gtest gtest_main)
gtest_discover_tests(unittest_state_prediction)
