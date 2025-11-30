# **************************** system tests ****************************

add_executable(jacobian_tests ../physics/leg_kinematics.cpp ../physics/rotation.cpp
                              test_utils.cpp jacobian_tests.cpp)
target_link_libraries(jacobian_tests IO)


# test_localization
add_executable(test_localization test_localization.cpp test_utils.cpp)
target_link_libraries(test_localization IO RobotStateLib StateEstimationLib)


# **************************** unittests ****************************
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.17.0
)
FetchContent_MakeAvailable(googletest)
enable_testing()
include(GoogleTest)


add_executable(unittest_leg_kinematics ../physics/leg_kinematics.cpp unittests/unittest_leg_kinematics.cpp)
target_link_libraries(unittest_leg_kinematics GTest::gtest_main)
gtest_discover_tests(unittest_leg_kinematics)

add_executable(unittest_rotation ../physics/rotation.cpp unittests/unittest_rotation.cpp)
target_link_libraries(unittest_rotation GTest::gtest_main)
gtest_discover_tests(unittest_rotation)

add_executable(unittest_basic_communication
        unittests/unittest_basic_communication.cpp ../communication/communication.cpp)
target_include_directories(unittest_basic_communication PUBLIC ../communication)
target_link_libraries(unittest_basic_communication GTest::gtest_main)
gtest_discover_tests(unittest_basic_communication)

add_executable(unittest_state_prediction
        unittests/unittest_state_prediction.cpp ../state_estimation/state_prediction.cpp)
target_link_libraries(unittest_state_prediction RobotStateLib GTest::gtest_main)
gtest_discover_tests(unittest_state_prediction)
