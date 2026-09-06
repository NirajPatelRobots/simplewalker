# microcontroller/control is built in the microcontroller project as well as here
add_subdirectory(../../microcontroller/control control/)

add_executable(unittest_motor_model unittest_motor_model.cpp)
target_link_libraries(unittest_motor_model GTest::gtest_main MotorModel)
gtest_discover_tests(unittest_motor_model)

add_executable(unittest_model_functions unittest_model_functions.cpp)
target_link_libraries(unittest_model_functions GTest::gtest_main MotorModel)
gtest_discover_tests(unittest_model_functions)

add_executable(test_motor_model_api test_motor_model_API.cpp)
target_link_libraries(test_motor_model_api GTest::gtest_main MotorModelMicrocontroller MotorModelUser)
gtest_discover_tests(test_motor_model_api)
