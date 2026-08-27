# microcontroller/control is built in the microcontroller project as well as here
add_subdirectory(../../microcontroller/control control/)

add_executable(unittest_motor_model unittest_motor_model.cpp)
target_link_libraries(unittest_motor_model GTest::gtest_main MotorModel)
gtest_discover_tests(unittest_motor_model)
