add_library(MotorControl)
target_sources(MotorControl
        PRIVATE "motor_control.cpp"
        PUBLIC
            FILE_SET HEADERS
            BASE_DIRS .
            FILES "motor_control.hpp"
        )
target_link_libraries(MotorControl PUBLIC pico_stdlib)
