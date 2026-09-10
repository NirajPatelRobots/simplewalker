add_library(motor_calibration)
target_sources(motor_calibration
        PRIVATE
            "signal_generator.cpp" "motor_calibrator.cpp"
        PUBLIC
            FILE_SET HEADERS
            BASE_DIRS .
            FILES "signal_generator.hpp" "motor_calibrator.hpp"
        )
target_link_libraries(motor_calibration PUBLIC pico_motor_IO pico_comm MotorModelMicrocontroller)
