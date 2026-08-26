add_library(pico_motor_IO)
target_sources(pico_motor_IO
        PRIVATE
            "motor_output.cpp" "../sensors/ADC_reader.cpp" "motor_IO.cpp"
        PUBLIC
            FILE_SET HEADERS
            BASE_DIRS . ../sensors
            FILES "motor_output.hpp" "../sensors/ADC_reader.hpp" "motor_IO.hpp"
        )
target_link_libraries(pico_motor_IO PUBLIC pico_stdlib hardware_pwm hardware_spi)
