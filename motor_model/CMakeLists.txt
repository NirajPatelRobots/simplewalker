add_library(MotorModelUser)
target_sources(MotorModelUser PRIVATE "motor_model_API_user.cpp")
target_link_libraries(MotorModelUser MotorModel)


add_library(MotorModelFileIO)
target_sources(MotorModelFileIO
        PRIVATE "motor_model_file_IO.cpp"
        PUBLIC
            FILE_SET HEADERS
            BASE_DIRS ./ ${PROJECT_SOURCE_DIR}/microcontroller/control
            FILES "motor_model_file_IO.hpp" "${PROJECT_SOURCE_DIR}/microcontroller/control/motor_model_API.hpp"
        )
target_link_libraries(MotorModelFileIO nlohmann_json::nlohmann_json)
