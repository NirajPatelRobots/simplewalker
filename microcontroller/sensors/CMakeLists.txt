


if (DEFINED ENV{MPU6050_SRC_DIR} AND DEFINED ENV{MPU6050_BUILD_DIR})
    SET(HAS_IMU ON PARENT_SCOPE)
    message("MPU6050 src: $ENV{MPU6050_SRC_DIR} build: $ENV{MPU6050_BUILD_DIR}")
    add_library(IMULib)
    target_sources(IMULib
            PRIVATE
                "IMU_errcheck.cpp" "IMUContext.cpp"
            PUBLIC
                FILE_SET HEADERS
                BASE_DIRS .
                FILES "IMU_errcheck.h" "IMUContext.h"
            )
    target_link_libraries(IMULib pico_stdlib hardware_i2c)
    target_include_directories(IMULib PUBLIC $ENV{MPU6050_SRC_DIR} $ENV{MPU6050_BUILD_DIR})
    target_link_directories(IMULib PUBLIC $ENV{MPU6050_BUILD_DIR})
else()
    SET(HAS_IMU OFF PARENT_SCOPE)
    message("No MPU6050_SRC_DIR and MPU6050_BUILD_DIR defined")
endif()
