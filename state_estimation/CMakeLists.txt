add_library(RobotStateLib)
target_sources(RobotStateLib
        PRIVATE
            "robot_state.cpp" "../physics/leg_kinematics.cpp" "../physics/rotation.cpp"
        PUBLIC
            FILE_SET HEADERS
            BASE_DIRS . ../physics
            FILES "robot_state.hpp" "../physics/leg_kinematics.hpp" "../physics/rotation.hpp"
        )

add_library(StateEstimationLib)
target_sources(StateEstimationLib
        PRIVATE
            state_estimation.cpp ../sensors/sensorBoss.cpp
        PUBLIC
            FILE_SET HEADERS
            BASE_DIRS . ../sensors
            FILES state_estimation.hpp ../sensors/sensorBoss.hpp ../sensors/sensor_data.h
        )
target_link_libraries(StateEstimationLib RobotStateLib)
