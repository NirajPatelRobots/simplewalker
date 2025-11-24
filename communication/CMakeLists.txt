add_library(InterboardComm)
target_sources(InterboardComm
        PRIVATE
            "communication.cpp" "comm_serial.cpp"
        PUBLIC
            FILE_SET HEADERS
            BASE_DIRS .
            FILES communication.hpp comm_serial.hpp
        )
target_link_libraries(InterboardComm wiringPi)

add_library(ExternalComm)
target_sources(ExternalComm
        PRIVATE
            "communication.cpp" "comm_tcp.cpp"
        PUBLIC
            FILE_SET HEADERS
            BASE_DIRS .
            FILES communication.hpp comm_tcp.hpp
        )
