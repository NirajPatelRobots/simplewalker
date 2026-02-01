add_library(BasicCommunication)
target_sources(BasicCommunication
        PRIVATE
            "communication.cpp"
        PUBLIC
            FILE_SET HEADERS
            BASE_DIRS .
            FILES communication.hpp
        )
add_library(InterboardComm)
target_sources(InterboardComm
        PRIVATE
            "comm_serial.cpp"
        PUBLIC
            FILE_SET HEADERS
            BASE_DIRS .
            FILES comm_serial.hpp
        )
target_link_libraries(InterboardComm BasicCommunication wiringPi)

add_library(ExternalComm)
target_sources(ExternalComm
        PRIVATE
            "comm_tcp.cpp"
        PUBLIC
            FILE_SET HEADERS
            BASE_DIRS .
            FILES comm_tcp.hpp
        )
target_link_libraries(ExternalComm BasicCommunication pthread)
