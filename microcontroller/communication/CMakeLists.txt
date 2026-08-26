add_library(pico_comm)
target_sources(pico_comm
        PRIVATE
            "pico_comm.cpp" "../../communication/communication.cpp"
        PUBLIC
            FILE_SET HEADERS
            BASE_DIRS . ../../communication
            FILES "pico_comm.hpp" "../../communication/communication.hpp"
        )
target_link_libraries(pico_comm PUBLIC pico_stdlib)
