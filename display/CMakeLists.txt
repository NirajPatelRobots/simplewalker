add_library(IO)
target_sources(IO
        PRIVATE
            "logger.cpp" "settings.cpp"
        PUBLIC
            FILE_SET HEADERS
            BASE_DIRS .
            FILES "logger.hpp" "settings.hpp"
        )
