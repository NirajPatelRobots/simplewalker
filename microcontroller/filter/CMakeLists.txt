# This directory is included in both CMake projects of this repo.
# 1. The microcontroller project is built for 32-bit ARM with the pico sdk.
# 2. The parent project builds tests for this directory on the host machine.
# The pico sdk doesn't have file IO, not even pipes, so tests need 
#   to be built with the host machine's os functions. That means this directory can
#   only use minimal dependencies.
# Currently uses only stdlib array

add_library(MicroFilter INTERFACE)
target_sources(MicroFilter
        PUBLIC
            FILE_SET HEADERS
            BASE_DIRS .
            FILES "micro_filter.hpp"
        )
