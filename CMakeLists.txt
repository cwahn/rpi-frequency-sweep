cmake_minimum_required(VERSION 3.0.0)
project(rpi-frequency-sweep VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

find_library(PIGPIO_LIBRARY pigpio)

add_executable(rpi-frequency-sweep main.cpp)
target_link_libraries(rpi-frequency-sweep ${PIGPIO_LIBRARY})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
