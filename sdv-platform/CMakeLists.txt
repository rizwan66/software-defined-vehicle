cmake_minimum_required(VERSION 3.16)
project(sdv_platform CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)
find_package(Threads REQUIRED)

include_directories(${CMAKE_SOURCE_DIR})

set(SRCS_COMMON
    drivers/can/can_interface.cpp
    services/ota/ota_manager.cpp)

add_executable(sdv_demo apps/demo/main.cpp ${SRCS_COMMON})
target_link_libraries(sdv_demo PRIVATE Threads::Threads)

add_executable(sdv_tests tests/test_all.cpp ${SRCS_COMMON})
target_link_libraries(sdv_tests PRIVATE Threads::Threads)

enable_testing()
add_test(NAME unit_tests COMMAND sdv_tests)
