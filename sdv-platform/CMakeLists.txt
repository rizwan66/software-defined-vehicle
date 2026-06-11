cmake_minimum_required(VERSION 3.20)
project(sdv_platform CXX)

# ── C++23 required for: std::expected, std::flat_map, std::mdspan ────────────
# Wave 1 features (C++20): std::jthread, std::span, concepts, __VA_OPT__
# Wave 2 features (C++23): std::expected, std::flat_map, std::print (if avail.)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)   # enforce -std=c++23 not -std=gnu++23

# ── Compiler warnings ─────────────────────────────────────────────────────────
add_compile_options(
    -Wall -Wextra -Wpedantic
    -Wno-unused-parameter          # common in virtual overrides
)

# ── Optional: sanitizers in debug builds ─────────────────────────────────────
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()

find_package(Threads REQUIRED)
include_directories(${CMAKE_SOURCE_DIR})

# ── Shared compilation units ──────────────────────────────────────────────────
set(SRCS_COMMON
    drivers/can/can_interface.cpp
    services/ota/ota_manager.cpp)

# ── Original demo ─────────────────────────────────────────────────────────────
add_executable(sdv_demo apps/demo/main.cpp ${SRCS_COMMON})
target_link_libraries(sdv_demo PRIVATE Threads::Threads)

# ── Wave 1 demo (C++20) ───────────────────────────────────────────────────────
add_executable(sdv_demo_enhanced apps/demo/main_enhanced.cpp ${SRCS_COMMON})
target_link_libraries(sdv_demo_enhanced PRIVATE Threads::Threads)

# ── Wave 2 demo (C++23 + new subsystems) ─────────────────────────────────────
add_executable(sdv_demo_v2 apps/demo/main_v2.cpp ${SRCS_COMMON})
target_link_libraries(sdv_demo_v2 PRIVATE Threads::Threads)

# ── Test suite ────────────────────────────────────────────────────────────────
add_executable(sdv_tests tests/test_all.cpp ${SRCS_COMMON})
target_link_libraries(sdv_tests PRIVATE Threads::Threads)

enable_testing()
add_test(NAME unit_tests COMMAND sdv_tests)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS sdv_demo sdv_demo_enhanced sdv_demo_v2 sdv_tests
        RUNTIME DESTINATION bin)
