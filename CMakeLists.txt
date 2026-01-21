cmake_minimum_required(VERSION 3.10)
project(ThrGenCpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

# Main executable (Server)
add_executable(ThrGenCpp src/main.cpp src/EdgeDetector.cpp src/PathPlanner.cpp src/ThrGenerator.cpp src/GifGenerator.cpp src/stb_impl.cpp)

# CLI Tool
add_executable(ThrGenCLI src/cli.cpp src/EdgeDetector.cpp src/PathPlanner.cpp src/ThrGenerator.cpp src/GifGenerator.cpp src/stb_impl.cpp)
target_link_libraries(ThrGenCLI PRIVATE Threads::Threads)

# Link threads (needed for httplib)
find_package(Threads REQUIRED)
target_link_libraries(ThrGenCpp PRIVATE Threads::Threads)

# Optimization flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(ThrGenCpp PRIVATE -O3 -march=native -Wall -Wextra)
endif()

# Tests
add_executable(ThrGenCpp_Tests 
    tests/main.cpp 
    tests/test_EdgeDetector.cpp 
    tests/test_PathPlanner.cpp 
    tests/test_ThrGenerator.cpp
    tests/test_Integration.cpp
    src/EdgeDetector.cpp
    src/PathPlanner.cpp
    src/ThrGenerator.cpp
    src/GifGenerator.cpp
    src/stb_impl.cpp
)
target_link_libraries(ThrGenCpp_Tests PRIVATE Threads::Threads)
