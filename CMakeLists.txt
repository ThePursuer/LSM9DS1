cmake_minimum_required(VERSION 3.10)

project(LSM9DS1Demo)

# Set C++ standard to C++20
set(CMAKE_CXX_STANDARD 20)

# Find required packages
find_package(Threads REQUIRED)

# Set compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# Add the LSM9DS1 driver header and source files to the project
add_library(LSM9DS1 LSM9DS1.hpp LSM9DS1.cpp)

# Add the main application source file to the project
add_executable(main main.cpp)

# Link the main application with the LSM9DS1 library and the pthread library
target_link_libraries(main LSM9DS1 Threads::Threads)