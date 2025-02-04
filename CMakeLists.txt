cmake_minimum_required(VERSION 3.5.0)
project(franka-utils VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Curses REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Franka REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

include_directories(include)

link_libraries(Eigen3::Eigen Franka::Franka Threads::Threads)

add_library(examples_common src/examples_common.cpp)

add_executable(recorder src/recorder_new.cpp)
