cmake_minimum_required(VERSION 3.5.0)
project(franka-teleop-utils VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(Franka_DIR "" CACHE FILEPATH "LibFranka install path")

option(REPORT_RATE "Report sending/receiving rates" OFF)

find_package(CapnProto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Franka REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(yaml-cpp)

include_directories(
  include
  ${CAPNP_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR})

add_definitions(${CAPNP_DEFINITIONS})

if(REPORT_RATE)
  add_definitions(-DREPORT_RATE)
endif()

capnp_generate_cpp(CAPNP_SRCS CAPNP_HDRS messages/robot-state.capnp)

add_library(
  common
  src/memory_outputstream.cpp
  src/motion_generator.cpp
  src/teleop_utils.cpp)
target_link_libraries(
  common
  Eigen3::Eigen
)

add_executable(
  receiver
  src/receiver.cpp
  ${CAPNP_SRCS}
  ${CAPNP_HDRS})
target_link_libraries(
  receiver
  ${CAPNP_LIBRARIES}
  common
  Franka::Franka
  ${CAPNP_LIBRARIES}
  Threads::Threads
  ${YAML_CPP_LIBRARIES})

add_executable(
  sender
  src/sender.cpp
  ${CAPNP_SRCS}
  ${CAPNP_HDRS})
target_link_libraries(
  sender
  ${CAPNP_LIBRARIES}
  common
  Franka::Franka
  ${CAPNP_LIBRARIES}
  Threads::Threads
  ${YAML_CPP_LIBRARIES})
