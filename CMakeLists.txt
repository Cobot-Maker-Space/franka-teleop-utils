cmake_minimum_required(VERSION 3.5.0)
project(franka-utils VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(Franka_DIR "/home/johnkiran/libfranka/build")

find_package(Eigen3 REQUIRED)
find_package(Franka REQUIRED)
find_package(CapnProto REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

include_directories(include ${CAPNP_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

add_definitions(${CAPNP_DEFINITIONS})

capnp_generate_cpp(CAPNP_SRCS CAPNP_HDRS messages/robot-state.capnp)

add_library(examples_common src/examples_common.cpp)
target_link_libraries(
  examples_common
  Eigen3::Eigen
)

add_executable(follower src/follower.cpp ${CAPNP_SRCS} ${CAPNP_HDRS})
target_link_libraries(
  follower
  ${CAPNP_LIBRARIES}
  examples_common
  Franka::Franka
  ${CAPNP_LIBRARIES}
  Threads::Threads)

add_executable(
  leader
  src/leader.cpp
  src/ioutils.cpp
  ${CAPNP_SRCS}
  ${CAPNP_HDRS})
target_link_libraries(
  leader
  ${CAPNP_LIBRARIES}
  examples_common
  Franka::Franka
  ${CAPNP_LIBRARIES}
  Threads::Threads)

add_executable(player src/player.cpp ${CAPNP_SRCS} ${CAPNP_HDRS})
target_link_libraries(
  player
  ${CAPNP_LIBRARIES}
  examples_common
  Franka::Franka
  ${CAPNP_LIBRARIES}
  Threads::Threads)

add_executable(recorder src/recorder.cpp ${CAPNP_SRCS} ${CAPNP_HDRS})
target_link_libraries(
  recorder
  ${CAPNP_LIBRARIES}
  examples_common
  Franka::Franka
  ${CAPNP_LIBRARIES}
  Threads::Threads)

add_executable(recording2csv src/recording2csv.cpp ${CAPNP_SRCS} ${CAPNP_HDRS})
target_link_libraries(recording2csv ${CAPNP_LIBRARIES})
