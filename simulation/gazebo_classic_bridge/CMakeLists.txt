cmake_minimum_required(VERSION 3.10)
project(cbf_rover_gazebo_classic_bridge LANGUAGES CXX)

find_package(gazebo REQUIRED)

add_executable(
  cbf_rover_gazebo_classic_bridge
  src/gazebo_classic_bridge.cc
)

add_library(
  cbf_rover_gazebo_state_plugin
  SHARED
  src/state_plugin.cc
)

add_library(
  cbf_rover_gazebo_camera_plugin
  SHARED
  src/camera_plugin.cc
)

target_include_directories(
  cbf_rover_gazebo_classic_bridge
  PRIVATE
    ${GAZEBO_INCLUDE_DIRS}
)

target_include_directories(
  cbf_rover_gazebo_state_plugin
  PRIVATE
    ${GAZEBO_INCLUDE_DIRS}
)

target_include_directories(
  cbf_rover_gazebo_camera_plugin
  PRIVATE
    ${GAZEBO_INCLUDE_DIRS}
)

target_link_directories(
  cbf_rover_gazebo_classic_bridge
  PRIVATE
    ${GAZEBO_LIBRARY_DIRS}
)

target_link_directories(
  cbf_rover_gazebo_state_plugin
  PRIVATE
    ${GAZEBO_LIBRARY_DIRS}
)

target_link_directories(
  cbf_rover_gazebo_camera_plugin
  PRIVATE
    ${GAZEBO_LIBRARY_DIRS}
)

target_link_libraries(
  cbf_rover_gazebo_classic_bridge
  PRIVATE
    ${GAZEBO_LIBRARIES}
)

target_link_libraries(
  cbf_rover_gazebo_state_plugin
  PRIVATE
    ${GAZEBO_LIBRARIES}
)

target_link_libraries(
  cbf_rover_gazebo_camera_plugin
  PRIVATE
    ${GAZEBO_LIBRARIES}
)

target_compile_features(
  cbf_rover_gazebo_classic_bridge
  PRIVATE
    cxx_std_17
)

target_compile_features(
  cbf_rover_gazebo_state_plugin
  PRIVATE
    cxx_std_17
)

target_compile_features(
  cbf_rover_gazebo_camera_plugin
  PRIVATE
    cxx_std_17
)

target_compile_options(
  cbf_rover_gazebo_classic_bridge
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

target_compile_options(
  cbf_rover_gazebo_state_plugin
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

target_compile_options(
  cbf_rover_gazebo_camera_plugin
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)
