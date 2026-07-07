cmake_minimum_required(VERSION 3.5)
project(teleop_data_collection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

set(CMAKE_AUTOMOC ON)

add_executable(teleop_data_collection src/main.cc src/teleop_gui.h)
target_link_libraries(teleop_data_collection
  ManipServerMultiCam
  TOUCH
  ${RUT}
  ${YAMLLib}
  Qt5::Widgets
)
target_link_options(teleop_data_collection PRIVATE
  -Wl,-rpath-link,/lib/x86_64-linux-gnu
  -Wl,-rpath-link,/usr/lib/x86_64-linux-gnu
)
