cmake_minimum_required(VERSION 3.5)
#######################
project(manipulation_data_collection)

set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include/
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(manipulation_data_collection src/main.cc)
target_link_libraries(manipulation_data_collection
  ManipServer
  ${RUT}
  ${YAMLLib}
)
# System lib path must come before conda's for libgdal->libcurl resolution
target_link_options(manipulation_data_collection PRIVATE
  -Wl,-rpath-link,/lib/x86_64-linux-gnu
  -Wl,-rpath-link,/usr/lib/x86_64-linux-gnu
)
