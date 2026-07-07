cmake_minimum_required(VERSION 3.10)
project(Touch CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_library(HD_LIB HD REQUIRED)
find_library(HDU_LIB HDU REQUIRED)
find_library(RUT Utilities HINTS ${CMAKE_INSTALL_PREFIX}/lib/RobotUtilities)

# Shared library used by ManipServer
add_library(TOUCH SHARED src/touch.cpp)
target_include_directories(TOUCH PUBLIC
    include
    /usr/include
    ${EIGEN3_INCLUDE_DIRS}
)
target_compile_definitions(TOUCH PUBLIC linux)
target_link_libraries(TOUCH PUBLIC
    ${HD_LIB}
    ${HDU_LIB}
    ${RUT}
    yaml-cpp
)
install(TARGETS TOUCH DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include/)

# Standalone test executables
add_executable(touch_read src/main.cpp)
add_executable(touch_param_control src/main2.cpp)

foreach(target touch_read touch_param_control)
    target_include_directories(${target} PRIVATE
        include
        /usr/include
    )
    target_compile_definitions(${target} PRIVATE linux)
    target_link_libraries(${target} PRIVATE
        ${HD_LIB}
        ${HDU_LIB}
    )
endforeach()
