cmake_minimum_required(VERSION 3.10)
project(Leptrino C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.4 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(LEPTRINO_SOURCES
    src/Leptrino_driver.cpp
    src/SerialCommunication.cpp
    src/Leptrino_ft.cpp
)

add_library(LEPTRINO SHARED ${LEPTRINO_SOURCES})

target_include_directories(LEPTRINO PUBLIC
    include
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(LEPTRINO PUBLIC
    ${RUT}
    pthread
    yaml-cpp
)

add_executable(ft_read src/Test.cpp)
target_link_libraries(ft_read PRIVATE LEPTRINO)

install(TARGETS LEPTRINO DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)