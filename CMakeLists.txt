cmake_minimum_required(VERSION 2.8.3)
project(farmwise_odometry)

add_compile_options(-std=c++17)

set(BOOST_ROOT "/Users/lihang/Downloads/boost_1_84_0")
set(BOOST_INCLUDEDIR "/Users/lihang/Downloads/boost_1_84_0/boost")
set(BOOST_LIBRARYDIR "/Users/lihang/Downloads/boost_1_84_0/stage/lib")

SET(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} "/Users/lihang/Downloads/boost_1_84_0/boost")
SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} "/Users/lihang/Downloads/boost_1_84_0/stage/lib")

FIND_PACKAGE(Boost)
IF (Boost_FOUND)
    INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
    ADD_DEFINITIONS( "-DHAS_BOOST" )
ENDIF()

find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

add_executable(test_drive_straight
  src/odometry_wheels.cpp
  src/test_drive_straight.cpp
)

target_link_libraries(test_drive_straight
  ${Boost_LIBRARIES}
)

