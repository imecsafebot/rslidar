cmake_minimum_required(VERSION 3.16)

project(rslidar_people_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL REQUIRED COMPONENTS
    common
    io
    filters
    segmentation
    search
)

include_directories(
    ${PCL_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
)

add_definitions(
    ${PCL_DEFINITIONS}
)

add_executable(
    people_detector
    src/people_detector.cpp
)

target_link_libraries(
    people_detector
    ${PCL_LIBRARIES}
)
