cmake_minimum_required(VERSION 3.16)
project(rslidar_people_viewer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL REQUIRED COMPONENTS common io visualization)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(rslidar_people_viewer
    src/people_viewer.cpp
)

target_link_libraries(rslidar_people_viewer
    ${PCL_LIBRARIES}
)
