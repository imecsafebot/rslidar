cmake_minimum_required(VERSION 3.16)

project(rslidar_people_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt automatically generates MOC files for classes using Q_OBJECT
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(MPI REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

find_package(PCL REQUIRED COMPONENTS
    common
    filters
    segmentation
    search
)

# libpcap
pkg_check_modules(PCAP REQUIRED libpcap)

# RoboSense rs_driver
set(RS_DRIVER_ROOT
    "$ENV{HOME}/Documents/rslidar_sdk/src/rslidar_skd/src/rs_driver/src"
    CACHE PATH "Path to the RoboSense rs_driver source directory"
)

if(NOT EXISTS "${RS_DRIVER_ROOT}/rs_driver/api/lidar_driver.hpp")
    message(FATAL_ERROR
        "rs_driver not found at ${RS_DRIVER_ROOT}\n"
        "Expected: ${RS_DRIVER_ROOT}/rs_driver/api/lidar_driver.hpp"
    )
endif()

message(STATUS "Using rs_driver: ${RS_DRIVER_ROOT}")

include_directories(
    ${RS_DRIVER_ROOT}
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/gui/include
    ${PCAP_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(rslidar_people_demo

    gui/src/main.cpp
    gui/src/mainwindow.cpp
    gui/src/pointcloudwidget.cpp
    gui/src/detectoradapter.cpp

    src/people_detector.cpp
    src/people_tracker.cpp

    gui/include/mainwindow.h
    gui/include/pointcloudwidget.h
    gui/include/detectoradapter.h

    include/people_detector.h
    include/people_tracker.h
    include/people_types.h
)

target_include_directories(rslidar_people_demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/gui/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${RS_DRIVER_ROOT}
    ${PCAP_INCLUDE_DIRS}
)

target_compile_options(rslidar_people_demo PRIVATE
    -O3
    -Wall
    -Wextra
    -Wpedantic
)

target_link_directories(rslidar_people_demo PRIVATE
    ${PCAP_LIBRARY_DIRS}
)

target_link_libraries(rslidar_people_demo PRIVATE
    Qt5::Widgets
    ${PCL_LIBRARIES}
    ${PCAP_LIBRARIES}
    Threads::Threads
)

message(STATUS "RoboSense rs_driver: ${RS_DRIVER_ROOT}")
message(STATUS "PCL libraries: ${PCL_LIBRARIES}")
message(STATUS "PCAP libraries: ${PCAP_LIBRARIES}")

add_executable(sensor_probe
    tools/sensor_probe.cpp gui/src/detectoradapter.cpp
    gui/include/detectoradapter.h src/people_detector.cpp src/people_tracker.cpp)
target_link_libraries(sensor_probe PRIVATE Qt5::Widgets ${PCL_LIBRARIES} ${PCAP_LIBRARIES} Threads::Threads)
