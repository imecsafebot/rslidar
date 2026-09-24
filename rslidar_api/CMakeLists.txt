cmake_minimum_required(VERSION 3.10)

project(rslidar_api)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(rslidar_api
    main.cpp
)

target_include_directories(rslidar_api PRIVATE
    /home/neda/Documents/rslidar_sdk/src/rslidar_skd/src/rs_driver/src
)

target_link_libraries(rslidar_api
    pthread
    pcap
)
