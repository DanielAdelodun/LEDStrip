cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(MAVLink_LEDStrip_Server)

add_executable(LEDStrip_Server
	LEDStrip_Server.cpp
)

find_package(MAVSDK REQUIRED)

target_include_directories(LEDStrip_Server PUBLIC
    /usr/local/include/
    /usr/include/
    /usr/local/include/ws2811/
    /usr/local/include/mavsdk/plugins/mavlink_passthrough/include/
)

target_link_libraries(LEDStrip_Server
    MAVSDK::mavsdk
    ws2811
)

if(NOT MSVC)
	add_compile_options(LEDStrip_Server PRIVATE -Wall -Wextra)
else()
	add_compile_options(LEDStrip_Server PRIVATE -WX -W2)
endif()
