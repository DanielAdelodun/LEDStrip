cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(LEDString_Server)

add_executable(ControlLights
    ControlLights.cpp
)

find_package(MAVSDK REQUIRED)

target_include_directories(Lights PUBLIC
    /usr/local/include/
    /usr/include/
    /usr/local/include/ws2811/
    /usr/local/include/mavsdk/plugins/mavlink_passthrough/include/
)

target_link_libraries(ControlLights
    MAVSDK::mavsdk
    ws2811
)

if(NOT MSVC)
    add_compile_options(ControlLights PRIVATE -Wall -Wextra)
else()
    add_compile_options(ControlLights PRIVATE -WX -W2)
endif()
