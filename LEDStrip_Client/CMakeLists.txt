cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(LEDString_Client)

add_executable(SendLights
    SendLights.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(SendLights
    MAVSDK::mavsdk
)

target_include_directories(SendLights PUBLIC
    /usr/local/include/mavsdk/plugins/mavlink_passthrough/include/
    /usr/local/include/
    /usr/include/
    
)

if(NOT MSVC)
    add_compile_options(SendLights PRIVATE -Wall -Wextra)
else()
    add_compile_options(SendLights PRIVATE -WX -W2)
endif()
