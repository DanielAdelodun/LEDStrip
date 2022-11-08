cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(MAVLink_LEDStrip_Client C CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
endif()

set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DVK_PROTOTYPES")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DVK_PROTOTYPES")

# GLFW
set(GLFW_DIR third-party/glfw) # Set this to point to an up-to-date GLFW repo
option(GLFW_BUILD_EXAMPLES "Build the GLFW example programs" OFF)
option(GLFW_BUILD_TESTS "Build the GLFW test programs" OFF)
option(GLFW_BUILD_DOCS "Build the GLFW documentation" OFF)
option(GLFW_INSTALL "Generate installation target" OFF)
option(GLFW_DOCUMENT_INTERNALS "Include internals in documentation" OFF)
add_subdirectory(${GLFW_DIR} binary_dir EXCLUDE_FROM_ALL)
include_directories(${GLFW_DIR}/include)

# Dear ImGui
set(IMGUI_DIR third-party/imgui)
include_directories(${IMGUI_DIR} ${IMGUI_DIR}/backends ..)

# Libraries
find_package(Vulkan REQUIRED)
find_package(MAVSDK REQUIRED)
set(LIBRARIES "glfw;Vulkan::Vulkan;MAVSDK::mavsdk")

# Use vulkan headers from glfw:
include_directories(${GLFW_DIR}/deps)

file(GLOB sources src/*.cpp)

add_executable(LEDStrip_Client ${sources} ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp ${IMGUI_DIR}/backends/imgui_impl_vulkan.cpp ${IMGUI_DIR}/imgui.cpp ${IMGUI_DIR}/imgui_draw.cpp ${IMGUI_DIR}/imgui_demo.cpp ${IMGUI_DIR}/imgui_tables.cpp ${IMGUI_DIR}/imgui_widgets.cpp)
target_include_directories(LEDStrip_Client PUBLIC
    "/usr/local/include/mavsdk/plugins/mavlink_passthrough/include/"
    "/usr/local/include/"
    "/usr/include/"
    "include"
)
target_link_libraries(LEDStrip_Client ${LIBRARIES})
target_compile_definitions(LEDStrip_Client PUBLIC -DImTextureID=ImU64)

if(NOT MSVC)
    add_compile_options(LEDStrip_Client PRIVATE -Wall -Wextra)
else()
    add_compile_options(LEDStrip_Client PRIVATE -WX -W2)
endif()
