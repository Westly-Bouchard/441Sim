cmake_minimum_required(VERSION 3.31)

project(441Sim)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenGL REQUIRED)
find_package(glfw3 CONFIG REQUIRED)

add_subdirectory(external/imgui)

add_executable(441Sim main.cpp)

target_link_libraries(441Sim imgui glfw OpenGL::GL)
