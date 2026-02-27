cmake_minimum_required(VERSION 3.31)

project(441Sim)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenGL REQUIRED)
find_package(glfw3 CONFIG REQUIRED)

find_package(Boost REQUIRED)

add_subdirectory(external/imgui)

add_executable(441Sim main.cpp
        graphics.h
        util.h
        draw.h
        MecanumSystem.h
        MecanumSystem.cpp
        SimRobot.cpp
        SimRobot.h)

target_include_directories(441Sim PRIVATE ${Boost_INCLUDE_DIRS})

target_link_libraries(441Sim imgui glfw OpenGL::GL)
