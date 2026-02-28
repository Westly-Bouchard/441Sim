cmake_minimum_required(VERSION 3.31)

project(441Sim)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenGL REQUIRED)
find_package(glfw3 CONFIG REQUIRED)

find_package(Boost REQUIRED)

add_subdirectory(external)

add_executable(441Sim main.cpp
        graphics.h
        util.h
        draw.h
        sim/MecanumSystem.h
        sim/MecanumSystem.cpp
        sim/SimRobot.cpp
        sim/SimRobot.h
        sim/SimMotor.cpp
        sim/SimMotor.h
        user/arduino/arduinoMain.cpp
        user/wrappers/arduinoLib.cpp
        user/wrappers/MotorWrapper.cpp
)

target_include_directories(441Sim PRIVATE ${Boost_INCLUDE_DIRS})

target_link_libraries(441Sim imgui glfw OpenGL::GL)

target_compile_definitions(441Sim PUBLIC "SIM")
