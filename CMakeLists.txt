cmake_minimum_required(VERSION 3.10)
project(Assignment1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(GLOB SOURCES "src/*.cpp")

add_executable(Assignment1 ${SOURCES})

target_include_directories(Assignment1 PRIVATE ${CMAKE_SOURCE_DIR}/include)
