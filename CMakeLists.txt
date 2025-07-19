cmake_minimum_required(VERSION 3.14)
project(ALEOS)
find_package(octomap REQUIRED)

include_directories(include)
file(GLOB sources "src/*.cpp")

add_executable(pathfinder main.cpp ${sources})
target_link_libraries(pathfinder PRIVATE octomap)
