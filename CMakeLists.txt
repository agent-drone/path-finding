cmake_minimum_required(VERSION 3.26)
set(BUILD_SHARED_LIBS OFF)

project(pathfinder)

include_directories(include)
file(GLOB sources "src/*.cpp")

find_package(ompl REQUIRED)
find_package(octomap REQUIRED)
find_package(pybind11 REQUIRED)

pybind11_add_module(pathfinder ${sources})
target_link_libraries(
    pathfinder
    PRIVATE octomap
    PRIVATE ompl::ompl
)
install(TARGETS pathfinder LIBRARY DESTINATION pathfinder)
