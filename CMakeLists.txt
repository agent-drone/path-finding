cmake_minimum_required(VERSION 3.28)
project(pathfinder)

find_package(ompl REQUIRED)
find_package(octomap REQUIRED)
find_package(pybind11 REQUIRED)

include_directories(include)
file(GLOB sources "src/*.cpp")

pybind11_add_module(pathfinder ${sources})
target_link_libraries(
    pathfinder
    PRIVATE octomap
    PRIVATE ompl::ompl
)
install(TARGETS pathfinder LIBRARY DESTINATION pathfinder)
