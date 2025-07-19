cmake_minimum_required(VERSION 3.14)
project(MyProject)
find_package(octomap REQUIRED)

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE octomap)
