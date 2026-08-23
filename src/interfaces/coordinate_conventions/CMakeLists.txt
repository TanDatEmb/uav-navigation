cmake_minimum_required(VERSION 3.16)
project(coordinate_conventions LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

install(DIRECTORY include/ DESTINATION include)
ament_export_dependencies(Eigen3)
ament_export_include_directories(include)
ament_package()
