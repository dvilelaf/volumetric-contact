# Standalone project: configure with -DCMAKE_PREFIX_PATH=<volcontact install prefix>.
cmake_minimum_required(VERSION 3.22)
project(volcontact_minimal LANGUAGES CXX)

find_package(volcontact CONFIG REQUIRED)  # Also finds Eigen (and OpenMP if the core uses it).

add_executable(minimal main.cpp)
target_link_libraries(minimal PRIVATE volcontact::core)
