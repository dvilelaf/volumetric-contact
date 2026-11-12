cmake_minimum_required(VERSION 3.22)
project(volcontact VERSION 0.1.0 LANGUAGES CXX)

# Top level: the benchmark (scenarios, engine backends, apps) and the tests are built.
# Consumed through add_subdirectory/FetchContent: only the core, unless asked.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(VOLCONTACT_IS_TOP_LEVEL ON)
else()
  set(VOLCONTACT_IS_TOP_LEVEL OFF)
endif()

option(VOLCONTACT_BUILD_BENCHMARK "Build the scenarios, engine backends and apps" ${VOLCONTACT_IS_TOP_LEVEL})
option(VOLCONTACT_WITH_DRAKE "Build the Drake engine backend (needs the benchmark)" ON)
option(VOLCONTACT_WITH_CHRONO "Build the Chrono engine backend (needs the benchmark)" ON)
option(VOLCONTACT_WITH_MUJOCO "Build the MuJoCo engine backend (needs the benchmark)" ON)
option(VOLCONTACT_BUILD_TESTS "Build the unit tests" ${VOLCONTACT_IS_TOP_LEVEL})
# OFF by default: -march=native makes a library that only runs on CPUs like the build host's, which
# is wrong for anything installed or shipped. Turn it ON for a build you run where you compile it
# (about 8 % faster intersections); scripts/verify and scripts/bench.sh do, so benchmark numbers
# stay comparable.
option(VOLCONTACT_NATIVE "Compile the core with -march=native (not portable)" OFF)
option(VOLCONTACT_OPENMP "Evaluate contact pairs in parallel with OpenMP" ON)
option(VOLCONTACT_INSTALL "Generate the install rules" ${VOLCONTACT_IS_TOP_LEVEL})
option(VOLCONTACT_PIC "Compile the core as position-independent code (static libraries)" ON)

# Global settings only when this is the top-level project; a parent project keeps its own.
if(VOLCONTACT_IS_TOP_LEVEL)
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  endif()
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

if(VOLCONTACT_BUILD_TESTS)
  enable_language(C)  # The C API test is compiled as C.
endif()

# Eigen 3.4 or newer. The version is checked here, not by find_package: Eigen 5's package file refuses
# a request for 3.4 (same-major rule) although the core compiles unchanged against it.
find_package(Eigen3 REQUIRED NO_MODULE)
if(Eigen3_VERSION VERSION_LESS 3.4)
  message(FATAL_ERROR "Eigen 3.4 or newer is required, found ${Eigen3_VERSION}")
endif()

add_library(volcontact_warnings INTERFACE)
if(MSVC)
  # /W4 is MSVC's -Wall -Wextra; /permissive- is standard conformance; /utf-8 reads the sources as UTF-8.
  target_compile_options(volcontact_warnings INTERFACE /W4 /permissive- /utf-8)
  target_compile_definitions(volcontact_warnings INTERFACE _USE_MATH_DEFINES NOMINMAX)
else()
  target_compile_options(volcontact_warnings INTERFACE -Wall -Wextra -Wpedantic -Wshadow
    $<$<COMPILE_LANGUAGE:CXX>:-Wnon-virtual-dtor
    # Designated initializers that rely on default member initializers are intended.
    -Wno-missing-field-initializers>)
endif()

add_subdirectory(core)

if(VOLCONTACT_BUILD_BENCHMARK)
  if(NOT CMAKE_CXX_STANDARD)  # The benchmark code is built with the project's settings.
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
  endif()
  add_subdirectory(scenarios)
  add_subdirectory(engines)
  add_subdirectory(apps)
endif()

if(VOLCONTACT_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
