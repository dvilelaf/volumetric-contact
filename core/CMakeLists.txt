# Engine-agnostic geometry and contact model. Depends on Eigen only (and OpenMP, optionally).
add_library(volcontact_core
  src/bsp.cpp
  src/bvh.cpp
  src/c_api.cpp
  src/compound.cpp
  src/contact_model.cpp
  src/distributed.cpp
  src/gonthier.cpp
  src/intersection.cpp
  src/mesh.cpp
  src/pcm.cpp
  src/polar_moments.cpp
)
add_library(volcontact::core ALIAS volcontact_core)
set_target_properties(volcontact_core PROPERTIES
  EXPORT_NAME core
  OUTPUT_NAME volcontact
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  POSITION_INDEPENDENT_CODE ${VOLCONTACT_PIC})
target_compile_features(volcontact_core PUBLIC cxx_std_20)
target_include_directories(volcontact_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
# The warnings are for building this project only: they must not end up in the exported target.
target_link_libraries(volcontact_core PUBLIC Eigen3::Eigen PRIVATE $<BUILD_INTERFACE:volcontact_warnings>)

# Host-specific SIMD only for the core (about 8% faster intersections). Its
# headers use no Eigen types with alignment requirements, so it links safely with
# generic binaries such as Drake's; compiling the backends with -march=native
# would change Eigen's alignment and break that ABI. The hot code (clip_inside,
# VolumeIntegrator) is instantiated and inlined only inside the core. Off by default
# (see the option in the top-level CMakeLists.txt).
if(VOLCONTACT_NATIVE)
  target_compile_options(volcontact_core PRIVATE -march=native)
endif()

if(VOLCONTACT_OPENMP)
  # Optional: Apple Clang ships without OpenMP, and then the core evaluates the pairs serially.
  find_package(OpenMP COMPONENTS CXX)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(volcontact_core PRIVATE OpenMP::OpenMP_CXX)
    target_compile_definitions(volcontact_core PRIVATE VOLCONTACT_OPENMP)
    set(VOLCONTACT_HAS_OPENMP ON)
  else()
    message(STATUS "OpenMP not found: contact pairs are evaluated serially")
  endif()
endif()

if(VOLCONTACT_INSTALL)
  include(GNUInstallDirs)
  include(CMakePackageConfigHelpers)
  set(VOLCONTACT_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/volcontact)

  install(TARGETS volcontact_core EXPORT volcontactTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
  install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
  install(EXPORT volcontactTargets NAMESPACE volcontact:: DESTINATION ${VOLCONTACT_CMAKE_DIR})

  # Dependencies that consumers of a static build must find as well.
  set(VOLCONTACT_USES_OPENMP ${VOLCONTACT_HAS_OPENMP})
  configure_package_config_file(${PROJECT_SOURCE_DIR}/cmake/volcontactConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/volcontactConfig.cmake INSTALL_DESTINATION ${VOLCONTACT_CMAKE_DIR})
  write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/volcontactConfigVersion.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/volcontactConfig.cmake
                ${CMAKE_CURRENT_BINARY_DIR}/volcontactConfigVersion.cmake
          DESTINATION ${VOLCONTACT_CMAKE_DIR})
endif()
