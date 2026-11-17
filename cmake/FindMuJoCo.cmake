# Finds the MuJoCo release binaries, which ship libmujoco.so and headers but no
# CMake package config. Looks in MUJOCO_ROOT (variable or environment) and
# /opt/mujoco, and defines the imported target mujoco::mujoco.
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
  HINTS ${MUJOCO_ROOT} ENV MUJOCO_ROOT /opt/mujoco PATH_SUFFIXES include)
find_library(MUJOCO_LIBRARY mujoco
  HINTS ${MUJOCO_ROOT} ENV MUJOCO_ROOT /opt/mujoco PATH_SUFFIXES lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MuJoCo DEFAULT_MSG MUJOCO_LIBRARY MUJOCO_INCLUDE_DIR)

if(MuJoCo_FOUND AND NOT TARGET mujoco::mujoco)
  add_library(mujoco::mujoco SHARED IMPORTED)
  set_target_properties(mujoco::mujoco PROPERTIES
    IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}")
endif()
mark_as_advanced(MUJOCO_INCLUDE_DIR MUJOCO_LIBRARY)
