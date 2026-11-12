# Engine interface plus one optional backend per engine.
add_library(volcontact_engines registry.cpp)
add_library(volcontact::engines ALIAS volcontact_engines)
target_include_directories(volcontact_engines
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(volcontact_engines PUBLIC volcontact::scenarios PRIVATE volcontact_warnings)

if(VOLCONTACT_WITH_DRAKE)
  find_package(drake CONFIG REQUIRED)
  target_sources(volcontact_engines PRIVATE
    drake_backend/drake_engine.cpp
    drake_backend/volumetric_contact_system.cpp
  )
  target_link_libraries(volcontact_engines PRIVATE drake::drake)
  target_compile_definitions(volcontact_engines PRIVATE VOLCONTACT_WITH_DRAKE)
endif()

if(VOLCONTACT_WITH_CHRONO)
  find_package(Chrono CONFIG REQUIRED)
  target_sources(volcontact_engines PRIVATE chrono_backend/chrono_engine.cpp)
  target_link_libraries(volcontact_engines PRIVATE Chrono::Chrono_core)
  target_compile_definitions(volcontact_engines PRIVATE VOLCONTACT_WITH_CHRONO)
endif()

if(VOLCONTACT_WITH_MUJOCO)
  list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
  if(POLICY CMP0144)  # MUJOCO_ROOT is a search prefix of find_package(MuJoCo).
    cmake_policy(SET CMP0144 NEW)
  endif()
  find_package(MuJoCo REQUIRED)
  target_sources(volcontact_engines PRIVATE mujoco_backend/mujoco_engine.cpp)
  target_link_libraries(volcontact_engines PRIVATE mujoco::mujoco)
  target_compile_definitions(volcontact_engines PRIVATE VOLCONTACT_WITH_MUJOCO)
endif()
