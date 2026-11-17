# Engine-agnostic scenario descriptions and metrics.
file(GLOB SCENARIO_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(volcontact_scenarios ${SCENARIO_SOURCES})
add_library(volcontact::scenarios ALIAS volcontact_scenarios)
target_include_directories(volcontact_scenarios PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(volcontact_scenarios PUBLIC volcontact::core PRIVATE volcontact_warnings)
