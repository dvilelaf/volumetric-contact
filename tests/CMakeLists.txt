include(FetchContent)
# DOWNLOAD_EXTRACT_TIMESTAMP exists from CMake 3.24; on 3.22 it would be read as part of URL_HASH.
set(VOLCONTACT_FETCH_OPTIONS)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
  list(APPEND VOLCONTACT_FETCH_OPTIONS DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
endif()
FetchContent_Declare(doctest
  URL https://github.com/doctest/doctest/archive/refs/tags/v2.4.11.tar.gz
  URL_HASH SHA256=632ed2c05a7f53fa961381497bf8069093f0d6628c5f26286161fbd32a560186
  ${VOLCONTACT_FETCH_OPTIONS}
)
FetchContent_GetProperties(doctest)
if(NOT doctest_POPULATED)
  FetchContent_Populate(doctest)
endif()

# Core only (no benchmark needed): the C API, driven from C and compared with the C++ API.
add_executable(volcontact_core_tests capi_test.cpp capi_drive.c)
target_include_directories(volcontact_core_tests PRIVATE ${doctest_SOURCE_DIR})
target_link_libraries(volcontact_core_tests PRIVATE volcontact::core volcontact_warnings)
add_test(NAME volcontact_core_tests COMMAND volcontact_core_tests)

if(VOLCONTACT_BUILD_BENCHMARK)
  file(GLOB TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp)
  add_executable(volcontact_tests ${TEST_SOURCES})
  target_include_directories(volcontact_tests PRIVATE ${doctest_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/scenarios/src)
  # Engine tests pick their backend from available_engines() and skip it if absent.
  target_link_libraries(volcontact_tests PRIVATE volcontact::scenarios volcontact::engines volcontact_warnings)
  target_compile_definitions(volcontact_tests PRIVATE VOLCONTACT_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
  add_test(NAME volcontact_tests COMMAND volcontact_tests)
endif()
