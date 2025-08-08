find_package(Catch2 3 REQUIRED)

add_executable(tests SparseSetTest.cpp)
target_link_libraries(tests PRIVATE CyclingSimulatorLib Catch2::Catch2WithMain)
target_include_directories(tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)

include(CTest)
include(Catch)
catch_discover_tests(tests TEST_PREFIX unit. WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} DISCOVERY_MODE PRE_TEST)