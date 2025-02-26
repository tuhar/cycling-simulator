# add_library(CyclingSimulator
#     CyclingSimulator.cpp
# )
# target_include_directories(CyclingSimulatorLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# target_compile_features(CyclingSimulatorLib PUBLIC cxx_std_20)

add_executable(CyclingSimulator main.cpp)
target_compile_features(CyclingSimulator PRIVATE cxx_std_20)
# target_link_libraries(CyclingSimulator PRIVATE CyclingSimulatorLib)