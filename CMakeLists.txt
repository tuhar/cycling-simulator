cmake_minimum_required(VERSION 3.31.2)

project(CyclingSimulatorLib VERSION 0.0.1
                DESCRIPTION "Cycling Simulator - fun side project"
                LANGUAGES CXX)

find_package(SFML 3.0 REQUIRED COMPONENTS Graphics Window System)
find_package(fmt REQUIRED)

add_subdirectory(src)

add_custom_target(
    run ALL,
    COMMAND CyclingSimulator
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)