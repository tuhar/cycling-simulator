cmake_minimum_required(VERSION 3.31.2)

project(CyclingSimulator VERSION 0.0.1
                DESCRIPTION "Cycling Simulator - fun side project"
                LANGUAGES CXX)

add_subdirectory(src)

add_custom_target(
    run ALL,
    COMMAND CyclingSimulator
)