cmake_minimum_required(VERSION 3.11)
project(Molecular_Dynamics)
enable_testing()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_STANDARD 17)

include(tools)

add_external_package(Eigen3 VERSION 3.4.0 CONFIG)

set(_HDRS
        # Add header files here
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/verlet.h
        )

set(_SRCS
        # Add source files here
        ${CMAKE_CURRENT_SOURCE_DIR}/src/verlet.cpp
        )

set(_EXEC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
        )

add_subdirectory(tests)

# Simulation code
add_executable(Mol_Dyn ${_SRCS} ${_HDRS} ${_EXEC})
target_link_libraries(Mol_Dyn PUBLIC Eigen3::Eigen)
