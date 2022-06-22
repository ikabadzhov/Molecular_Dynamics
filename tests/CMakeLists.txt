include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/eaf9a3fd77869cf95befb87455a2e2a2e85044ff.zip
)
FetchContent_MakeAvailable(googletest)

set(_TST
        ${CMAKE_CURRENT_SOURCE_DIR}/test_verlet.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/test_atoms.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/test_lj_direct_summation.cpp
        )

# Tests
include_directories(tests ${CMAKE_SOURCE_DIR})
add_executable(tests ${_SRCS} ${_HDRS} ${_TST})
target_link_libraries(tests PUBLIC Eigen3::Eigen gtest gtest_main)

include(GoogleTest)
gtest_discover_tests(tests)
