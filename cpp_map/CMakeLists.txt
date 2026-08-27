cmake_minimum_required(VERSION 3.16)
project(mower_map LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(mower_map src/grid_map.cpp)
target_include_directories(mower_map PUBLIC include)
target_compile_options(mower_map PRIVATE -Wall -Wextra -Wpedantic)

add_executable(mower_map_demo examples/demo.cpp)
target_link_libraries(mower_map_demo PRIVATE mower_map)

add_executable(mower_coverage_demo examples/coverage_demo.cpp)
target_link_libraries(mower_coverage_demo PRIVATE mower_map)

enable_testing()
add_executable(mower_map_test tests/grid_map_test.cpp)
target_link_libraries(mower_map_test PRIVATE mower_map)
add_test(NAME mower_map_test COMMAND mower_map_test)
