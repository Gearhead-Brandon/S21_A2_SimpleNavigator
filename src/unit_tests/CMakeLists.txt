cmake_minimum_required(VERSION 3.0)

project(Simplenavigator LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_COMPILER "/usr/bin/g++-11")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fprofile-arcs -ftest-coverage -Wall -Werror -Wextra") 

file(GLOB Main
    "*.cpp"
)

file(GLOB_RECURSE Graph_Files
    "../Navigator/Graph/*.cpp"
    "../Navigator/Components/*.cpp" 
)

file(GLOB_RECURSE GraphAlgorithms_Files
    "../Navigator/GraphAlgorithms/*.cpp"
    "../Navigator/Components/*.cpp" 
)
 
#Create a library target
add_library(s21_graph STATIC ${Graph_Files})
add_library(s21_graph_algorithms STATIC ${GraphAlgorithms_Files})

# Create an executable target
add_executable(tests ${MAIN} ${Main})

# Add necessary libraries or dependencies
target_link_libraries(
    tests
    s21_graph
    s21_graph_algorithms
    -lgtest
)

set_target_properties(
    tests
    PROPERTIES
    LINK_FLAGS "--coverage"
    ENABLE_TESTING TRUE
)