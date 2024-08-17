cmake_minimum_required(VERSION 3.0)

project(Simplenavigator LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_COMPILER "/usr/bin/g++-11")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Werror -Wextra") 
 
#Create a library target
if(BUILD_GRAPH_LIBRARY OR BUILD_NAVIGATOR_EXE)
    file(GLOB_RECURSE Graph_Files
        "./Navigator/Graph/*.cpp"
        "./Navigator/Components/*.cpp"
    )

    add_library(s21_graph STATIC ${Graph_Files})
    set_target_properties(s21_graph PROPERTIES PREFIX "")
endif()

if(BUILD_GRAPH_ALGORITHMS_LIBRARY OR BUILD_NAVIGATOR_EXE)
    file(GLOB_RECURSE GraphAlgorithms_Files
        "./Navigator/GraphAlgorithms/*.cpp"
        "./Navigator/Components/*.cpp"
    )

    add_library(s21_graph_algorithms STATIC ${GraphAlgorithms_Files})
    set_target_properties(s21_graph_algorithms PROPERTIES PREFIX "")
endif()

# Create an executable target
if(BUILD_NAVIGATOR_EXE)
    file(GLOB Main
        "./main.cpp"
        "./Navigator/ConsoleUI/*.cpp"
    )

    add_executable(SimpleNavigator ${MAIN} ${Main})
    target_link_libraries(
        SimpleNavigator 
        s21_graph 
        s21_graph_algorithms
    )
endif()