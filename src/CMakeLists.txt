cmake_minimum_required(VERSION 3.0)

project(Simplenavigator LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_COMPILER "/usr/bin/g++-11")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Werror -Wextra") 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 ") 

file(GLOB Main
    "./main.cpp"
)

file(GLOB_RECURSE Components
    "./Navigator/Components/*.cpp"
)

file(GLOB Graph_Files
    "./Navigator/Graph/*.cpp"
    "./Navigator/GraphAlgorithms/*.cpp"
)
 
#Create a library target
add_library(Components STATIC ${Components})
add_library(Graph STATIC ${Graph_Files})

# Create an executable target
add_executable(SimpleNavigator ${MAIN} ${Main})

# Add necessary libraries or dependencies
target_link_libraries(
    SimpleNavigator
    Graph
    Components
)