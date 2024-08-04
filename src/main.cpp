#include <iostream>
#include "Navigator/GraphAlgorithms/s21_graph_algorithms.h"

using namespace s21;

int main() {
    
    Graph graph;
    OpResult result = graph.LoadGraphFromFile("../GraphFilesExample/graph4_v2");

    std::cout << std::endl << "------------------------------------------" << std::endl;
    std::cout << result.getErrorMessage() << std::endl;
    std::cout << "------------------------------------------" << std::endl << std::endl;

    // result = graph.LoadGraphFromFile("../GraphFilesExample/graph4_v2");

    // std::cout << std::endl << "------------------------------------------" << std::endl;
    // std::cout << result.getErrorMessage() << std::endl;
    // std::cout << "------------------------------------------" << std::endl << std::endl;


    graph.ExportGraphToDot("../GraphFilesExample/graph4");

    GraphAlgorithms algorithms;

    std::vector<int> path = std::move(algorithms.DepthFirstSearch(graph, 1));

    for (int v : path) {
        std::cout << v << " ";
    }

    std::cout << std::endl << std::endl;

    return 0;
}