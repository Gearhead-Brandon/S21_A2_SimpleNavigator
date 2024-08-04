#ifndef S21_GRAPH_H
#define S21_GRAPH_H

#include <iostream>
#include "../Components/S21Matrix/S21Matrix.h"
#include "../Components/OpResult/OpResult.h"
#include <vector>

namespace s21
{

  class Graph{

    S21Matrix<int> adjacencyMatrix_;

  public:
    Graph();
    ~Graph() = default;

    OpResult LoadGraphFromFile(std::string filename);

    void ExportGraphToDot(std::string filename);

    //int operator()(int i, int j) const;

    std::vector<int> getAdjacentVertices(int start_vertex);

    int size() const;

  private:
    void reset();
  };

}

#endif