#ifndef S21_GRAPH_H
#define S21_GRAPH_H

#include <iostream>
#include "../Common/S21Matrix/S21Matrix.h"

namespace s21{

  class Graph {

    S21Matrix<int> adjacencyMatrix_;

  public:
    Graph();
    ~Graph() = default;

    void LoadGraphFromFile(std::string filename);

    void SaveGraphToFile(std::string filename);
  };

}

#endif