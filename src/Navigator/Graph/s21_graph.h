/**
 * @file s21_graph.h
 * @brief Header file describing the class for storing the graph
 */

#ifndef S21_GRAPH_H
#define S21_GRAPH_H

#include <iostream>
#include <vector>

#include "../Components/OpResult/OpResult.h"
#include "../Components/S21Matrix/S21Matrix.h"

namespace s21 {

/**
 * @brief Class describing the graph
 */
class Graph {
  //! Matrix of the graph
  S21Matrix<int> adjacencyMatrix_;

 public:
  /**
   * @brief Constructor
   */
  Graph();

  /**
   * @brief Destructor
   */
  ~Graph() = default;

  /**
   * @brief Load graph from file
   * @param filename - path to the file
   * @return Operation result
   */
  OpResult LoadGraphFromFile(std::string filename);

  /**
   * @brief Export graph to .dot file
   * @param filename - path to the file
   */
  void ExportGraphToDot(std::string filename);

  /**
   * @brief Get adjacent vertices
   * @param start_vertex - start vertex
   * @return Adjacent vertices
   */
  std::vector<int> getAdjacentVertices(int start_vertex);

  /**
   * @brief Get weight of the edge
   * @param start_vertex - start vertex
   * @param end_vertex - end vertex
   * @return Weight
   */
  int getWeight(int start_vertex, int end_vertex) const;

  /**
   * @brief Get number of vertices in the graph
   * @return Size
   */
  int size() const;

  /**
   * @brief Check if the edge is oriented
   * @param start_vertex - start vertex
   * @param end_vertex - end vertex
   * @return True if the edge is oriented
   */
  bool isEdgeOriented(int start_vertex, int end_vertex);
};

}  // namespace s21

#endif