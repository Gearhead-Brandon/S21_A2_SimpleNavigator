/**
 * @file s21_graph.cpp
 * @brief Implementation of the class for storing the graph
 */

#include "s21_graph.h"

#include <filesystem>

#include "fstream"
#include "sstream"

namespace s21 {

/**
 * @brief Class describing the graph
 */
Graph::Graph() : adjacencyMatrix_() {}

/**
 * @brief Load graph from file
 * @param filename - path to the file
 * @return Operation result
 */
OpResult Graph::LoadGraphFromFile(std::string filename) {
  std::ifstream file(filename);

  if (std::filesystem::is_directory(filename))
    return OpResult(false, "Is a directory, not a file");

  if (!file.is_open()) return OpResult(false, "File not found");

  int size = 0;

  file >> size;

  if (size <= 0) return {false, "Incorrect matrix size"};

  file >> std::ws;

  S21Matrix<int> newMatrix(size, size);
  newMatrix.Fill(0);

  std::string line("");

  for (int i = 0; i < size; ++i) {
    std::getline(file, line);
    std::istringstream iss(line);
    for (int j = 0; j < size; ++j) {
      int value = -1;

      iss >> value;

      if (value <= -1) return {false, "Incorrect rib weight"};

      newMatrix(i, j) = value;
    }

    if (iss.rdbuf()->in_avail() > 0) return {false, "Incorrect row format"};
  }

  adjacencyMatrix_ = std::move(newMatrix);

  return {true, ""};
}

/**
 * @brief Export graph to .dot file
 * @param filename - path to the file
 */
void Graph::ExportGraphToDot(std::string filename) {
  int size = adjacencyMatrix_.GetRows();

  if (size <= 0 || filename.length() == 0) return;

  std::ofstream file;

  if (filename.length() < 4 || filename.substr(filename.length() - 4) != ".dot")
    filename += ".dot";

  file.open(filename);

  file << "graph graphname {\n";

  for (int i = 0; i < size; ++i) file << "\t" << i + 1 << ";\n";

  for (int i = 0; i < size; ++i) {
    // Check for self-loop (loop handling)
    if (adjacencyMatrix_(i, i) != 0)
      file << "\t" << i + 1 << " -- " << i + 1 << ";\n";

    for (int j = 0; j < i + 1; ++j)
      if (adjacencyMatrix_(i, j) != 0)
        if (adjacencyMatrix_(j, i) == 0)
          file << "\t" << i + 1 << " -> " << j + 1 << ";\n";

    // Iterate through other nodes to find edges
    for (int j = i + 1; j < size; ++j)
      if (adjacencyMatrix_(i, j) != 0) {
        if (adjacencyMatrix_(j, i) == 0)
          file << "\t" << i + 1 << " -> " << j + 1 << ";\n";
        else
          file << "\t" << i + 1 << " -- " << j + 1 << ";\n";
      }
  }

  file << "}";
}

/**
 * @brief Get weight of the edge
 * @param start_vertex - start vertex
 * @param end_vertex - end vertex
 * @return Weight
 */
int Graph::getWeight(int start_vertex, int end_vertex) const {
  return (start_vertex < 0 || end_vertex < 0)
             ? 0
             : adjacencyMatrix_(start_vertex, end_vertex);
}

/**
 * @brief Check if the edge is oriented
 * @param start_vertex - start vertex
 * @param end_vertex - end vertex
 * @return True if the edge is oriented
 */
bool Graph::isEdgeOriented(int start_vertex, int end_vertex) {
  return (adjacencyMatrix_(start_vertex, end_vertex) !=
          adjacencyMatrix_(end_vertex, start_vertex));
}

/**
 * @brief Get adjacent vertices
 * @param start_vertex - start vertex
 * @return Adjacent vertices
 */
std::vector<int> Graph::getAdjacentVertices(int start_vertex) {
  std::vector<int> adjacent_vertices;
  for (int i = 0; i < adjacencyMatrix_.GetRows(); ++i)
    if (adjacencyMatrix_(start_vertex, i) != 0) adjacent_vertices.push_back(i);

  return adjacent_vertices;
}

/**
 * @brief Get number of vertices in the graph
 * @return Size
 */
int Graph::size() const { return adjacencyMatrix_.GetRows(); }
}  // namespace s21