/**
 * @file ConsoleUI.h
 * @brief Header file describing the class ConsoleUI
 */

#ifndef CONSOLE_UI_H
#define CONSOLE_UI_H

#include <functional>
#include <iostream>

#include "../GraphAlgorithms/s21_graph_algorithms.h"
#include "Enum/Options.h"

namespace s21 {

/**
 * @brief Class describing the console user interface
 *        (the main class of the program)
 */
class UI {
  //! The pointer to the graph
  Graph* graph_;

 public:
  /**
   * @brief The constructor
   */
  UI();

  /**
   * @brief The destructor
   */
  ~UI();

  /**
   * @brief Start the event loop
   */
  void StartEventLoop();

 private:
  /**
   * @brief Print input error message
   */
  void printInputError();

  /**
   * @brief Check if the string is a number
   * @param str
   * @return True if the string is a number
   */
  bool isNumber(const std::string& str);

  /**
   * @brief Print menu with available operations
   */
  void printMenu();

  /**
   * @brief Print error message about not existing operation
   */
  void printNotExistingOperation();

  /**
   * @brief Print selected operation
   * @param op Selected operation
   * @param isExit Exit flag
   */
  void printSelectedOperation(Options op, bool& isExit);

  /**
   * @brief Load adjacency matrix from file
   */
  void loadGraphFromFile();

  /**
   * @brief Save graph to file in .dot format
   */
  void saveGraphToFile();

  /**
   * @brief Print error message about graph size
   */
  void printGraphSizeError();

  /**
   * @brief Print Breath First Search result
   */
  void printBFSResult();

  /**
   * @brief Print Depth First Search result
   */
  void printDFSResult();

  /**
   * @brief Print Dejkstra algorithm result
   */
  void printDejkstraResult();

  /**
   * @brief Print Floyd-Warshall algorithm result (Shortest Path Matrix)
   */
  void printFloydWarshallResult();

  /**
   * @brief Print Spanning Tree result (Adjacency Matrix)
   */
  void printSpanningTreeResult();

  /**
   * @brief Print Tsm algorithm result (TsmResult struct)
   */
  void printTsmResult();

  /**
   * @brief Execute Tsm algorithm
   * @param N Number of solutions
   * @param message Message for the user
   * @param func Function to execute
   */
  void executeTsmAlgorithm(int N, const std::string& message,
                           std::function<TsmResult(Graph&)> func);

  /**
   * @brief Get vertex from the user and check if it is valid
   * @param size Size of the graph
   */
  int getVertex(int size);

  /**
   * @brief Clear the screen using ANSI escape sequences
   */
  void clearScreen();

  /**
   * @brief Print vector of vertices
   * @param vec Vector of vertices
   * @param message Message for the user
   */
  void printVectorOfVertices(const std::vector<int>& vec,
                             const std::string& message);

  /**
   * @brief Print matrix
   * @param matrix Matrix
   * @param message Message for the user
   */
  void printMatrix(S21Matrix<int>& matrix, const char* message);
};
}  // namespace s21

#endif