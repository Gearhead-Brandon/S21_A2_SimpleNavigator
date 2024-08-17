/**
 * @file AntColonyOptimization.h
 * @brief Header file describing the class AntColonyOptimization
 */

#ifndef ANT_COLONY_OPTIMIZATION_H
#define ANT_COLONY_OPTIMIZATION_H

#include <vector>

#include "../../../Components/TsmResult/TsmResult.h"
#include "../../../Graph/s21_graph.h"

namespace s21 {

/**
 * @brief Class describing the AntColonyOptimization
 *        algorithm for solving the traveling salesman problem
 */
class AntColonyOptimization {
  //! Alpha parameter
  const double alpha_;

  //! Beta parameter
  const double beta_;

  //! Evaporation parameter
  const double evaporation_;

  //! Q parameter
  const double Q_;

  //! Number of ants
  const int numAnts_;

  //! Number of iterations
  const int iterations_;

  //! Pheromone matrix
  S21Matrix<double> pheromone_;

  //! Infinity
  const double INF;

 public:
  /**
   * @brief Parametrized constructor
   * @param countOfVertices - count of vertices
   */
  AntColonyOptimization(int countOfVertices);

  /**
   * @brief Solve the traveling salesman problem
   * @param graph - the graph
   * @return The result of the traveling salesman problem
   */
  TsmResult solveTsp(Graph& graph);

 private:
  /**
   * @brief Calculate the length of the route
   * @param graph - the graph
   * @param tour - the tour
   * @return The length of the route
   */
  double calculateRouteLength(Graph& graph, const std::vector<int>& tour);

  /**
   * @brief Check for duplicates in the tour
   * @param numbers - the tour
   * @param size - the size of the tour
   * @return True if the tour has duplicates
   */
  bool hasDuplicates(const std::vector<int>& numbers, int size);

  /**
   * @brief Check if the graph is totally connected
   * @param graph - the graph
   * @return True if the graph is totally connected
   *         False otherwise
   */
  bool isGraphTotallyConnected(Graph& graph);

  /**
   * @brief Evaporate the pheromone matrix and update it
   * @param graph - the graph
   * @param tours - the tours
   */
  void updatePheromone(Graph& graph,
                       const std::vector<std::vector<int>>& tours);

  /**
   * @brief Add steps to an unconnected graph
   * @param numOfVertices - the number of vertices
   * @param tour - the tour
   * @return True if the tour is invalid
   */
  bool unconnectedGraphProcessing(int numOfVertices, std::vector<int>& tour);

  /**
   * @brief Update the best tour
   * @param graph - the graph
   * @param tour - the tour
   * @param bestResult - the best tour result
   * @return True if the tour is better
   *         False otherwise
   */
  bool bestTourUpdate(Graph& graph, std::vector<int>& tour,
                      TsmResult& bestResult);

  /**
   * @brief One tour of the ants
   * @param graph - the graph
   * @param tour - the tour
   * @param graphIsTotallyConnected - the graph is totally connected
   */
  void antsTour(Graph& graph, std::vector<int>& tour,
                bool graphIsTotallyConnected);
};
}  // namespace s21

#endif