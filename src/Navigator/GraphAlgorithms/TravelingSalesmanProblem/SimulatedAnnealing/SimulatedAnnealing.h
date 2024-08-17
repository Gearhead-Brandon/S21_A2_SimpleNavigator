/**
 * @file SimulatedAnnealing.h
 * @brief Header file describing the class SimulatedAnnealing
 */

#ifndef SIMULATED_ANNEALING_H
#define SIMULATED_ANNEALING_H

#include <vector>

#include "../../../Components/TsmResult/TsmResult.h"
#include "../../../Graph/s21_graph.h"

namespace s21 {

/**
 * @brief Class describing the simulated annealing algorithm
 *        for solving the traveling salesman problem
 */
class SimulatedAnnealing {
  //! Initial temperature
  double initialTemperature_;

  //! Final temperature
  double finalTemperature_;

  //! Cooling rate
  double coolingRate_;

  //! Current path
  std::vector<int> currentSolution_;

  //! Infinity
  const double INF_;

 public:
  /**
   * @brief Parametrized constructor
   * @param countOfVertices - count of vertices
   */
  SimulatedAnnealing(int countOfVertices);

  /**
   * @brief Solve the traveling salesman problem
   * @param graph - the graph
   * @return The result of the traveling salesman problem
   */
  TsmResult solveTsp(Graph &graph);

 private:
  /**
   * @brief Generate the initial solution
   * @param countOfVertices - count of vertices
   * @return The initial solution
   */
  std::vector<int> generateInitialSolution(int countOfVertices);

  /**
   * @brief Calculate the length of the route
   * @param route - the route
   * @param graph - the graph
   * @return The length of the route
   */
  double calculateRouteLength(const std::vector<int> &route,
                              const Graph &graph);

  /**
   * @brief Generate a new solution
   * @param route - the route
   * @param graph - the graph
   */
  void generateNewSolution(std::vector<int> &route, const Graph &graph);
};
}  // namespace s21

#endif