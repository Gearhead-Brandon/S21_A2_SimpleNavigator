/**
 * @file SimulatedAnnealing.cpp
 * @brief Implementation of the class SimulatedAnnealing
 */

#include "SimulatedAnnealing.h"

#include <algorithm>
#include <chrono>

#include "random"

namespace s21 {

/**
 * @brief Parametrized constructor
 * @param countOfVertices - count of vertices
 */
SimulatedAnnealing::SimulatedAnnealing(int countOfVertices)
    : initialTemperature_(10000.0),
      finalTemperature_(0.000001),
      coolingRate_(countOfVertices >= 11 ? 0.99999 : 0.95),
      currentSolution_(generateInitialSolution(countOfVertices)),
      INF_(std::numeric_limits<double>::max()) {}

/**
 * @brief Solve the traveling salesman problem
 * @param graph - the graph
 * @return The result of the traveling salesman problem
 */
auto SimulatedAnnealing::solveTsp(Graph &graph) -> TsmResult {
  TsmResult result;
  result.distance = calculateRouteLength(currentSolution_, graph);
  result.vertices = currentSolution_;

  for (double temperature = initialTemperature_;
       temperature > finalTemperature_; temperature *= coolingRate_) {
    std::vector<int> new_solution = currentSolution_;
    generateNewSolution(new_solution, graph);

    double new_length = calculateRouteLength(new_solution, graph);
    double delta_E = new_length - result.distance;

    if (delta_E < 0 ||
        std::exp(-delta_E / temperature) > (double)rand() / RAND_MAX) {
      currentSolution_ = new_solution;

      if (new_length < result.distance) {
        result.distance = new_length;
        result.vertices = std::move(new_solution);
      }
    }
  }

  return result;
}

/**
 * @brief Generate the initial solution
 * @param countOfVertices - count of vertices
 * @return The initial solution
 */
auto SimulatedAnnealing::generateInitialSolution(int countOfVertices)
    -> std::vector<int> {
  std::vector<int> route(countOfVertices);

  // Fill the route with numbers from 0 to n - 1
  std::iota(route.begin(), route.end(), 0);

  std::random_device rd;
  std::mt19937 gen(rd());

  // Shuffle the route
  std::shuffle(route.begin(), route.end(), gen);

  return route;
}

/**
 * @brief Calculate the length of the route
 * @param route - the route
 * @param graph - the graph
 * @return The length of the route
 */
auto SimulatedAnnealing::calculateRouteLength(const std::vector<int> &route,
                                              const Graph &graph) -> double {
  double length = 0.0;
  int weight = 0;

  for (std::size_t i = 0; i < route.size() - 1; ++i) {
    weight = graph.getWeight(route[i], route[i + 1]);
    if (weight > 0)
      length += weight;
    else
      return INF_;
  }

  weight = graph.getWeight(route.back(), route[0]);

  if (weight > 0)
    length += weight;
  else
    return INF_;

  return length;
}

/**
 * @brief Generate a new solution
 * @param route - the route
 * @param graph - the graph
 */
void SimulatedAnnealing::generateNewSolution(std::vector<int> &route,
                                             const Graph &graph) {
  int countOfVertices = route.size();

  std::uniform_int_distribution<> dist(0, countOfVertices - 1);
  std::default_random_engine re(
      std::chrono::system_clock::now().time_since_epoch().count());

  int i = dist(re);
  int j = dist(re);

  // Check if the edge exists between the two cities
  if (graph.getWeight(route[i], route[j]) != 0) {
    std::swap(route[i], route[j]);
  } else {
    // If the edge doesn't exist, then generate a new solution
    generateNewSolution(route, graph);
  }
}
}  // namespace s21