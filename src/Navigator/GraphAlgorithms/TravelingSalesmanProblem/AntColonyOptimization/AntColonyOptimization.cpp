/**
 * @file AntColonyOptimization.cpp
 * @brief Implementation of the class AntColonyOptimization
 */

#include "AntColonyOptimization.h"

#include <algorithm>
#include <chrono>
#include <random>
#include <set>

namespace s21 {

/**
 * @brief Parametrized constructor
 * @param countOfVertices - count of vertices
 */
AntColonyOptimization::AntColonyOptimization(int countOfVertices)
    : alpha_(1.0),
      beta_(0.4),
      evaporation_(0.02),
      Q_(1.0),
      numAnts_(countOfVertices * 2),
      iterations_(100),
      pheromone_(countOfVertices, countOfVertices),
      INF(std::numeric_limits<double>::max()) {
  for (int i = 0; i < pheromone_.GetRows(); ++i)
    for (int j = 0; j < pheromone_.GetCols(); ++j)
      pheromone_(i, j) = (i != j ? 0.2 : 0.0);
}

/**
 * @brief Calculate the length of the route
 * @param graph - the graph
 * @param tour - the tour
 * @return The length of the route
 */
auto AntColonyOptimization::calculateRouteLength(Graph &graph,
                                                 const std::vector<int> &tour)
    -> double {
  double tourLength = 0.0;

  for (std::size_t i = 0; i < tour.size() - 1; ++i)
    tourLength += graph.getWeight(tour[i], tour[i + 1]);

  return tourLength + graph.getWeight(tour[tour.size() - 1], tour[0]);
}

/**
 * @brief Check for duplicates in the tour
 * @param numbers - the tour
 * @param size - the size of the tour
 * @return True if the tour has duplicates
 */
bool AntColonyOptimization::hasDuplicates(const std::vector<int> &numbers,
                                          int size) {
  std::set<int> unique_numbers;

  for (int i = 0; i < size; ++i) {
    auto [_, inserted] = unique_numbers.insert(numbers[i]);

    if (!inserted) return true;
  }

  return false;
}

/**
 * @brief Check if the graph is totally connected
 * @param graph - the graph
 * @return True if the graph is totally connected
 *         False otherwise
 */
bool AntColonyOptimization::isGraphTotallyConnected(Graph &graph) {
  for (int i = 0; i < graph.size(); ++i)
    for (int j = i + 1; j < graph.size(); ++j)
      if (!graph.getWeight(i, j)) return false;

  return true;
}

/**
 * @brief Solve the traveling salesman problem
 * @param graph - the graph
 * @return The result of the traveling salesman problem
 */
TsmResult AntColonyOptimization::solveTsp(Graph &graph) {
  const bool k_isGraphTotallyConnected = isGraphTotallyConnected(graph);

  int numVertices = graph.size();

  TsmResult bestResult;
  bestResult.distance = std::numeric_limits<double>::max();

  for (int iter = 0; iter < iterations_; ++iter) {
    std::vector<std::vector<int>> tours(numAnts_, std::vector<int>());

    // Start the ants
    for (int ant = 0; ant != numAnts_; ++ant) {
      // Start the tour
      antsTour(graph, tours[ant], k_isGraphTotallyConnected);

      // Adding steps to an unconnected graph
      if (!k_isGraphTotallyConnected &&
          unconnectedGraphProcessing(numVertices, tours[ant]))
        continue;

      // Update the best tour result
      if (bestTourUpdate(graph, tours[ant], bestResult)) iter = 0;
    }

    // Evaporate the pheromone
    updatePheromone(graph, tours);
  }

  return bestResult;
}

/**
 * @brief One tour of the ants
 * @param graph - the graph
 * @param tour - the tour
 * @param graphIsTotallyConnected - the graph is totally connected
 */
void AntColonyOptimization::antsTour(Graph &graph, std::vector<int> &tour,
                                     bool graphIsTotallyConnected) {
  int numVertices = graph.size();

  std::uniform_int_distribution<> dist(0, numVertices - 1);
  std::default_random_engine re(
      std::chrono::system_clock::now().time_since_epoch().count());

  int goal = dist(re);
  int currentVertex = goal;

  tour.push_back(currentVertex);

  std::vector<bool> visited(numVertices, false);
  visited[currentVertex] = true;

  bool found = false;

  int prevVertex = -1;

  while (!found) {
    std::vector<double> probabilities(numVertices, 0.0);
    double probability_sum = 0.0;

    for (int j : graph.getAdjacentVertices(currentVertex)) {
      if (graphIsTotallyConnected && visited[j]) continue;

      double w = (1.0 / pow(graph.getWeight(currentVertex, j), beta_));

      probabilities[j] = pow(pheromone_(currentVertex, j), alpha_) * w;

      probability_sum += probabilities[j];
    }

    double random = static_cast<double>(rand()) / RAND_MAX * probability_sum;

    int nextVertex = 0;

    // Choose the next vertex by roulette wheel
    while (random > 0.0 && nextVertex < numVertices) {
      random -= probabilities[nextVertex];
      ++nextVertex;
    }

    --nextVertex;

    if (nextVertex == prevVertex) break;

    prevVertex = currentVertex;

    if (nextVertex < 0) break;

    tour.push_back(nextVertex);
    visited[nextVertex] = true;
    currentVertex = nextVertex;

    //////////////////////////////////////////
    if (currentVertex == goal &&
        std::count(visited.begin(), visited.end(), true) == numVertices)
      found = true;
  }
}

/**
 * @brief Update the best tour
 * @param graph - the graph
 * @param tour - the tour
 * @param bestResult - the best tour result
 * @return True if the tour is better
 *         False otherwise
 */
bool AntColonyOptimization::bestTourUpdate(Graph &graph, std::vector<int> &tour,
                                           TsmResult &bestResult) {
  double tourLength = calculateRouteLength(graph, tour);

  if (tourLength > 0 && tourLength < bestResult.distance) {
    bestResult.vertices = tour;
    bestResult.distance = tourLength;
    return true;
  }

  return false;
}

/**
 * @brief Add steps to an unconnected graph
 * @param numOfVertices - the number of vertices
 * @param tour - the tour
 * @return True if the tour is invalid
 */
bool AntColonyOptimization::unconnectedGraphProcessing(int numOfVertices,
                                                       std::vector<int> &tour) {
  if (tour.size() < static_cast<std::size_t>(numOfVertices + 1) ||
      hasDuplicates(tour, tour.size() - 1)) {
    tour[0] = -1;
    return true;
  }

  return false;
}

/**
 * @brief Evaporate the pheromone matrix and update it
 * @param graph - the graph
 * @param tours - the tours
 */
void AntColonyOptimization::updatePheromone(
    Graph &graph, const std::vector<std::vector<int>> &tours) {
  int numVertices = graph.size();

  for (int i = 0; i < numVertices; ++i) {
    for (int j = 0; j < numVertices; ++j) {
      if (i == j) continue;

      pheromone_(i, j) *= (1 - evaporation_);

      pheromone_(j, i) = pheromone_(i, j);
    }
  }

  for (int ant = 0; ant < numAnts_; ++ant) {
    if (tours[ant][0] == -1) continue;

    double length = calculateRouteLength(graph, tours[ant]);
    double delta = Q_ / length;

    for (std::size_t i = 0; i < tours[ant].size() - 1; ++i) {
      pheromone_(tours[ant][i], tours[ant][i + 1]) += delta;
      pheromone_(tours[ant][i + 1], tours[ant][i]) += delta;
    }

    pheromone_(tours[ant][tours[ant].size() - 1], tours[ant][0]) += delta;
    pheromone_(tours[ant][0], tours[ant][tours[ant].size() - 1]) += delta;
  }
}
}  // namespace s21