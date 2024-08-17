/**
 * @file GeneticAlghoritm.cpp
 * @brief Implementation of the class GeneticAlghoritm
 */

#include "GeneticAlghoritm.h"

#include <algorithm>
#include <chrono>
#include <random>
#include <set>

namespace s21 {

/**
 * @brief Parametrized constructor
 * @param countOfVertices - count of vertices
 */
GeneticAlghoritm::GeneticAlghoritm(int countOfVertices)
    : populationSize_(800),
      generations_(100),
      mutationRate_(0.01),
      population_(initPopulation(countOfVertices)),
      INF_(std::numeric_limits<double>::max()) {}

/**
 * @brief Solve the traveling salesman problem
 * @param graph - the graph
 * @return The result of the traveling salesman problem
 */
TsmResult GeneticAlghoritm::solveTsp(const Graph &graph) {
  for (auto &individual : population_)
    individual.adaptability = calculateRouteLength(individual.route, graph);

  for (int i = 0; i < generations_; ++i) {
    // New population
    std::vector<Individual> newPopulation;

    while (static_cast<int>(newPopulation.size()) < populationSize_) {
      Individual parent1 = tournamentSelection(population_, 5);
      Individual parent2 = tournamentSelection(population_, 5);
      Individual child = crossover(parent1, parent2);

      if (rand() / (double)RAND_MAX < mutationRate_) child = mutation(child);

      child.adaptability = calculateRouteLength(child.route, graph);

      newPopulation.push_back(child);
    }

    population_ = std::move(newPopulation);
  }

  auto best =
      std::move(std::min_element(population_.begin(), population_.end(),
                                 [](const Individual &a, const Individual &b) {
                                   return a.adaptability < b.adaptability;
                                 }));

  TsmResult result;
  result.distance = best->adaptability;
  result.vertices = std::move(best->route);

  return result;
}

/**
 * @brief Tournament selection
 * @param population - the population
 * @param tournamentSize - the size of the tournament
 * @return The selected individual
 */
auto GeneticAlghoritm::tournamentSelection(
    const std::vector<Individual> &population, int tournamentSize)
    -> Individual {
  std::vector<int> tournamentCandidates;

  for (int i = 0; i < tournamentSize; ++i)
    tournamentCandidates.push_back(rand() % population.size());

  std::sort(tournamentCandidates.begin(), tournamentCandidates.end(),
            [&population](int i, int j) {
              return population[i].adaptability < population[j].adaptability;
            });

  return population[tournamentCandidates[0]];
}

/**
 * @brief Crossover of the individuals
 * @param parent1 - the first parent
 * @param parent2 - the second parent
 * @return The child
 */
auto GeneticAlghoritm::crossover(const Individual &parent1,
                                 const Individual &parent2) -> Individual {
  std::uniform_int_distribution<> dist(0, parent1.route.size() - 1);
  std::default_random_engine re(
      std::chrono::system_clock::now().time_since_epoch().count());

  int crossoverPoint = dist(re) + 1;

  Individual child;

  child.route.insert(child.route.end(), parent1.route.begin(),
                     parent1.route.begin() + crossoverPoint);

  std::set<int> usedCities(child.route.begin(), child.route.end());

  for (int gene : parent2.route)
    if (!usedCities.count(gene)) child.route.push_back(gene);

  return child;
}

/**
 * @brief Mutation of the individual
 * @param individual - the individual
 * @return The mutated individual
 */
auto GeneticAlghoritm::mutation(const Individual &individual) -> Individual {
  Individual mutant = individual;

  int start = rand() % mutant.route.size();
  int end = rand() % mutant.route.size();

  std::reverse(mutant.route.begin() + std::min(start, end),
               mutant.route.begin() + std::max(start, end) + 1);

  return mutant;
}

/**
 * @brief Generate the initial population
 * @param countOfVertices - count of vertices
 * @return The vector of initial population
 */
auto GeneticAlghoritm::initPopulation(int countOfVertices)
    -> std::vector<Individual> {
  std::vector<Individual> population;

  for (int i = 0; i < populationSize_; ++i) {
    std::vector<int> route(countOfVertices);

    std::iota(route.begin(), route.end(),
              0);  // Заполняем маршрут числами от 0 до numCities-1

    std::random_shuffle(route.begin(), route.end());  // Перемешиваем маршрут

    population.push_back(
        {std::move(route), INF_});  // Инициализируем фитнес бесконечностью
  }

  return population;
}

/**
 * @brief Calculate the route length
 * @param route - the route
 * @param graph - the graph
 * @return The length of the route
 */
auto GeneticAlghoritm::calculateRouteLength(const std::vector<int> &route,
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
}  // namespace s21