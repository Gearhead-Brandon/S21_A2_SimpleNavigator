/**
 * @file GeneticAlghoritm.h
 * @brief Header file describing the class GeneticAlghoritm
 *        for solving the traveling salesman problem
 */

#ifndef GENETIC_ALGHORITM_H
#define GENETIC_ALGHORITM_H

#include <vector>

#include "../../../Components/TsmResult/TsmResult.h"
#include "../../../Graph/s21_graph.h"

namespace s21 {

/**
 * @brief Class describing the genetic alghoritm
 *        for solving the traveling salesman problem
 */
class GeneticAlghoritm {
  /**
   * @brief Class describing the individual
   */
  struct Individual {
    //! Route of the individual
    std::vector<int> route;

    //! Adaptability of the individual
    double adaptability = 0.0;
  };

  //! Population size
  const int populationSize_;

  //! Number of generations
  const int generations_;

  //! Mutation rate
  const double mutationRate_;

  //! Population of individuals
  std::vector<Individual> population_;

  //! Infinity
  const double INF_;

 public:
  /**
   * @brief Parametrized constructor
   * @param countOfVertices - count of vertices
   */
  GeneticAlghoritm(int countOfVertices);

  /**
   * @brief Solve the traveling salesman problem
   * @param graph - the graph
   * @return The result of the traveling salesman problem
   */
  TsmResult solveTsp(const Graph& graph);

 private:
  /**
   * @brief Generate the initial population
   * @param countOfVertices - count of vertices
   * @return The vector of initial population
   */
  std::vector<Individual> initPopulation(int countOfVertices);

  /**
   * @brief Calculate the route length
   * @param route - the route
   * @param graph - the graph
   * @return The length of the route
   */
  double calculateRouteLength(const std::vector<int>& route,
                              const Graph& graph);

  /**
   * @brief Mutation of the individual
   * @param individual - the individual
   * @return The mutated individual
   */
  Individual mutation(const Individual& individual);

  /**
   * @brief Crossover of the individuals
   * @param parent1 - the first parent
   * @param parent2 - the second parent
   * @return The child
   */
  Individual crossover(const Individual& parent1, const Individual& parent2);

  /**
   * @brief Tournament selection
   * @param population - the population
   * @param tournamentSize - the size of the tournament
   * @return The selected individual
   */
  Individual tournamentSelection(const std::vector<Individual>& population,
                                 int tournamentSize);
};
}  // namespace s21

#endif