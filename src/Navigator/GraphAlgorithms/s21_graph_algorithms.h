/**
 * @file s21_graph_algorithms.h
 * @brief Header file describing the class GraphAlgorithms
 */

#ifndef S21_GRAPH_ALGORITHMS_H
#define S21_GRAPH_ALGORITHMS_H

#include <limits>

#include "../Components/Containers/Queue/s21_queue.h"
#include "../Components/Containers/Stack/s21_stack.h"
#include "../Components/TsmResult/TsmResult.h"
#include "../Graph/s21_graph.h"
#include "TravelingSalesmanProblem/AntColonyOptimization/AntColonyOptimization.h"
#include "TravelingSalesmanProblem/GeneticAlghoritm/GeneticAlghoritm.h"
#include "TravelingSalesmanProblem/SimulatedAnnealing/SimulatedAnnealing.h"

namespace s21 {

/**
 * @brief Class describing the algorithms for graph
 */
class GraphAlgorithms {
  /**
   * @brief Class describing the constants
   */
  enum class Constants {
    INF = std::numeric_limits<int>::max()  //! Max value of int
  };

  /**
   * @brief Class describing the GraphVertex
   *        (a vertex of the graph)
   */
  struct GraphVertex {
    //! Number of the vertex
    int vertex;

    //! Distance from the start vertex
    int distance;

    /**
     * @brief Operator <
     * @param other - other vertex
     * @return True if the distance is less
     */
    bool operator<(const GraphVertex &other) const;
  };

 public:
  /**
   * @brief Constructor
   */
  GraphAlgorithms() = default;

  /**
   * @brief Destructor
   */
  ~GraphAlgorithms() = default;

  /**
   * @brief Depth-first traversal algorithm for graph
   * @param graph - the graph
   * @param start_vertex - start vertex
   * @return The result of the traversal
   */
  static std::vector<int> DepthFirstSearch(Graph &graph, int start_vertex);

  /**
   * @brief Breadth-first traversal algorithm for graph
   * @param graph - the graph
   * @param start_vertex - start vertex
   * @return The result of the traversal
   */
  static std::vector<int> BreadthFirstSearch(Graph &graph, int start_vertex);

  /**
   * @brief Get shortest path between vertices by Dejkstra's algorithm
   * @param graph - the graph
   * @param vertex1 - start vertex
   * @param vertex2 - end vertex
   * @return The shortest path between vertices
   */
  static std::size_t GetShortestPathBetweenVertices(Graph &graph, int vertex1,
                                                    int vertex2);

  /**
   * @brief Get shortest paths between all vertices by Floyd-Warshall algorithm
   * @param graph - the graph
   * @return The shortest paths between all vertices in adjacency matrix
   */
  static S21Matrix<int> GetShortestPathsBetweenAllVertices(Graph &graph);

  /**
   * @brief Get the least spanning tree by Prim's algorithm
   * @param graph - the graph
   * @return The least spanning tree in adjacency matrix
   */
  static S21Matrix<int> GetLeastSpanningTree(Graph &graph);

  /**
   * @brief Solve the traveling salesman problem by ant algorithm
   * @param graph - the graph
   * @return The result of the traveling salesman problem
   */
  static TsmResult SolveTravelingSalesmanProblem(Graph &graph);

  /**
   * @brief Solve the traveling salesman problem by simulated annealing
   * @param graph - the graph
   * @return The result of the traveling salesman problem
   */
  static TsmResult SolveTravelingSalesmanProblemThroughSimulatedAnnealing(
      Graph &graph);

  /**
   * @brief Solve the traveling salesman problem by genetic algorithm
   * @param graph - the graph
   * @return The result of the traveling salesman problem
   */
  static TsmResult SolveTravelingSalesmanProblemThroughGeneticAlgorithm(
      Graph &graph);

 private:
  /**
   * @brief Init adjacency matrix for Floyd-Warshall algorithm
   * @param graph - the graph
   * @return The adjacency matrix
   */
  static S21Matrix<int> FloydWarshallAlgorithmMatrixInit(Graph &graph);

  /**
   * @brief Create adjacency matrix for Prim's algorithm from spantree
   * @param graph - the graph
   * @param spantree - the spantree
   * @return The adjacency matrix
   */
  static S21Matrix<int> PrimAlgorithmAdjacencyMatrixCreate(
      Graph &graph, const std::vector<int> &spantree);

  /**
   * @brief Normalization of the result of the traveling salesman problem
   * @param result - the result of the traveling salesman problem
   * @return The result of the traveling salesman problem
   */
  static TsmResult tsmResultNormalization(TsmResult result);

  /**
   * @brief Check if the graph is connected
   * @param graph - the graph
   * @return True if the graph is connected
   */
  static bool isGraphConnected(Graph &graph);
};
}  // namespace s21

#endif