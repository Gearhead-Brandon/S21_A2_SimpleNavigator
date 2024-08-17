/**
 * @file s21_graph_algorithms.cpp
 * @brief Implementation of the class GraphAlgorithms
 */

#include "s21_graph_algorithms.h"

#include <algorithm>
#include <chrono>
#include <queue>
#include <set>
#include <unordered_set>

#include "random"

namespace s21 {

/**
 * @brief Operator <
 * @param other - other vertex
 * @return True if the distance is less
 */
bool GraphAlgorithms::GraphVertex::operator<(const GraphVertex &other) const {
  return distance > other.distance;
}

/**
 * @brief Depth-first traversal algorithm for graph
 * @param graph - the graph
 * @param start_vertex - start vertex
 * @return The result of the traversal
 */
auto GraphAlgorithms::DepthFirstSearch(Graph &graph, int start_vertex)
    -> std::vector<int> {
  --start_vertex;

  // Array to keep track of visited vertices
  std::vector<bool> visited(graph.size(), false);
  // Add the starting vertex to the stack and mark it as visited
  visited[start_vertex] = true;

  // Stack to store the vertices
  // in the order in which they were visited
  s21::stack<int> s;
  // Add the starting vertex to the stack
  s.push(start_vertex);

  // Vector to store the path
  // in the order in which it was visited
  std::vector<int> path;

  while (!s.empty()) {
    int v = s.top();
    s.pop();

    path.push_back(v + 1);

    for (int neighbor : graph.getAdjacentVertices(v)) {
      if (!visited[neighbor]) {
        visited[neighbor] = true;
        s.push(neighbor);
      }
    }
  }

  return path;
}

/**
 * @brief Breadth-first traversal algorithm for graph
 * @param graph - the graph
 * @param start_vertex - start vertex
 * @return The result of the traversal
 */
auto GraphAlgorithms::BreadthFirstSearch(Graph &graph, int start_vertex)
    -> std::vector<int> {
  --start_vertex;

  // Array to keep track of visited vertices
  std::vector<bool> visited(graph.size(), false);
  // Add the starting vertex to the queue and mark it as visited
  visited[start_vertex] = true;

  // Queue to store the vertices
  // in the order in which they were visited
  s21::queue<int> q;

  // Add the starting vertex to the queue
  q.push(start_vertex);

  // Vector to store the path
  // in the order in which it was visited
  std::vector<int> path{};

  while (!q.empty()) {
    int v = q.front();
    q.pop();

    path.push_back(v + 1);

    for (int neighbor : graph.getAdjacentVertices(v)) {
      if (!visited[neighbor]) {
        visited[neighbor] = true;
        q.push(neighbor);
      }
    }
  }

  return path;
}

/**
 * @brief Get shortest path between vertices by Dejkstra's algorithm
 * @param graph - the graph
 * @param vertex1 - start vertex
 * @param vertex2 - end vertex
 * @return The shortest path between vertices
 */
std::size_t GraphAlgorithms::GetShortestPathBetweenVertices(Graph &graph,
                                                            int vertex1,
                                                            int vertex2) {
  // Dijkstra algorithm
  --vertex1;
  --vertex2;

  int size = graph.size();

  std::vector<int> distances(size, static_cast<int>(Constants::INF));
  distances[vertex1] = 0;

  std::priority_queue<GraphVertex> pq;
  pq.push({vertex1, 0});

  std::vector<bool> visited(size, false);

  while (!pq.empty()) {
    int v = pq.top().vertex;
    pq.pop();

    if (visited[v]) continue;

    visited[v] = true;

    if (v == vertex2) return distances[v];

    for (int n : graph.getAdjacentVertices(v)) {
      int weight = graph.getWeight(v, n);

      if (distances[v] + weight < distances[n]) {
        distances[n] = distances[v] + weight;
        pq.push({n, distances[n]});
      }
    }
  }

  return std::numeric_limits<std::size_t>::max();
}

/**
 * @brief Init adjacency matrix for Floyd-Warshall algorithm
 * @param graph - the graph
 * @return The adjacency matrix
 */
auto GraphAlgorithms::FloydWarshallAlgorithmMatrixInit(Graph &graph)
    -> S21Matrix<int> {
  int size = graph.size();
  std::vector<bool> visited(size, false);
  S21Matrix<int> matrix(size, size);

  for (int i = 0; i < size; ++i) {
    for (int j = 0; j < size; ++j) {
      int weight = graph.getWeight(i, j);

      if (weight == 0 && i != j) weight = static_cast<int>(Constants::INF);

      matrix(i, j) = weight;
    }
  }

  return matrix;
}

/**
 * @brief Get shortest paths between all vertices by Floyd-Warshall algorithm
 * @param graph - the graph
 * @return The shortest paths between all vertices in adjacency matrix
 */
auto GraphAlgorithms::GetShortestPathsBetweenAllVertices(Graph &graph)
    -> S21Matrix<int> {
  S21Matrix<int> adjacencyMatrix = FloydWarshallAlgorithmMatrixInit(graph);

  int size = graph.size();

  // Floyd-Warshall algorithm
  for (int k = 0; k < size; ++k) {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        int w_i_k = adjacencyMatrix(i, k);
        int w_k_j = adjacencyMatrix(k, j);

        if (w_i_k == static_cast<int>(Constants::INF) ||
            w_k_j == static_cast<int>(Constants::INF))
          continue;

        adjacencyMatrix(i, j) = std::min(adjacencyMatrix(i, j), w_i_k + w_k_j);
      }
    }
  }

  return adjacencyMatrix;
}

/**
 * @brief Get the least spanning tree by Prim's algorithm
 * @param graph - the graph
 * @return The least spanning tree in adjacency matrix
 */
auto GraphAlgorithms::GetLeastSpanningTree(Graph &graph) -> S21Matrix<int> {
  if (!isGraphConnected(graph)) return {0, 0};

  // Prim's algorithm
  int size = graph.size();
  int start_vertex = 0;

  std::vector<bool> visited(size, false);

  std::vector<int> spantree(size, -1);

  std::vector<int> key(size, static_cast<int>(Constants::INF));
  key[start_vertex] = 0;

  // Minimum heap for selecting the edge with minimum weight
  std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>,
                      std::greater<std::pair<int, int>>>
      pq;
  pq.push({0, start_vertex});

  while (!pq.empty()) {
    int u = pq.top().second;
    pq.pop();

    if (visited[u]) continue;

    visited[u] = true;

    for (int v : graph.getAdjacentVertices(u)) {
      int weight = graph.getWeight(u, v);
      if (!visited[v] && weight < key[v]) {
        spantree[v] = u;
        key[v] = weight;
        pq.push({weight, v});
      }
    }
  }

  return PrimAlgorithmAdjacencyMatrixCreate(graph, spantree);
}

/**
 * @brief Create adjacency matrix for Prim's algorithm from spantree
 * @param graph - the graph
 * @param spantree - the spantree
 * @return The adjacency matrix
 */
auto GraphAlgorithms::PrimAlgorithmAdjacencyMatrixCreate(
    Graph &graph, const std::vector<int> &spantree) -> S21Matrix<int> {
  int size = graph.size();
  S21Matrix<int> mst(size, size);
  mst.Fill(0);

  for (int i = 1; i < size; ++i) {
    // if (spantree[i] == -1) {
    //     std::cout << "\033[31mNo MST exists\033[0m" << std::endl;
    //     //continue;
    // }

    mst(spantree[i], i) = graph.getWeight(spantree[i], i);

    if (!graph.isEdgeOriented(spantree[i], i))
      mst(i, spantree[i]) =
          graph.getWeight(i, spantree[i]);  // Для неориентированного графа
  }

  return mst;
}

/**
 * @brief Solve the traveling salesman problem by ant algorithm
 * @param graph - the graph
 * @return The result of the traveling salesman problem
 */
auto GraphAlgorithms::SolveTravelingSalesmanProblem(Graph &graph) -> TsmResult {
  if (!isGraphConnected(graph))
    return {{}, static_cast<double>(TsmResultErrors::GraphIsNotConnected)};

  AntColonyOptimization aoc(graph.size());

  return tsmResultNormalization(aoc.solveTsp(graph));
}

/**
 * @brief Solve the traveling salesman problem by simulated annealing
 * @param graph - the graph
 * @return The result of the traveling salesman problem
 */
auto GraphAlgorithms::SolveTravelingSalesmanProblemThroughSimulatedAnnealing(
    Graph &graph) -> TsmResult {
  if (!isGraphConnected(graph))
    return {{}, static_cast<double>(TsmResultErrors::GraphIsNotConnected)};

  SimulatedAnnealing sa(graph.size());

  return tsmResultNormalization(sa.solveTsp(graph));
}

/**
 * @brief Solve the traveling salesman problem by genetic algorithm
 * @param graph - the graph
 * @return The result of the traveling salesman problem
 */
auto GraphAlgorithms::SolveTravelingSalesmanProblemThroughGeneticAlgorithm(
    Graph &graph) -> TsmResult {
  if (!isGraphConnected(graph))
    return {{}, static_cast<double>(TsmResultErrors::GraphIsNotConnected)};

  GeneticAlghoritm ga(graph.size());

  return tsmResultNormalization(ga.solveTsp(graph));
}

/**
 * @brief Normalization of the result of the traveling salesman problem
 * @param result - the result of the traveling salesman problem
 * @return The result of the traveling salesman problem
 */
TsmResult GraphAlgorithms::tsmResultNormalization(TsmResult result) {
  if (result.distance == std::numeric_limits<double>::max())
    return {{}, static_cast<double>(TsmResultErrors::ThereIsNoHamiltonCycle)};

  std::transform(result.vertices.begin(), result.vertices.end(),
                 result.vertices.begin(), [](int x) { return x + 1; });

  if (!result.vertices.empty() && result.vertices.back() != result.vertices[0])
    result.vertices.push_back(result.vertices[0]);

  return result;
}

/**
 * @brief Check if the graph is connected
 * @param graph - the graph
 * @return True if the graph is connected
 */
bool GraphAlgorithms::isGraphConnected(Graph &graph) {
  for (int i = 0; i < graph.size(); ++i) {
    std::vector<int> path = BreadthFirstSearch(graph, i + 1);

    if (static_cast<int>(path.size()) != graph.size()) return false;
  }

  return true;
}
}  // namespace s21