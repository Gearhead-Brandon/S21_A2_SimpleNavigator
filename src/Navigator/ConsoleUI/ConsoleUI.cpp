/**
 * @file ConsoleUI.cpp
 * @brief Implementation of the class ConsoleUI
 */

#include "ConsoleUI.h"

#include <chrono>
#include <filesystem>
#include <numeric>

#include "algorithm"

namespace s21 {

/**
 * @brief The constructor
 */
UI::UI() : graph_(new Graph) { srand(time(0)); }

/**
 * @brief The destructor
 */
UI::~UI() { delete graph_; }

/**
 * @brief Start the event loop
 */
void UI::StartEventLoop() {
  clearScreen();

  std::string input{};
  bool continueLoop = true;

  while (continueLoop) {
    printMenu();

    std::getline(std::cin, input);

    if (isNumber(input))
      printSelectedOperation(static_cast<Options>(stoi(input) - 1),
                             continueLoop);
    else
      printInputError();
  }
}

/**
 * @brief Clear the screen using ANSI escape sequences
 */
void UI::clearScreen() { std::cout << "\033c"; }

/**
 * @brief Print menu with available operations
 */
void UI::printMenu() {
  std::cout << "---------------------------------------\n"
            << "|   \033[35m 1. Load graph from file \033[0m         |\n"
            << "+-------------------------------------+\n"
            << "|   \033[34m 2. Save graph to file \033[0m           |\n"
            << "+-------------------------------------+\n"
            << "|   \033[32m 3. Breadth-first graph traversal \033[0m|\n"
            << "--------------------------------------+\n"
            << "|   \033[33m 4. Depth-first graph traversal \033[0m  |\n"
            << "+-------------------------------------+\n"
            << "|   \033[36m 5. Finding the shortest path \033[0m    |\n"
            << "|   \033[36m    between two vertices \033[0m         |\n"
            << "+-------------------------------------+\n"
            << "|   \033[32m 6. Finding the shortest paths \033[0m   |\n"
            << "|   \033[32m    between all pairs of vertices \033[0m|\n"
            << "+-------------------------------------+\n"
            << "|   \033[34m 7. Finding the minimum spanning \033[0m |\n"
            << "|   \033[34m    tree in a graph  \033[0m             |\n"
            << "+-------------------------------------+\n"
            << "|   \033[33m 8. Solving the traveling  \033[0m       |\n"
            << "|   \033[33m    salesman problem \033[0m             |\n"
            << "+-------------------------------------+\n"
            << "|   \033[31m 9. Exit \033[0m                         |\n"
            << "---------------------------------------\n"
            << "\n  \033[33mSelect operation:\033[0m ";
}

/**
 * @brief Print selected operation
 * @param op Selected operation
 * @param isExit Exit flag
 */
void UI::printSelectedOperation(Options op, bool &continueLoop) {
  if (static_cast<int>(op) < 0 || static_cast<int>(op) > 8) {
    printNotExistingOperation();
    return;
  }

  if (op > Options::LoadGraph && op < Options::Exit && !graph_->size()) {
    clearScreen();
    printGraphSizeError();
    return;
  }

  clearScreen();

  switch (op) {
    case Options::LoadGraph:
      loadGraphFromFile();
      break;
    case Options::SaveGraph:
      saveGraphToFile();
      break;
    case Options::BFS:
      printBFSResult();
      break;
    case Options::DFS:
      printDFSResult();
      break;
    case Options::Dejkstra:
      printDejkstraResult();
      break;
    case Options::FloydWarshall:
      printFloydWarshallResult();
      break;
    case Options::SpanningTree:
      printSpanningTreeResult();
      break;
    case Options::Tsm:
      printTsmResult();
      break;
    case Options::Exit:
      continueLoop = false;
      break;
  }
}

/**
 * @brief Check if the string is a number
 * @param str
 * @return True if the string is a number
 */
bool UI::isNumber(const std::string &s) {
  return !s.empty() &&
         std::all_of(s.begin() + ((s[0] == '+' || s[0] == '-') ? 1 : 0),
                     s.end(), ::isdigit);
}

/**
 * @brief Print input error message
 */
void UI::printInputError() {
  clearScreen();
  std::cout << "   --------------------------------\n"
            << "   |\033[31m    Input error. Try again  \033[0m  |\n"
            << "   --------------------------------\n\n";
}

/**
 * @brief Print error message about not existing operation
 */
void UI::printNotExistingOperation() {
  clearScreen();
  std::cout << "   --------------------------------\n"
            << "   |\033[31m    Operation doesn't exist   \033[0m |\n"
            << "   --------------------------------\n";
}

/**
 * @brief Load adjacency matrix from file
 */
void UI::loadGraphFromFile() {
  std::cout << "   --------------------------------\n"
            << "   |\033[32m    Load graph from file     \033[0m |\n"
            << "   | \033[32m  Or enter 'q' to get back  \033[0m |\n"
            << "   --------------------------------\n\n";

  std::string input;

  std::cout << "\033[33mInput path to file >> ";
  std::getline(std::cin, input);
  std::cout << "\033[0m";

  clearScreen();

  if (input.size() == 1 && input[0] == 'q') return;

  OpResult result = graph_->LoadGraphFromFile(input);

  if (!result.IsSuccess()) {
    std::cout << "   ---------------------------------\n"
              << "         \033[31m" << result.getErrorMessage()
              << "\033[0m          \n"
              << "   ---------------------------------\n\n";
    return;
  }

  std::cout << "   --------------------------------\n"
            << "   |\033[32m   File loaded successfully   \033[0m|\n"
            << "   --------------------------------\n\n";
}

/**
 * @brief Save graph to file in .dot format
 */
void UI::saveGraphToFile() {
  std::cout << "   --------------------------------\n"
            << "   |\033[32m    Save graph to file       \033[0m |\n"
            << "   | \033[32m  Or enter 'q' to get back  \033[0m |\n"
            << "   --------------------------------\n\n";

  std::string input;

  std::cout << "\033[33mInput path and name new file >> ";
  std::getline(std::cin, input);
  std::cout << "\033[0m";

  clearScreen();

  if (input.size() == 1 && input[0] == 'q') return;

  graph_->ExportGraphToDot(input);

  clearScreen();

  std::cout << "   --------------------------------\n"
            << "   |\033[32m     File saved successfully\033[0m  |\n"
            << "   --------------------------------\n\n";
}

/**
 * @brief Print error message about graph size
 */
void UI::printGraphSizeError() {
  clearScreen();
  std::cout << "    --------------------------------\n"
            << "    |\033[31m      Graph not loaded       \033[0m |\n"
            << "    --------------------------------\n\n";
}

/**
 * @brief Print Breath First Search result
 */
void UI::printBFSResult() {
  int size = graph_->size();

  std::cout << "   ----------------------------------\n"
            << "   | \033[36m     Breadth-first search     \033[0m |\n"
            << "   | \033[32m      Enter start vertex      \033[0m |\n"
            << "   | \033[32m        (from 1 to " << size
            << ")         \033[0m |\n"
            << "   | \033[31m    Or enter 'q' to get back  \033[0m |\n"
            << "   ----------------------------------\n\n";

  int startVertex = getVertex(size);

  if (startVertex == -1) return;

  std::vector<int> result =
      GraphAlgorithms::BreadthFirstSearch(*graph_, startVertex);

  printVectorOfVertices(result, "Breadth-first search result");
}

/**
 * @brief Get vertex from the user and check if it is valid
 * @param size Size of the graph
 */
int UI::getVertex(int size) {
  int vertex = 0;

  std::string input;

  std::cout << "\033[33mInput vertex from \033[36m1\033[33m to \033[36m" << size
            << "\033[33m >> ";
  std::getline(std::cin, input);
  std::cout << "\033[0m";

  clearScreen();

  if (input.size() < 1) {
    printInputError();
    return -1;
  } else if (input.size() == 1 && input[0] == 'q')
    return -1;
  else if (isNumber(input))
    vertex = stoi(input);
  else {
    printInputError();
    return -1;
  }

  if (vertex < 1 || vertex > size) {
    std::cout << "   --------------------------------\n"
              << "   | \033[31m   This vertex doesn't exist\033[0m |\n"
              << "   --------------------------------\n\n";

    return -1;
  }

  return vertex;
}

/**
 * @brief Print Depth First Search result
 */
void UI::printDFSResult() {
  int size = graph_->size();

  std::cout << "   ----------------------------------\n"
            << "   | \033[36m      Depth-first search      \033[0m |\n"
            << "   | \033[32m      Enter start vertex      \033[0m |\n"
            << "   | \033[32m        (from 1 to " << size
            << ")         \033[0m |\n"
            << "   | \033[31m    Or enter 'q' to get back  \033[0m |\n"
            << "   ----------------------------------\n\n";

  int startVertex = getVertex(size);

  if (startVertex == -1) return;

  std::vector<int> result =
      GraphAlgorithms::DepthFirstSearch(*graph_, startVertex);

  printVectorOfVertices(result, "Depth-first search result");
}

/**
 * @brief Print Dejkstra algorithm result
 */
void UI::printDejkstraResult() {
  int size = graph_->size();

  std::cout << "   ----------------------------------\n"
            << "   | \033[36m  Dejkstra's algorithm search \033[0m |\n"
            << "   | \033[32m      Enter\033[33m start\033[32m vertex      "
               "\033[0m |\n"
            << "   | \033[32m        from \033[33m1 \033[32mto \033[33m" << size
            << "\033[32m        " << (size >= 10 ? "" : " ") << "\033[0m   |\n"
            << "   | \033[31m    Or enter 'q' to get back  \033[0m |\n"
            << "   ----------------------------------\n\n";

  int startVertex = getVertex(size);

  if (startVertex == -1) return;

  std::cout << "   ----------------------------------\n"
            << "   | \033[36m  Dejkstra's algorithm search \033[0m |\n"
            << "   | \033[32m    Enter\033[33m destination\033[32m vertex  "
               "\033[0m |\n"
            << "   | \033[32m        from \033[33m1 \033[32mto \033[33m" << size
            << "\033[32m        " << (size >= 10 ? "" : " ") << "\033[0m   |\n"
            << "   | \033[31m    Or enter 'q' to get back  \033[0m |\n"
            << "   ----------------------------------\n\n";

  int endVertex = getVertex(size);

  if (endVertex == -1) return;

  std::size_t result = GraphAlgorithms::GetShortestPathBetweenVertices(
      *graph_, startVertex, endVertex);

  if (result == std::numeric_limits<std::size_t>::max())
    std::cout << "   ----------------------------------\n"
              << "    \033[36m  Dejkstra's algorithm search \033[0m \n"
              << "    \033[31m     Path doesn't exist      \033[0m \n"
              << "   ----------------------------------\n\n";
  else
    std::cout << "   ----------------------------------\n"
              << "    \033[36m  Dejkstra's algorithm search \033[0m \n"
              << "    \033[32m   Shortest path from \033[33m" << startVertex
              << "\033[32m to \033[33m" << endVertex << "\033[0m\n"
              << "     \033[33m           " << result << "\033[0m\n"
              << "   ----------------------------------\n\n";
}

/**
 * @brief Print Floyd-Warshall algorithm result (Shortest Path Matrix)
 */
void UI::printFloydWarshallResult() {
  S21Matrix<int> result =
      GraphAlgorithms::GetShortestPathsBetweenAllVertices(*graph_);

  clearScreen();

  printMatrix(result, "Matrix of shortest paths between all vertices");
  std::cout << "\n";
}

/**
 * @brief Print Spanning Tree result (Adjacency Matrix)
 */
void UI::printSpanningTreeResult() {
  clearScreen();

  S21Matrix<int> result = GraphAlgorithms::GetLeastSpanningTree(*graph_);

  if (result.GetCols() == 0)
    std::cout << "   ----------------------------------\n"
              << "    \033[31m  Spanning tree doesn't exist \033[0m \n"
              << "   ----------------------------------\n";
  else
    printMatrix(result, "Adjacency matrix of the least spanning tree");

  std::cout << "\n";
}

/**
 * @brief Print Tsm algorithm result (TsmResult struct)
 */
void UI::printTsmResult() {
  std::cout << "   ----------------------------------\n"
            << "   | \033[36m  Traveling Salesman Problem  \033[0m |\n"
            << "   | \033[31m   Or enter 'q' to get back   \033[0m |\n"
            << "   ----------------------------------\n\n";

  std::string input;

  std::cout << "\033[33mInput number of solutions >> ";
  std::getline(std::cin, input);
  std::cout << "\033[0m";

  clearScreen();

  int N = 0;

  if (input.size() < 1) {
    printInputError();
    return;
  } else if (input.size() == 1 && input[0] == 'q')
    return;
  else if (isNumber(input)) {
    N = stoi(input);
    if (N < 1) {
      printInputError();
      return;
    }
  } else {
    printInputError();
    return;
  }

  clearScreen();

  std::function<TsmResult(Graph &)> func =
      GraphAlgorithms::SolveTravelingSalesmanProblem;

  executeTsmAlgorithm(N, "ANT ALGORITHM", func);

  std::cout << std::endl;
  for (std::size_t i = 0; i < 40; ++i) std::cout << "---";
  std::cout << "--" << std::endl;

  func =
      GraphAlgorithms::SolveTravelingSalesmanProblemThroughSimulatedAnnealing;

  executeTsmAlgorithm(N, "SIMULATED ANNEALING ALGORITHM", func);

  std::cout << std::endl;
  for (std::size_t i = 0; i < 40; ++i) std::cout << "---";
  std::cout << "--" << std::endl;

  func = GraphAlgorithms::SolveTravelingSalesmanProblemThroughGeneticAlgorithm;

  executeTsmAlgorithm(N, "GENETIC ALGORITHM", func);

  std::cout << "\n";
}

/**
 * @brief Execute Tsm algorithm
 * @param N Number of solutions
 * @param message Message for the user
 * @param func Function to execute
 */
void UI::executeTsmAlgorithm(int N, const std::string &message,
                             std::function<TsmResult(Graph &)> func) {
  auto durationAntAlgorithm = std::chrono::duration<double, std::milli>::zero();

  for (int i = 0; i < N - 1; i++) {
    auto start = std::chrono::high_resolution_clock::now();
    func(*graph_);
    durationAntAlgorithm += std::chrono::high_resolution_clock::now() - start;
  }

  auto start = std::chrono::high_resolution_clock::now();
  auto [vertices, distance] = func(*graph_);
  durationAntAlgorithm += std::chrono::high_resolution_clock::now() - start;

  std::cout << "\n";

  std::cout << "   \033[36m " << message << " \033[0m" << std::endl
            << std::endl;
  if (distance == static_cast<int>(TsmResultErrors::GraphIsNotConnected))
    std::cout << "    ----------------------------------\n"
              << "    |\033[31m     Graph is not connected\033[0m     |\n"
              << "    ----------------------------------\n\n";
  else if (distance ==
           static_cast<int>(TsmResultErrors::ThereIsNoHamiltonCycle))
    std::cout << "    ----------------------------------\n"
              << "    |\033[31m    There is no solution to\033[0m     |\n"
              << "    | \033[31mthe traveling salesman problem\033[0m |\n"
              << "    ----------------------------------\n\n";
  else {
    std::cout << "    \033[32mTime in ms \033[33m"
              << durationAntAlgorithm.count() << "\033[0m" << std::endl
              << "    \033[32mVertices\033[33m " << graph_->size() << "\033[0m"
              << std::endl
              << "    \033[32mDistance \033[33m" << distance << "\033[0m"
              << std::endl;
    printVectorOfVertices(vertices, "PATH");
  }
}

/**
 * @brief Print matrix
 * @param matrix Matrix
 * @param message Message for the user
 */
void UI::printMatrix(S21Matrix<int> &matrix, const char *message) {
  int n = matrix.GetRows();

  std::vector<int> widthsCols(n, 1);

  for (int j = 0; j < n; j++)
    for (int i = 0; i < n; i++) {
      int num_width = 0;

      if (matrix(i, j) == INT32_MAX)
        num_width = 3;
      else
        num_width = std::to_string(matrix(i, j)).size();

      if (num_width > widthsCols[j]) widthsCols[j] = num_width;
    }

  int allWidth =
      6 + std::accumulate(widthsCols.begin(), widthsCols.end(), 0,
                          [](int sum, int a) { return sum + a + 2; });

  if (n >= 10) allWidth++;

  std::cout << std::endl
            << std::setw(allWidth / 4) << std::left << "\033[36m" << message
            << "\033[0m\n"
            << std::endl;

  for (int j = 0; j < allWidth + 2; j++) std::cout << '-';
  std::cout << std::endl;

  std::cout << "| \033[32m0\033[0m" << (n >= 10 ? "  " : " ") << "| ";
  for (int j = 0; j < n; j++)
    std::cout << "\033[32m" << std::setw(widthsCols[j] + 2) << std::left
              << j + 1;
  std::cout << "\033[0m" << /*(n >= 10 ? " " : "") <<*/ " |" << std::endl;

  std::cout << '+';
  for (int j = 0; j < allWidth; j++)
    std::cout << (j == (n >= 10 ? 4 : 3) ? '+' : '-');
  std::cout << '+' << std::endl;

  for (int i = 0; i < n; i++) {
    std::cout << "| \033[32m" << i + 1 << (n >= 10 && i < 9 ? " " : "")
              << "\033[0m | ";

    for (int j = 0; j < n; j++)
      std::cout << "\033[33m" << std::setw(widthsCols[j] + 2) << std::left
                << (matrix(i, j) == INT32_MAX ? "INF"
                                              : std::to_string(matrix(i, j)))
                << "\033[0m";
    std::cout << "\033[0m |" << std::endl;
  }

  for (int j = 0; j < allWidth + 2; j++) std::cout << '-';
  std::cout << std::endl;
}

/**
 * @brief Print vector of vertices
 * @param vec Vector of vertices
 * @param message Message for the user
 */
void UI::printVectorOfVertices(const std::vector<int> &vec,
                               const std::string &message) {
  std::cout << "\033[32m    " << message << " \033[0m"
            << "-- ";

  std::cout << "\033[33m";
  for (auto n : vec) std::cout << n << " ";
  std::cout << "\033[0m--" << std::endl;
}
}  // namespace s21