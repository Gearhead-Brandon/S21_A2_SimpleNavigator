#include "../Navigator/GraphAlgorithms/s21_graph_algorithms.h"
#include "fstream"
#include "unit_tests.h"

using namespace s21;

class FloydWarshallAlgorithmTests : public testing::Test {
 protected:
  Graph graph_;
  GraphAlgorithms graph_algorithms_;

  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(FloydWarshallAlgorithmTests, FloydWarshallAlgorithmOnGraph5) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph5");

  S21Matrix<int> matrix =
      graph_algorithms_.GetShortestPathsBetweenAllVertices(graph_);

  // Assert
  S21Matrix<int> expected = {{0, 2, 10, 12, 16},
                             {20, 0, 10, 10, 14},
                             {10, 12, 0, 15, 14},
                             {20, 22, 15, 0, 4},
                             {16, 18, 14, 4, 0}};

  ASSERT_EQ(matrix, expected);
}

TEST_F(FloydWarshallAlgorithmTests, FloydWarshallAlgorithmOnGraph4) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph4");

  S21Matrix<int> matrix =
      graph_algorithms_.GetShortestPathsBetweenAllVertices(graph_);

  // Assert
  S21Matrix<int> expected = {
      {0, 29, 20, 21}, {29, 0, 44, 29}, {20, 44, 0, 15}, {21, 29, 15, 0}};

  ASSERT_EQ(matrix, expected);
}

TEST_F(FloydWarshallAlgorithmTests, FloydWarshallAlgorithmOnGraph4_) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph4_");

  S21Matrix<int> matrix =
      graph_algorithms_.GetShortestPathsBetweenAllVertices(graph_);

  // Assert
  S21Matrix<int> expected = {
      {0, 10, 20, 39}, {29, 0, 10, 29}, {59, 30, 0, 59}, {58, 29, 39, 0}};

  ASSERT_EQ(matrix, expected);
}

TEST_F(FloydWarshallAlgorithmTests, FloydWarshallAlgorithmOnGraph8) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph8");

  S21Matrix<int> matrix =
      graph_algorithms_.GetShortestPathsBetweenAllVertices(graph_);

  // Assert
  S21Matrix<int> expected = {
      {0, 2, 2, 3, 1, 1, 3, 3}, {2, 0, 4, 5, 1, 3, 1, 5},
      {2, 4, 0, 1, 3, 1, 5, 1}, {3, 5, 1, 0, 4, 2, 6, 2},
      {1, 1, 3, 4, 0, 2, 2, 4}, {1, 3, 1, 2, 2, 0, 4, 2},
      {3, 1, 5, 6, 2, 4, 0, 6}, {3, 5, 1, 2, 4, 2, 6, 0}};

  ASSERT_EQ(matrix, expected);
}

TEST_F(FloydWarshallAlgorithmTests, FloydWarshallAlgorithmOnGraph11) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph11");

  S21Matrix<int> matrix =
      graph_algorithms_.GetShortestPathsBetweenAllVertices(graph_);

  // Assert
  S21Matrix<int> expected = {{0, 2, 10, 12, 16, 17, 74, 12, 4, 14, 14},
                             {20, 0, 10, 10, 14, 22, 72, 19, 24, 22, 12},
                             {10, 12, 0, 15, 14, 25, 81, 9, 14, 23, 13},
                             {20, 22, 15, 0, 4, 12, 92, 12, 24, 13, 23},
                             {16, 18, 14, 4, 0, 16, 90, 9, 20, 16, 22},
                             {31, 25, 25, 12, 16, 0, 95, 24, 31, 3, 13},
                             {91, 72, 81, 82, 86, 94, 0, 90, 95, 94, 84},
                             {12, 14, 9, 12, 9, 24, 86, 0, 15, 23, 13},
                             {4, 6, 14, 16, 20, 13, 78, 15, 0, 10, 18},
                             {28, 22, 23, 13, 16, 3, 94, 23, 28, 0, 10},
                             {18, 12, 13, 22, 22, 13, 84, 13, 18, 10, 0}};

  ASSERT_EQ(matrix, expected);
}

TEST_F(FloydWarshallAlgorithmTests, FloydWarshallAlgorithmOnUnconnectedGraph) {
  // Act
  graph_.LoadGraphFromFile("test_files/incoherent");

  S21Matrix<int> matrix =
      graph_algorithms_.GetShortestPathsBetweenAllVertices(graph_);

  // Assert
  S21Matrix<int> expected = {{0, 1, 2147483647, 2147483647},
                             {1, 0, 2147483647, 2147483647},
                             {2147483647, 2147483647, 0, 1},
                             {2147483647, 2147483647, 1, 0}};

  ASSERT_EQ(matrix, expected);
}