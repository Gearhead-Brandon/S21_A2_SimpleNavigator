#include "../Navigator/GraphAlgorithms/s21_graph_algorithms.h"
#include "fstream"
#include "unit_tests.h"

using namespace s21;

class PrimsAlgorithmTests : public testing::Test {
 protected:
  Graph graph_;
  GraphAlgorithms graph_algorithms_;

  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(PrimsAlgorithmTests, PrimsAlgorithmOnGraph5) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph5");

  S21Matrix<int> matrix = graph_algorithms_.GetLeastSpanningTree(graph_);

  // Assert
  S21Matrix<int> expected = {{0, 2, 10, 0, 0},
                             {0, 0, 0, 10, 0},
                             {10, 0, 0, 0, 0},
                             {0, 0, 0, 0, 4},
                             {0, 0, 0, 4, 0}};

  ASSERT_EQ(matrix, expected);
}

TEST_F(PrimsAlgorithmTests, PrimsAlgorithmOnGraph4) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph4");

  S21Matrix<int> matrix = graph_algorithms_.GetLeastSpanningTree(graph_);

  // Assert
  S21Matrix<int> expected = {
      {0, 29, 20, 0}, {29, 0, 0, 0}, {20, 0, 0, 15}, {0, 0, 15, 0}};

  ASSERT_EQ(matrix, expected);
}

TEST_F(PrimsAlgorithmTests, PrimsAlgorithmOnGraph4_) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph4_");

  S21Matrix<int> matrix = graph_algorithms_.GetLeastSpanningTree(graph_);

  // Assert
  S21Matrix<int> expected = {
      {0, 10, 0, 0}, {0, 0, 10, 29}, {0, 0, 0, 0}, {0, 29, 0, 0}};

  ASSERT_EQ(matrix, expected);
}

TEST_F(PrimsAlgorithmTests, PrimsAlgorithmOnGraph8) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph8");

  S21Matrix<int> matrix = graph_algorithms_.GetLeastSpanningTree(graph_);

  // Assert
  S21Matrix<int> expected = {
      {0, 0, 0, 0, 1, 1, 0, 0}, {0, 0, 0, 0, 1, 0, 1, 0},
      {0, 0, 0, 1, 0, 1, 0, 1}, {0, 0, 1, 0, 0, 0, 0, 0},
      {1, 1, 0, 0, 0, 0, 0, 0}, {1, 0, 1, 0, 0, 0, 0, 0},
      {0, 1, 0, 0, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0, 0, 0}};

  ASSERT_EQ(matrix, expected);
}

TEST_F(PrimsAlgorithmTests, PrimsAlgorithmOnGraph11) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph11");

  S21Matrix<int> matrix = graph_algorithms_.GetLeastSpanningTree(graph_);

  // Assert
  S21Matrix<int> expected = {
      {0, 2, 10, 0, 0, 0, 0, 0, 4, 0, 0}, {0, 0, 0, 0, 0, 0, 72, 0, 0, 0, 0},
      {10, 0, 0, 0, 0, 0, 0, 9, 0, 0, 0}, {0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 4, 0, 0, 0, 9, 0, 0, 0},  {0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0},
      {0, 72, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 9, 0, 9, 0, 0, 0, 0, 0, 0},
      {4, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0}, {0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 10},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0}};

  ASSERT_EQ(matrix, expected);
}

TEST_F(PrimsAlgorithmTests, PrimsAlgorithmOnGraphOnUnconnectedGraph) {
  // Act
  graph_.LoadGraphFromFile("test_files/incoherent");

  S21Matrix<int> matrix = graph_algorithms_.GetLeastSpanningTree(graph_);

  // Assert
  ASSERT_EQ(matrix.GetRows(), 0);
  ASSERT_EQ(matrix.GetCols(), 0);
}