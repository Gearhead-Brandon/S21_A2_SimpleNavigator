#include "../Navigator/GraphAlgorithms/s21_graph_algorithms.h"
#include "fstream"
#include "unit_tests.h"

using namespace s21;

class DepthFirstSearchTests : public testing::Test {
 protected:
  Graph graph_;
  GraphAlgorithms graph_algorithms_;

  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(DepthFirstSearchTests, DepthFirstSearchOnGraph5) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph5");

  std::vector<int> path = graph_algorithms_.DepthFirstSearch(graph_, 1);

  // Assert
  std::vector<int> expected = {1, 5, 4, 3, 2};

  ASSERT_EQ(path.size(), expected.size());
  ASSERT_EQ(path, expected);
}

TEST_F(DepthFirstSearchTests, DepthFirstSearchOnGraph4_) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph4_");

  std::vector<int> path = graph_algorithms_.DepthFirstSearch(graph_, 1);

  // Assert
  std::vector<int> expected = {1, 2, 4, 3};

  ASSERT_EQ(path.size(), expected.size());
  ASSERT_EQ(path, expected);
}

TEST_F(DepthFirstSearchTests, DepthFirstSearchOnGraph8) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph8");

  std::vector<int> path = graph_algorithms_.DepthFirstSearch(graph_, 3);

  // Assert
  std::vector<int> expected = {3, 8, 6, 1, 5, 2, 7, 4};

  ASSERT_EQ(path.size(), expected.size());
  ASSERT_EQ(path, expected);
}

TEST_F(DepthFirstSearchTests, DepthFirstSearchOnGraph20) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph20");

  std::vector<int> path = graph_algorithms_.DepthFirstSearch(graph_, 1);

  // Assert
  std::vector<int> expected = {1,  20, 19, 18, 17, 16, 15, 14, 13, 12,
                               11, 10, 9,  8,  7,  6,  5,  4,  3,  2};

  ASSERT_EQ(path.size(), expected.size());
  ASSERT_EQ(path, expected);
}

TEST_F(DepthFirstSearchTests, DepthFirstSearchOnUnconnectedGraph) {
  // Act
  graph_.LoadGraphFromFile("test_files/incoherent");

  std::vector<int> path = graph_algorithms_.DepthFirstSearch(graph_, 1);

  // Assert
  std::vector<int> expected = {1, 2};

  ASSERT_EQ(path.size(), expected.size());
  ASSERT_EQ(path, expected);

  // Act
  path = std::move(graph_algorithms_.DepthFirstSearch(graph_, 3));

  // Assert
  expected = {3, 4};

  ASSERT_EQ(path.size(), expected.size());
  ASSERT_EQ(path, expected);
}