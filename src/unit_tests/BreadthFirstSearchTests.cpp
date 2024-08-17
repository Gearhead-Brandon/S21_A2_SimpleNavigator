#include "../Navigator/GraphAlgorithms/s21_graph_algorithms.h"
#include "fstream"
#include "unit_tests.h"

using namespace s21;

class BreadthFirstSearchTests : public testing::Test {
 protected:
  Graph graph_;
  GraphAlgorithms graph_algorithms_;

  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(BreadthFirstSearchTests, BreadthFirstSearchOnGraph5) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph5");

  std::vector<int> path = graph_algorithms_.BreadthFirstSearch(graph_, 1);

  // Assert
  std::vector<int> expected = {1, 2, 3, 4, 5};

  ASSERT_EQ(path.size(), expected.size());
  ASSERT_EQ(path, expected);
}

TEST_F(BreadthFirstSearchTests, BreadthFirstSearchOnGraph4_) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph4_");

  std::vector<int> path = graph_algorithms_.BreadthFirstSearch(graph_, 1);

  // Assert
  std::vector<int> expected = {1, 2, 3, 4};

  ASSERT_EQ(path.size(), expected.size());
  ASSERT_EQ(path, expected);
}

TEST_F(BreadthFirstSearchTests, BreadthFirstSearchOnGraph8) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph8");

  std::vector<int> path = graph_algorithms_.BreadthFirstSearch(graph_, 3);

  // Assert
  std::vector<int> expected = {3, 4, 6, 8, 1, 5, 2, 7};

  ASSERT_EQ(path.size(), expected.size());
  ASSERT_EQ(path, expected);
}

TEST_F(BreadthFirstSearchTests, BreadthFirstSearchOnGraph20) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph20");

  std::vector<int> path = graph_algorithms_.BreadthFirstSearch(graph_, 1);

  // Assert
  std::vector<int> expected = {1,  2,  3,  4,  5,  6,  7,  8,  9,  10,
                               11, 12, 13, 14, 15, 16, 17, 18, 19, 20};

  ASSERT_EQ(path.size(), expected.size());
  ASSERT_EQ(path, expected);
}

TEST_F(BreadthFirstSearchTests, BreadthFirstSearchOnUnconnectedGraph) {
  // Act
  graph_.LoadGraphFromFile("test_files/incoherent");

  std::vector<int> path = graph_algorithms_.BreadthFirstSearch(graph_, 1);

  // Assert
  std::vector<int> expected = {1, 2};

  ASSERT_EQ(path.size(), expected.size());
  ASSERT_EQ(path, expected);

  // Act
  path = std::move(graph_algorithms_.BreadthFirstSearch(graph_, 3));

  // Assert
  expected = {3, 4};

  ASSERT_EQ(path.size(), expected.size());
  ASSERT_EQ(path, expected);
}