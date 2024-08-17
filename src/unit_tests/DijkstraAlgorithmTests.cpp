#include "../Navigator/GraphAlgorithms/s21_graph_algorithms.h"
#include "fstream"
#include "unit_tests.h"

using namespace s21;

class DijkstraAlgorithmTests : public testing::Test {
 protected:
  Graph graph_;
  GraphAlgorithms graph_algorithms_;

  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(DijkstraAlgorithmTests, DijkstraAlgorithmOnGraph5) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph5");

  std::size_t distance =
      graph_algorithms_.GetShortestPathBetweenVertices(graph_, 1, 4);

  // Assert
  ASSERT_EQ(distance, 12);

  // Act
  distance = graph_algorithms_.GetShortestPathBetweenVertices(graph_, 4, 2);

  // Assert
  ASSERT_EQ(distance, 22);

  // Act
  distance = graph_algorithms_.GetShortestPathBetweenVertices(graph_, 2, 5);

  // Assert
  ASSERT_EQ(distance, 14);
}

TEST_F(DijkstraAlgorithmTests, DijkstraAlgorithmOnGraph4) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph4");

  std::size_t distance =
      graph_algorithms_.GetShortestPathBetweenVertices(graph_, 3, 2);

  // Assert
  ASSERT_EQ(distance, 44);

  // Act
  distance = graph_algorithms_.GetShortestPathBetweenVertices(graph_, 1, 3);

  // Assert
  ASSERT_EQ(distance, 20);
}

TEST_F(DijkstraAlgorithmTests, DijkstraAlgorithmOnGraph4_) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph4_");

  std::size_t distance =
      graph_algorithms_.GetShortestPathBetweenVertices(graph_, 3, 1);

  // Assert
  ASSERT_EQ(distance, 59);

  // Act
  distance = graph_algorithms_.GetShortestPathBetweenVertices(graph_, 1, 3);

  // Assert
  ASSERT_EQ(distance, 20);
}

TEST_F(DijkstraAlgorithmTests, DijkstraAlgorithmOnGraph8) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph8");

  std::size_t distance =
      graph_algorithms_.GetShortestPathBetweenVertices(graph_, 3, 7);

  // Assert
  ASSERT_EQ(distance, 5);

  // Act
  distance = graph_algorithms_.GetShortestPathBetweenVertices(graph_, 4, 8);

  // Assert
  ASSERT_EQ(distance, 2);

  // Act
  distance = graph_algorithms_.GetShortestPathBetweenVertices(graph_, 1, 8);

  // Assert
  ASSERT_EQ(distance, 3);
}

TEST_F(DijkstraAlgorithmTests, DijkstraAlgorithmOnGraph20) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph20");

  std::size_t distance =
      graph_algorithms_.GetShortestPathBetweenVertices(graph_, 10, 18);

  // Assert
  ASSERT_EQ(distance, 10);

  // Act
  distance = graph_algorithms_.GetShortestPathBetweenVertices(graph_, 5, 18);

  // Assert
  ASSERT_EQ(distance, 9);

  // Act
  distance = graph_algorithms_.GetShortestPathBetweenVertices(graph_, 1, 20);

  // Assert
  ASSERT_EQ(distance, 7);
}

TEST_F(DijkstraAlgorithmTests, DijkstraAlgorithmOnUnconnectedGraph) {
  // Act
  graph_.LoadGraphFromFile("test_files/incoherent");

  std::size_t distance =
      graph_algorithms_.GetShortestPathBetweenVertices(graph_, 1, 2);

  // Assert
  ASSERT_EQ(distance, 1);

  // Act
  distance = graph_algorithms_.GetShortestPathBetweenVertices(graph_, 3, 4);

  // Assert
  ASSERT_EQ(distance, 1);

  // Act
  distance = graph_algorithms_.GetShortestPathBetweenVertices(graph_, 1, 24);

  // Assert
  ASSERT_EQ(distance, std::numeric_limits<std::size_t>::max());
}