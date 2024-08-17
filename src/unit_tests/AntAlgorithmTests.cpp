#include "../Navigator/GraphAlgorithms/s21_graph_algorithms.h"
#include "fstream"
#include "unit_tests.h"

using namespace s21;

class AntAlgorithmTests : public testing::Test {
 protected:
  Graph graph_;
  GraphAlgorithms graph_algorithms_;

  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(AntAlgorithmTests, AntAlgorithmOnGraph5) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph5");

  TsmResult result = graph_algorithms_.SolveTravelingSalesmanProblem(graph_);

  // Assert
  ASSERT_EQ(result.distance, 40);
  ASSERT_EQ(result.vertices.front(), result.vertices.back());
}

TEST_F(AntAlgorithmTests, AntAlgorithmOnGraph4) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph4");

  TsmResult result = graph_algorithms_.SolveTravelingSalesmanProblem(graph_);

  // Assert
  ASSERT_EQ(result.distance, 93);
  ASSERT_EQ(result.vertices.front(), result.vertices.back());
}

TEST_F(AntAlgorithmTests, AntAlgorithmOnGraph4_) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph4_");

  TsmResult result = graph_algorithms_.SolveTravelingSalesmanProblem(graph_);

  // Assert
  ASSERT_EQ(result.distance,
            static_cast<double>(TsmResultErrors::ThereIsNoHamiltonCycle));
}

TEST_F(AntAlgorithmTests, AntAlgorithmOnUnconnectedGraph) {
  // Act
  graph_.LoadGraphFromFile("test_files/incoherent");

  TsmResult result = graph_algorithms_.SolveTravelingSalesmanProblem(graph_);

  // Assert
  ASSERT_EQ(result.distance,
            static_cast<double>(TsmResultErrors::GraphIsNotConnected));
}