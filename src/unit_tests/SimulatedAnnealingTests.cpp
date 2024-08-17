#include "../Navigator/GraphAlgorithms/s21_graph_algorithms.h"
#include "fstream"
#include "unit_tests.h"

using namespace s21;

class SimulatedAnnealingAlgorithmTests : public testing::Test {
 protected:
  Graph graph_;
  GraphAlgorithms graph_algorithms_;

  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(SimulatedAnnealingAlgorithmTests, SimulatedAnnealingAlgorithmOnGraph5) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph5");

  TsmResult result =
      graph_algorithms_.SolveTravelingSalesmanProblemThroughSimulatedAnnealing(
          graph_);

  // Assert
  ASSERT_EQ(result.distance, 40);
  ASSERT_EQ(result.vertices.front(), result.vertices.back());
}

TEST_F(SimulatedAnnealingAlgorithmTests, SimulatedAnnealingAlgorithmOnGraph4) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph4");

  TsmResult result =
      graph_algorithms_.SolveTravelingSalesmanProblemThroughSimulatedAnnealing(
          graph_);

  // Assert
  ASSERT_EQ(result.distance, 93);
  ASSERT_EQ(result.vertices.front(), result.vertices.back());
}

TEST_F(SimulatedAnnealingAlgorithmTests, SimulatedAnnealingAlgorithmOnGraph4_) {
  // Act
  graph_.LoadGraphFromFile("test_files/graph4_");

  TsmResult result =
      graph_algorithms_.SolveTravelingSalesmanProblemThroughSimulatedAnnealing(
          graph_);

  // Assert
  ASSERT_EQ(result.distance,
            static_cast<double>(TsmResultErrors::ThereIsNoHamiltonCycle));
}

TEST_F(SimulatedAnnealingAlgorithmTests,
       SimulatedAnnealingAlgorithmOnUnconnectedGraph) {
  // Act
  graph_.LoadGraphFromFile("test_files/incoherent");

  TsmResult result =
      graph_algorithms_.SolveTravelingSalesmanProblemThroughSimulatedAnnealing(
          graph_);

  // Assert
  ASSERT_EQ(result.distance,
            static_cast<double>(TsmResultErrors::GraphIsNotConnected));
}