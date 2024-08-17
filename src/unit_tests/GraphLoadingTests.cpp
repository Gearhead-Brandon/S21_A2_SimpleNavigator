#include "../Navigator/Graph/s21_graph.h"
#include "unit_tests.h"

using namespace s21;

class GraphLoadingTests : public testing::Test {
 protected:
  Graph graph_;

  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(GraphLoadingTests, LoadGraphFromFile) {
  // Act
  OpResult result = graph_.LoadGraphFromFile("test_files/graph4");

  // Assert
  ASSERT_TRUE(result.IsSuccess());

  S21Matrix<int> expected = {
      {0, 29, 20, 21}, {29, 0, 0, 29}, {20, 0, 0, 15}, {21, 29, 15, 0}};

  ASSERT_EQ(graph_.size(), expected.GetRows());

  for (int i = 0; i < expected.GetRows(); ++i)
    for (int j = 0; j < expected.GetCols(); ++j)
      ASSERT_EQ(graph_.getWeight(i, j), expected(i, j));
}

TEST_F(GraphLoadingTests, LoadGraphFromDirectory) {
  // Act
  OpResult result = graph_.LoadGraphFromFile("test_files");

  // Assert
  ASSERT_FALSE(result.IsSuccess());
  ASSERT_EQ(result.getErrorMessage(), "Is a directory, not a file");
}

TEST_F(GraphLoadingTests, FileNotFound) {
  // Act
  OpResult result = graph_.LoadGraphFromFile("test_files/file_not_found");

  // Assert
  ASSERT_FALSE(result.IsSuccess());
  ASSERT_EQ(result.getErrorMessage(), "File not found");
}

TEST_F(GraphLoadingTests, IncorrectSize) {
  // Act
  OpResult result = graph_.LoadGraphFromFile("test_files/incorrect_size_0");

  // Assert
  ASSERT_FALSE(result.IsSuccess());
  ASSERT_EQ(result.getErrorMessage(), "Incorrect matrix size");
}

TEST_F(GraphLoadingTests, IncorrectWeight) {
  // Act
  OpResult result = graph_.LoadGraphFromFile("test_files/incorrect_weight");

  // Assert
  ASSERT_FALSE(result.IsSuccess());
  ASSERT_EQ(result.getErrorMessage(), "Incorrect rib weight");

  // Act
  result = graph_.LoadGraphFromFile("test_files/incorrect_weight_letters");

  // Assert
  ASSERT_FALSE(result.IsSuccess());
  ASSERT_EQ(result.getErrorMessage(), "Incorrect rib weight");
}

TEST_F(GraphLoadingTests, IncorrectRowFormat) {
  // Act
  OpResult result = graph_.LoadGraphFromFile("test_files/incorrect_row_format");

  // Assert
  ASSERT_FALSE(result.IsSuccess());
  ASSERT_EQ(result.getErrorMessage(), "Incorrect row format");
}