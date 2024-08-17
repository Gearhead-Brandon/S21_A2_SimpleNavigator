#include "../Navigator/Graph/s21_graph.h"
#include "fstream"
#include "unit_tests.h"

using namespace s21;

class ExportGraphToDotTests : public testing::Test {
 protected:
  Graph graph_;

  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(ExportGraphToDotTests, ExportGraphToDot) {
  // Act
  OpResult result = graph_.LoadGraphFromFile("test_files/graph4");
  graph_.ExportGraphToDot("test_files/exported_graph4");

  // Assert
  ASSERT_TRUE(result.IsSuccess());

  std::ifstream file("test_files/exported_graph4.dot");
  std::string content;
  std::stringstream ss;

  if (file.is_open())
    while (std::getline(file, content)) ss << content;
  else
    std::cerr << "\033[31mUnable to open file.\033[0m" << std::endl;

  content = std::move(ss.str());

  std::replace_if(
      content.begin(), content.end(), [](char c) { return c == '\t'; }, ' ');

  std::string expectedGraph =
      "graph graphname { 1; 2; 3; 4; 1 -- 2; 1 -- 3; 1 -- 4; 2 -- 4; 3 -- 4;}";

  ASSERT_EQ(content.length(), expectedGraph.length());
  ASSERT_EQ(content, expectedGraph);
}

TEST_F(ExportGraphToDotTests, ExportHalfOrientedGraphToDot) {
  // Act
  OpResult result = graph_.LoadGraphFromFile("test_files/graph4__");
  graph_.ExportGraphToDot("test_files/exported_graph4__");

  std::cout << result.getErrorMessage();

  // Assert
  ASSERT_TRUE(result.IsSuccess());

  std::ifstream file("test_files/exported_graph4__.dot");
  std::string content;
  std::stringstream ss;

  if (file.is_open())
    while (std::getline(file, content)) ss << content;
  else
    std::cerr << "\033[31mUnable to open file.\033[0m" << std::endl;

  content = std::move(ss.str());

  std::replace_if(
      content.begin(), content.end(), [](char c) { return c == '\t'; }, ' ');

  std::string expectedGraph =
      "graph graphname { 1; 2; 3; 4; 1 -- 3; 1 -- 4; 2 -> 1; 2 -- 4; 3 -- 4;}";

  ASSERT_EQ(content.length(), expectedGraph.length());
  ASSERT_EQ(content, expectedGraph);
}

TEST_F(ExportGraphToDotTests, ExportOrientedGraphToDot) {
  // Act
  OpResult result = graph_.LoadGraphFromFile("test_files/oriented_graph4");
  graph_.ExportGraphToDot("test_files/exported_oriented_graph4");

  std::cout << result.getErrorMessage();

  // Assert
  ASSERT_TRUE(result.IsSuccess());

  std::ifstream file("test_files/exported_oriented_graph4.dot");
  std::string content;
  std::stringstream ss;

  if (file.is_open())
    while (std::getline(file, content)) ss << content;
  else
    std::cerr << "\033[31mUnable to open file.\033[0m" << std::endl;

  content = std::move(ss.str());

  std::replace_if(
      content.begin(), content.end(), [](char c) { return c == '\t'; }, ' ');

  std::string expectedGraph =
      "graph graphname { 1; 2; 3; 4; 1 -> 2; 2 -> 3; 2 -> 4;}";

  ASSERT_EQ(content.length(), expectedGraph.length());
  ASSERT_EQ(content, expectedGraph);
}

TEST_F(ExportGraphToDotTests, ExportLoopGraphToDot) {
  // Act
  OpResult result = graph_.LoadGraphFromFile("test_files/loop_graph4");
  graph_.ExportGraphToDot("test_files/exported_loop_graph4");

  std::cout << result.getErrorMessage();

  // Assert
  ASSERT_TRUE(result.IsSuccess());

  std::ifstream file("test_files/exported_loop_graph4.dot");
  std::string content;
  std::stringstream ss;

  if (file.is_open())
    while (std::getline(file, content)) ss << content;
  else
    std::cerr << "\033[31mUnable to open file.\033[0m" << std::endl;

  content = std::move(ss.str());

  std::replace_if(
      content.begin(), content.end(), [](char c) { return c == '\t'; }, ' ');

  std::string expectedGraph =
      "graph graphname { 1; 2; 3; 4; 1 -- 1; 1 -> 2; 2 -> 3; 2 -> 4;}";

  ASSERT_EQ(content.length(), expectedGraph.length());
  ASSERT_EQ(content, expectedGraph);
}

TEST_F(ExportGraphToDotTests, IncorrectSizeOrFilename) {
  // Act
  graph_.ExportGraphToDot("test_files/exported_graph4_2");

  // Assert
  std::ifstream file("test_files/exported_graph4_2.dot");

  ASSERT_FALSE(file.is_open());

  // Act
  graph_.ExportGraphToDot("");

  // Assert
  file.open("test_files/.dot", std::ios::in);

  ASSERT_FALSE(file.is_open());
}