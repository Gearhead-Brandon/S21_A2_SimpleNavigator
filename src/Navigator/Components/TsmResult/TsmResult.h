/**
 * @file TsmResult.h
 * @brief Header file describing the structt TsmResult (Traveling Salesman
 * Problem) and enum class TsmResultErrors
 */

#ifndef TSMRESULT_H
#define TSMRESULT_H

#include <vector>

namespace s21 {

/**
 * @enum TsmResultErrors
 * @brief Errors of the struct TsmResult
 */
enum class TsmResultErrors : int {
  GraphIsNotConnected = -1,
  ThereIsNoHamiltonCycle = -2
};

/**
 * @brief The struct TsmResult is used to check the result of the traveling
 * salesman problem
 */
struct TsmResult {
  //! Array with the desired route (with the order of traversal of the vertices)
  std::vector<int> vertices;

  //! Length of the route
  double distance;
};
}  // namespace s21

#endif