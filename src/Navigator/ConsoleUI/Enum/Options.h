/**
 * @file Options.h
 * @brief Header file describing the enum class Options
 */

#ifndef OPTIONS_H
#define OPTIONS_H

/**
 * @brief Enum class Options, which is used to check the desired operation
 *        selected by the user to execute an algorithm on a graph
 */
enum class Options : int {
  LoadGraph,  //! Load graph from file
  SaveGraph,  //! Save graph to file in .dot format
  BFS,        //! Breadth-first traversal algorithm
  DFS,        //! Depth-first traversal algorithm
  Dejkstra,   //! Dejkstra's algorithm for finding the shortest path between two
              //! vertices
  FloydWarshall,  //! Floyd-Warshall algorithm for finding the shortest paths
                  //! between all pairs of vertices
  SpanningTree,   //! Finding the minimum spanning tree in a graph
  Tsm,            //! Solving the traveling salesman problem
  Exit            //! Exit the program
};

#endif
