#ifndef S21_GRAPH_ALGORITHMS_H
#define S21_GRAPH_ALGORITHMS_H

#include "../Graph/s21_graph.h"
#include "../Components/Containers/Queue/s21_queue.h"
#include "../Components/Containers/Stack/s21_stack.h"

namespace s21{

class GraphAlgorithms {
 public:
  GraphAlgorithms();
  ~GraphAlgorithms() = default; 

  std::vector<int> DepthFirstSearch(Graph &graph, int start_vertex);
  std::vector<int> BreadthFirstSearch(Graph &graph, int start_vertex);
  
  //TsmResult SolveTravelingSalesmanProblem(Graph &graph);
};
}

#endif