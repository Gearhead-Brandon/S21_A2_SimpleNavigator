#include "s21_graph_algorithms.h"

namespace s21 {
    GraphAlgorithms::GraphAlgorithms() {}

    auto GraphAlgorithms::DepthFirstSearch(Graph &graph, int start_vertex)
        -> std::vector<int> {
        --start_vertex;

        // Array to keep track of visited vertices
        std::vector<bool> visited(graph.size(), false);
        // Add the starting vertex to the stack and mark it as visited
        visited[start_vertex] = true;

        // Stack to store the vertices
        // in the order in which they were visited
        s21::stack<int> s;
        // Add the starting vertex to the stack
        s.push(start_vertex);

        // Vector to store the path
        // in the order in which it was visited
        std::vector<int> path;

        while (!s.empty()) {
            int v = s.top();
            s.pop();

            path.push_back(v + 1);

            for (int neighbor : graph.getAdjacentVertices(v)) {
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    s.push(neighbor);
                }
            }
        }

    return path;
    }

    auto GraphAlgorithms::BreadthFirstSearch(Graph &graph, int start_vertex)
        -> std::vector<int> {
        --start_vertex;

        // Array to keep track of visited vertices
        std::vector<bool> visited(graph.size(), false);
        // Add the starting vertex to the queue and mark it as visited
        visited[start_vertex] = true;
        
        // Queue to store the vertices
        // in the order in which they were visited
        s21::queue<int> q;

        // Add the starting vertex to the queue
        q.push(start_vertex);

        // Vector to store the path
        // in the order in which it was visited
        std::vector<int> path{};

        while (!q.empty()) {
            int v = q.front();
            q.pop();

            path.push_back(v + 1);

            for(int neighbor : graph.getAdjacentVertices(v)) {
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    q.push(neighbor);
                }
            }
        }

        return path;
    } 
}