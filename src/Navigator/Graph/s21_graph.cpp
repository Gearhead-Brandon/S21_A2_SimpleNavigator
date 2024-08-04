#include "s21_graph.h"
#include "../Components/FileReader/FileReader.h"
#include "sstream"
#include <filesystem>

namespace s21 {

    Graph::Graph() : adjacencyMatrix_() {}

    void Graph::reset(){
        adjacencyMatrix_.Resize(0, 0);
    }

    OpResult Graph::LoadGraphFromFile(std::string filename){
        FileReader reader(filename);

        // std::filesystem::path currentPath = std::filesystem::current_path();
        // std::cout << "Current path: " << currentPath << std::endl;

        if(!reader.IsFileOpen())
            return OpResult(false, "File not found");

        int size = 0;

        reader.file >> size;

        if (size <= 0 || size <= 0)
            return {false, "Incorrect matrix size"};

        reader.file >> std::ws;

        adjacencyMatrix_.Resize(size, size);
        adjacencyMatrix_.Fill(0);

        std::string line("");

        for (int i = 0; i < size; ++i){
            std::getline(reader.file, line);
            std::istringstream iss(line);
            for (int j = 0; j < size; ++j){
                int value = 0;

                iss >> value;

                if (iss.fail()){
                    reset();
                    return {false, "Incorrect number of columns"};
                }

                if (value < 0){
                    reset();
                    return {false, "Incorrect rib weight"};
                }

                adjacencyMatrix_(i, j) = value;
            }

            if(iss.rdbuf()->in_avail() > 0){
                reset();
                return {false, "Incorrect number of columns"};
            }
        }

        std::cout << std::endl << "Adjacency matrix: " << adjacencyMatrix_.GetRows() << std::endl;
        adjacencyMatrix_.Print();

        return {true, ""};
    }

    void Graph::ExportGraphToDot(std::string filename){
        int size = adjacencyMatrix_.GetRows();

        if (size <= 0)
            return;

        std::ofstream file;
        file.open(filename + (filename.substr(filename.find_last_of(".")) == ".dot" ? "" : ".dot"));

        file << "graph graphname {\n";

        for(int i = 0; i < size; ++i)
            file << "\t" << i + 1 << ";\n";

        for (int i = 0; i < size; ++i) {
            // Check for self-loop (loop handling)
            if (adjacencyMatrix_(i, i) != 0) 
                file << "\t" << i + 1 << " -- " << i + 1 << ";\n";

            for(int j = 0; j < i+1; ++j)
                if (adjacencyMatrix_(i, j) != 0)
                    if (adjacencyMatrix_(j, i) == 0)
                        file << "\t" << i + 1 << " -> " << j + 1 << ";\n";
            
            // Iterate through other nodes to find edges
            for (int j = i + 1; j < size; ++j)
                if (adjacencyMatrix_(i, j) != 0)
                    if (adjacencyMatrix_(j, i) == 0)
                        file << "\t" << i + 1 << " -> " << j + 1 << ";\n";
                    else
                        file << "\t" << i + 1 << " -- " << j + 1 << ";\n";
        }

        file << "}";

        file.close();

        // for (int i = 0; i < size; ++i)
        //     for (int j = i + 1; j < size; ++j)
        //         if (adjacencyMatrix_(i, j) != 0)
        //             file << "\t" << i + 1 << " -- " << j + 1 << ";\n";
    }

    // int Graph::operator()(int i, int j) const{
    //     return adjacencyMatrix_(i, j);
    // }

    std::vector<int> Graph::getAdjacentVertices(int start_vertex){
        std::vector<int> adjacent_vertices;
        for (int i = 0; i < adjacencyMatrix_.GetRows(); ++i)
            if (adjacencyMatrix_(start_vertex, i) != 0 && i != start_vertex)
                adjacent_vertices.push_back(i);

        return adjacent_vertices;
    }

    int Graph::size() const{
        return adjacencyMatrix_.GetRows();
    }
}

// 4
// 0  29  0  0
// 29 0   10 29
// 0  30  0  0
// 0  29  0  0