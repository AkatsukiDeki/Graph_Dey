#include "graph.h"
#include <iostream>
#include <string>

int main() {
    Graph<std::string> g;

    g.add_vertex("A");
    g.add_vertex("B");
    g.add_vertex("C");
    g.add_vertex("D");

    g.add_edge("A", "B", 5.0);
    g.add_edge("A", "C", 3.0);
    g.add_edge("B", "D", 2.0);
    g.add_edge("C", "D", 1.0);

    std::cout << "Vertices: ";
    for (const auto& v : g.vertices()) {
        std::cout << v << " ";
    }
    std::cout << std::endl;

    std::cout << "Edges: ";
    for (const auto& e : g.edges("A")) {
        std::cout << "(" << e.from << ", " << e.to << ", " << e.weight << ") ";
    }
    std::cout << std::endl;

    std::cout << "Shortest path from A to D: ";
    for (const auto& e : g.shortest_path("A", "D")) {
        std::cout << "(" << e.from << ", " << e.to << ", " << e.weight << ") ";
    }
    std::cout << std::endl;

    std::cout << "Depth-First Search (DFS) traversal: ";
    for (const auto& v : g.dfs("A")) {
        std::cout << v << " ";
    }
    std::cout << std::endl;

    std::cout << "Dijkstra's shortest path from A to D: ";
    for (const auto& e : g.dijkstra_shortest_path("A", "D")) {
        std::cout << "(" << e.from << ", " << e.to << ", " << e.weight << ") ";
    }
    std::cout << std::endl;

    return 0;
}
