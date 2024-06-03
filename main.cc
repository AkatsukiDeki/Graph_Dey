#include <iostream>
#include "Graph.h"

int main() {
    // Создание графа
    Graph<std::string, double> graph;

    // Добавление вершин
    std::cout << "Step 1: Adding vertices A, B, C, D, E" << std::endl;
    graph.add_vertex("A");
    graph.add_vertex("B");
    graph.add_vertex("C");
    graph.add_vertex("D");
    graph.add_vertex("E");
    graph.print(); // Вывод текущего состояния графа

    // Добавление ребер
    std::cout << "\nStep 2: Adding edges between vertices" << std::endl;
    graph.add_edge("A", "B", 5.0);
    graph.add_edge("A", "C", 3.0);
    graph.add_edge("B", "D", 2.0);
    graph.add_edge("B", "E", 4.0);
    graph.add_edge("C", "B", 1.0);
    graph.add_edge("C", "D", 4.0);
    graph.add_edge("D", "E", 3.0);
    graph.print(); // Вывод текущего состояния графа

    // Нахождение оптимальной точки для склада
    std::cout << "\nStep 3: Finding the optimal warehouse point" << std::endl;
    std::string optimal_warehouse = graph.find_optimal_warehouse();

    // Вывод результата
    std::cout << "Optimal warehouse point: " << optimal_warehouse << std::endl;

    return 0;
}
ё
