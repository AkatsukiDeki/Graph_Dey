#include <iostream>
#include "Graph.h"

int main() {
    // Создание графа
    Graph<std::string, double> graph;

    // Добавление вершин
    graph.add_vertex("A");
    graph.add_vertex("B");
    graph.add_vertex("C");
    graph.add_vertex("D");
    graph.add_vertex("E");

    // Добавление ребер
    graph.add_edge("A", "B", 5.0);
    graph.add_edge("A", "C", 3.0);
    graph.add_edge("B", "D", 2.0);
    graph.add_edge("B", "E", 4.0);
    graph.add_edge("C", "B", 1.0);
    graph.add_edge("C", "D", 4.0);
    graph.add_edge("D", "E", 3.0);

    // Вывод всех вершин
    cout << "Vertices: ";
    for (const auto& vertex : graph.vertices()) {
        cout << vertex._val << " ";
    }
    cout << endl;

    // Вывод всех ребер для вершины "A"
    cout << "Edges for vertex A: ";
    for (const auto& edge : graph.edges("A")) {
        cout << "(" << graph.vertices()[edge._num]._val << ", " << edge._val << ") ";
    }
    cout << endl;

    // Обход графа в глубину начиная с вершины "A"
    cout << "DFS starting from vertex A: ";
    graph.dfs("A");
    cout << endl;

    // Выполнение алгоритма Дейкстры для поиска кратчайшего пути от вершины "A"
    cout << "Shortest distances from vertex A:" << endl;
    graph.Dijkstra("A", true);

    return 0;
}
