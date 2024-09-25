#include <iostream>
#include <locale> 
#include "Graph.h"

int main() {

    setlocale(LC_ALL, "ru_RU.UTF-8");

    // Создание графа
    Graph<std::string, double> graph;

    // Добавление вершин
    std::cout << "Шаг 1: Добавление вершин A, B, C, D, E" << std::endl;
    graph.add_vertex("A");
    graph.add_vertex("B");
    graph.add_vertex("C");
    graph.add_vertex("D");
    graph.add_vertex("E");

    // Вывод текущего состояния графа
    graph.print();

    // Добавление ребер
    std::cout << "\nШаг 2: Добавление ребер между вершинами" << std::endl;
    graph.add_edge("A", "B", 5.0);
    graph.add_edge("A", "C", 3.0);
    graph.add_edge("B", "D", 2.0);
    graph.add_edge("B", "E", 4.0);
    graph.add_edge("C", "B", 1.0);
    graph.add_edge("C", "D", 4.0);
    graph.add_edge("D", "E", 3.0);

    // Проверка на наличие отрицательных весов
    if (graph.has_negative_weights()) {
        std::cerr << "Ошибка: Граф содержит отрицательные веса!" << std::endl;
        return 1; // Завершение программы с ошибкой
    }

    // Нахождение оптимальной точки для склада
    std::cout << "\nШаг 3: Поиск оптимальной точки для склада" << std::endl;
    std::string optimal_warehouse = graph.find_optimal_warehouse();
    std::cout << "Оптимальная точка склада: " << optimal_warehouse << std::endl;

    // Дополнительно: Вывод кратчайших путей от одной из вершин
    std::cout << "\nШаг 4: Запуск алгоритма Дейкстры от вершины A:" << std::endl;
    graph.Dijkstra("A", true); // Вывод кратчайших расстояний от A

    return 0;
}
