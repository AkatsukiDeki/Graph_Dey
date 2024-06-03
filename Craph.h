#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <stack>
#include <algorithm>
#include <memory>
#include <limits>

template<typename Vertex, typename Distance = double>
class Graph {
public:
    struct Edge {
        Vertex from;
        Vertex to;
        Distance weight;

        Edge(const Vertex& _from, const Vertex& _to, const Distance& _weight)
            : from(_from), to(_to), weight(_weight) {}

        bool operator==(const Edge& other) const {
            return from == other.from && to == other.to && weight == other.weight;
        }
    };

    // Проверка-добавление-удаление вершин
    bool has_vertex(const Vertex& v) const {
        return vertices_.find(v) != vertices_.end();
    }

    void add_vertex(const Vertex& v) {
        if (!has_vertex(v)) {
            vertices_.insert(v);
        }
    }

    bool remove_vertex(const Vertex& v) {
        if (!has_vertex(v)) {
            return false;
        }

        for (const auto& edge : edges_) {
            if (edge.from == v || edge.to == v) {
                edges_.erase(edge);
            }
        }

        vertices_.erase(v);
        return true;
    }

    std::vector<Vertex> vertices() const {
        return std::vector<Vertex>(vertices_.begin(), vertices_.end());
    }

    // Проверка-добавление-удаление ребер
    void add_edge(const Vertex& from, const Vertex& to, const Distance& d) {
        add_vertex(from);
        add_vertex(to);
        edges_.emplace(from, to, d);
    }

    bool remove_edge(const Vertex& from, const Vertex& to) {
        auto it = std::find_if(edges_.begin(), edges_.end(), [&](const Edge& e) {
            return e.from == from && e.to == to;
        });
        if (it != edges_.end()) {
            edges_.erase(it);
            return true;
        }
        return false;
    }

    bool remove_edge(const Edge& e) {
        auto it = std::find(edges_.begin(), edges_.end(), e);
        if (it != edges_.end()) {
            edges_.erase(it);
            return true;
        }
        return false;
    }

    bool has_edge(const Vertex& from, const Vertex& to) const {
        return std::find_if(edges_.begin(), edges_.end(), [&](const Edge& e) {
            return e.from == from && e.to == to;
        }) != edges_.end();
    }

    bool has_edge(const Edge& e) const {
        return std::find(edges_.begin(), edges_.end(), e) != edges_.end();
    }

    std::vector<Edge> edges(const Vertex& vertex) {
        std::vector<Edge> result;
        for (const auto& edge : edges_) {
            if (edge.from == vertex) {
                result.push_back(edge);
            }
        }
        return result;
    }

    size_t order() const {
        return vertices_.size();
    }

    size_t degree(const Vertex& v) const {
        size_t count = 0;
        for (const auto& edge : edges_) {
            if (edge.from == v || edge.to == v) {
                ++count;
            }
        }
        return count;
    }

    // Поиск кратчайшего пути
    std::vector<Edge> shortest_path(const Vertex& from, const Vertex& to) const {
        std::unordered_map<Vertex, std::unique_ptr<Edge>> prev;
        std::priority_queue<std::pair<Distance, Vertex>> pq;

        prev[from] = nullptr;
        pq.emplace(0.0, from);

        while (!pq.empty()) {
            auto [dist, curr] = pq.top();
            pq.pop();

            if (curr == to) {
                std::vector<Edge> path;
                for (auto v = to; prev[v]; v = prev[v]->from) {
                    path.emplace_back(prev[v]->from, prev[v]->to, prev[v]->weight);
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            for (const auto& edge : edges_) {
                if (edge.from == curr) {
                    auto new_dist = -dist + edge.weight;
                    if (prev.count(edge.to) == 0 || new_dist < -prev[edge.to]->weight) {
                        prev[edge.to] = std::make_unique<Edge>(edge);
                        pq.emplace(-new_dist, edge.to);
                    }
                }
            }
        }

        return {};
    }

    // Обход в глубину
    std::vector<Vertex> dfs(const Vertex& start_vertex) const {
        std::vector<Vertex> result;
        std::unordered_set<Vertex> visited;
        std::stack<Vertex> stack;

        stack.push(start_vertex);
        visited.insert(start_vertex);
        result.push_back(start_vertex);

        while (!stack.empty()) {
            auto curr = stack.top();
            stack.pop();

            for (const auto& edge : edges_) {
                if (edge.from == curr && visited.count(edge.to) == 0) {
                    stack.push(edge.to);
                    visited.insert(edge.to);
                    result.push_back(edge.to);
                }
            }
        }

        return result;
    }

    // Поиск кратчайшего пути по алгоритму Дейкстры
    std::vector<Edge> dijkstra_shortest_path(const Vertex& from, const Vertex& to) const {
        std::unordered_map<Vertex, Distance> distances;
        std::unordered_map<Vertex, std::unique_ptr<Edge>> prev;
        std::priority_queue<std::pair<Distance, Vertex>> pq;

        for (const auto& v : vertices_) {
            distances[v] = std::numeric_limits<Distance>::max();
        }
        distances[from] = 0.0;
        prev[from] = nullptr;
        pq.emplace(0.0, from);

        while (!pq.empty()) {
            auto [dist, curr] = pq.top();
            pq.pop();

            if (curr == to) {
                std::vector<Edge> path;
                for (auto v = to; prev[v]; v = prev[v]->from) {
                    path.emplace_back(prev[v]->from, prev[v]->to, prev[v]->weight);
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            if (dist > distances[curr]) {
                continue;
            }

            for (const auto& edge : edges_) {
                if (edge.from == curr) {
                    auto new_dist = distances[curr] + edge.weight;
                    if (new_dist < distances[edge.to]) {
                        distances[edge.to] = new_dist;
                        prev[edge.to] = std::make_unique<Edge>(edge);
                        pq.emplace(-new_dist, edge.to);
                    }
                }
            }
        }

        
