#pragma once

#include<iostream>
#include<list>
#include<vector>
#include<exception>
#include <queue>
#include <limits>
#include <algorithm>
#include <string>

const int INF = std::numeric_limits<int>::max();

using namespace std;

template<typename V, typename Distance = double>
class Graph {
private:
	struct Edge {
		int _num;
		Distance _val;
	};
	struct Vertex {
		int _num;
		V _val;
		list<Edge> _edge;
	};
	vector<Vertex> _graph;



	void dfs_h(int from, vector<int>& visited)const {
		visited[from] = 1;
		for (auto& i : _graph[from]._edge) {
			if (!visited[i._num]) {
				dfs_h(i._num, visited);
			}
		}
		cout << _graph[from]._val << " ";
	}
	void process_dfs_h(int from, vector<int>& visited, void (*v_process)(V)) {
		visited[from] = 1;
		for (auto& i : _graph[from]._edge) {
			if (!visited[i._num]) {
				process_dfs_h(i._num, visited, v_process);
			}
		}
		v_process(_graph[from]._val);
	}
	void rework_for_chill_life() {
		size_t count = 0;
		for (auto& i : _graph) {
			i._num = count;
			count++;
		}
	}
	Vertex& operator[](V val) {
		for (auto& vertex : _graph) {
			if (vertex._val == val) {
				return vertex;
			}
		}
		throw invalid_argument("No such vertex");
	}
public:
	//HELP
	Graph() = default;
	void print() const {
		for (auto& vertex : _graph) {
			cout << vertex._val;
			for (auto& edge : vertex._edge) {
				cout << "-->" << "|" << _graph[edge._num]._val << "-" << edge._val << "|";
			}
			cout << endl;
		}
	}
	size_t size() const {
		return _graph.size();
	}
	vector<Vertex> vertices() const {
		return _graph;
	}
	list<Edge> edges(V from) const {
		if (has_vertex(from))
			return (*this)[from]._edge;
	}
	size_t order() const {
		return size();
	}
	size_t degree(V val) {
		return (*this)[val]._edge.size();
	}
	//VERTEX
	bool has_vertex(V _num) {
		try {
			(*this)(_num);
		}
		catch (invalid_argument& e) {
			return false;
		}
		return true;
	}
	void add_vertex(V val) {
		int new_num = _graph.size();
		_graph.push_back({ new_num,val });
	}
	void remove_vertex(V val) {
		for (auto& vertex : _graph) {
			vertex._edge.remove_if([&](Edge& x) { return _graph[x._num]._val == val; });
		}
		_graph.erase(std::remove_if(_graph.begin(), _graph.end(), [val](Vertex& v)
			{
				return v._val == val;
			}
		), _graph.end());
		rework_for_chill_life();
	}
	//EDGE
	void add_edge(V from, V to, Distance val) {
		try {
			(*this)[from]._edge.push_back({ (*this)[to]._num, val });
		}
		catch (invalid_argument& e) {
			std::cerr << e.what() << std::endl;
		}
	}
	bool has_edge(V from, V to) {
		if (has_vertex(from)) {
			for (auto& edge : (*this)[from]._edge)
				if (_graph[edge._num]._val == to)
					return true;
		}
		return false;
	}
	bool has_edge(V from, V to, Distance val) {
		if (has_vertex(from)) {
			for (auto& edge : (*this)[from]._edge)
				if (_graph[edge._num]._val == to && edge._val == val)
					return true;
		}
		return false;
	}
	void remove_edge(V from, V to) {
		if (has_vertex(from))
			(*this)[from]._edge.remove_if([&](Edge& x) { return _graph[x._num]._val == to; });
	}
	void remove_edge(V from, V to, Distance val) {
		if (has_vertex(from))
			(*this)[from]._edge.remove_if([&](Edge& x) { return (_graph[x._num]._val == to && x._val == val); });
	}

	bool has_negative_weights() {
		for (const auto& vertex : _graph) {
			for (const auto& edge : vertex._edge) {
				if (edge._val < 0) {
					return true;
				}
			}
		}
		return false;
	}


	//FUNCTIONAL
	void dfs(V start_vertex) {
		if (has_vertex(start_vertex)) {
			vector<int> visited(size() + 1, 0);
			dfs_h((*this)[start_vertex]._num, visited);
		}
	}
	vector<double> Dijkstra(V _from, bool flag) {
		if (has_negative_weights()) {
			throw std::runtime_error("Граф содержит отрицательные веса. Алгоритм Дейкстры не может быть использован.");
		}

		auto from = (*this)[_from]._num;
		vector<double> distance(size(), INF);
		distance[from] = 0;

		priority_queue<std::pair<int, int>, vector<std::pair<int, int>>, greater<>> queue;
		queue.push({ 0, from });

		while (!queue.empty()) {
			int u = queue.top().second;
			queue.pop();

			for (auto& edge : _graph[u]._edge) {
				int v = edge._num;
				int weight = edge._val;

				if (distance[v] > distance[u] + weight) {
					distance[v] = distance[u] + weight;
					queue.push({ distance[v], v });
				}
			}
		}

		if (flag) {
			for (int i = 0; i < size(); i++) {
				std::cout << "Shortest distance from vertex " << _from << " to vertex " << _graph[i]._val << " is " << distance[i] << std::endl;
			}
		}
		return distance;
	}
	V find_graph_center() {
		int center = -1;
		int max_dist = -1;
		for (auto& vertex : _graph) {
			vector<double> dist = Dijkstra(vertex._val, false);
			int max_d = *max_element(dist.begin(), dist.end());
			if (max_d < max_dist || max_dist == -1) {
				max_dist = max_d;
				center = vertex._num;
			}
		}
		return _graph[center]._val;
	}
	void process_dfs(V start_vertex, void (*v_process)(V)) {
		if (has_vertex(start_vertex)) {
			vector<int> visited(size() + 1, 0);
			process_dfs_h((*this)[start_vertex]._num, visited, v_process);
		}
	}

	V find_optimal_warehouse() {
		double min_avg_distance = INF;
		V optimal_warehouse;

		for (auto& vertex : _graph) {
			double total_distance = 0;
			for (auto& other_vertex : _graph) {
				if (vertex._val != other_vertex._val) {
					vector<double> dist = Dijkstra(other_vertex._val, false);
					total_distance += *min_element(dist.begin(), dist.end());
				}
			}

			double avg_distance = total_distance / (_graph.size() - 1);
			if (avg_distance < min_avg_distance) {
				min_avg_distance = avg_distance;
				optimal_warehouse = vertex._val;
			}
		}

		return optimal_warehouse;
	}
};
