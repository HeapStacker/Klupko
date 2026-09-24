#pragma once

#include <lemon/list_graph.h>

#include <limits>
#include <unordered_map>
#include <vector>

namespace graph {

    inline double infinity() { return std::numeric_limits<double>::infinity(); }

    using lemon::INVALID;
    using GraphBase = lemon::ListDigraph;
    using UndirectedBase = lemon::ListGraph;
    using Node = typename GraphBase::Node;
    using Connection = typename GraphBase::Arc;
    using NodeIt = typename GraphBase::NodeIt;
    using ConnectionIt = typename GraphBase::ArcIt;
    using OutConnectionIt = typename GraphBase::OutArcIt;
    using InConnectionIt = typename GraphBase::InArcIt;

    template <typename GraphT, typename Item>
    struct LemonIdHash {
        std::size_t operator()(const Item &item) const noexcept {
            return static_cast<std::size_t>(GraphT::id(item));
        }
    };

    template <typename T>
    struct KeyExtractor {
        static decltype(auto) get(const T &val) {
            if constexpr (requires { val.get_key(); }) return val.get_key();
            else if constexpr (requires { val.name; }) return val.name;
            else return val;
        }
    };

    // weight: duljina veze (najkraći put, matrica, MST, arborescencija)
    // capacity: kapacitet veze (maksimalni protok)
    // cost: cijena prolaska vezom (kineski poštar)
    struct ConnectionData {
        double weight{1};
        double capacity{1};
        double cost{1};
    };

    enum class ShortestPathAlgorithm {
        Dijkstra,
        BellmanFord,
        BFS,
        AStar
    };

    enum class PathStatus {
        Found,
        Unreachable,
        InvalidNodes,
        NegativeCycle,
        NegativeWeight,
        Unsupported
    };

    // total_weight je zbroj težina veza na putu.
    // hops je broj veza. BFS minimizira hops.
    // Dijkstra, Bellman-Ford i A* minimiziraju total_weight.
    template <typename NodeT, typename ConnT>
    struct PathResult {
        PathStatus status{PathStatus::Unreachable};
        bool found{false};
        std::vector<NodeT> path;
        std::vector<ConnT> arcs;
        double total_weight{std::numeric_limits<double>::infinity()};
        int hops{0};
    };

    template <typename GraphT, typename NodeT, typename ConnT>
    struct ShortestPathTree {
        PathStatus status{PathStatus::Unreachable};
        NodeT source{};
        std::unordered_map<NodeT, double, LemonIdHash<GraphT, NodeT>> distance;
        std::unordered_map<NodeT, ConnT, LemonIdHash<GraphT, NodeT>> predecessor;
    };

    template <typename KeyType>
    struct AllPairsShortestPathResult {
        std::vector<KeyType> node_keys;
        std::vector<std::vector<double>> dist_matrix;
        std::vector<std::vector<int>> next;
        bool has_negative_cycle{false};

        int indexOf(const KeyType &key) const {
            for (int i = 0; i < static_cast<int>(node_keys.size()); ++i) {
                if (node_keys[i] == key) return i;
            }
            return -1;
        }

        std::vector<KeyType> path(int from, int to) const {
            const int n = static_cast<int>(node_keys.size());
            if (from < 0 || to < 0 || from >= n || to >= n) return {};
            if (dist_matrix.empty() || next.empty()) return {};
            if (from == to) {
                if (dist_matrix[from][from] < 0) return {};
                return {node_keys[from]};
            }
            if (next[from][to] < 0) return {};

            std::vector<KeyType> out;
            int i = from;
            out.push_back(node_keys[i]);
            for (int steps = 0; i != to && steps <= n; ++steps) {
                i = next[i][to];
                if (i < 0 || i >= n) return {};
                out.push_back(node_keys[i]);
            }
            if (i != to) return {};
            return out;
        }
    };

    template <typename GraphT, typename ConnT>
    struct MaxFlowResult {
        bool feasible{false};
        double max_flow_value{0};
        std::unordered_map<ConnT, double, LemonIdHash<GraphT, ConnT>> flow;
    };

    template <typename ConnT>
    struct SpanningForest {
        std::vector<ConnT> edges;
        double total_weight{0};
        int component_count{0};
        bool is_tree{false};
    };

    struct ArborescenceResult {
        std::vector<Connection> arcs;
        double total_weight{0};
        bool spans_all{false};
    };

    enum class PostmanStatus {
        Found,
        Disconnected,
        NegativeCost,
        Infeasible,
        Unbounded
    };

    template <typename ConnT>
    struct PostmanResult {
        PostmanStatus status{PostmanStatus::Infeasible};
        std::vector<ConnT> tour;
        double total_cost{std::numeric_limits<double>::infinity()};
    };

    template <typename NodeT>
    struct ComponentPartition {
        int count{0};
        std::vector<std::vector<NodeT>> components;
    };

} // namespace graph
