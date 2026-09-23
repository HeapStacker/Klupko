#pragma once

#include <lemon/list_graph.h>

#include <functional>
#include <limits>
#include <memory>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace graph {

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

    template <typename NodeProps>
    class Graph {
    public:
        using KeyType = std::decay_t<decltype(KeyExtractor<NodeProps>::get(std::declval<NodeProps>()))>;
        using HeuristicFn = std::function<double(const NodeProps &, const NodeProps &)>;
        using Path = PathResult<Node, Connection>;
        using Tree = ShortestPathTree<GraphBase, Node, Connection>;
        using Flow = MaxFlowResult<GraphBase, Connection>;
        using Components = ComponentPartition<Node>;
        using Postman = PostmanResult<Connection>;

        struct MatrixRepresentation {
            std::vector<KeyType> node_keys;
            std::vector<std::vector<double>> matrix;
        };

        Graph();
        Graph(const Graph &other);
        Graph(Graph &&other);
        Graph &operator=(const Graph &other);
        Graph &operator=(Graph &&other);
        ~Graph();

        Node addNode(NodeProps props);
        Connection addConnection(Node u, Node v, double weight = 1);
        void updateNode(Node n, NodeProps props);
        void removeNode(Node n);
        void removeConnection(Connection a);

        Node getNodeByKey(const KeyType &key) const;
        bool hasNodeKey(const KeyType &key) const;

        const NodeProps &operator[](Node n) const;
        double &operator[](Connection a);
        const double &operator[](Connection a) const;

        double weight(Connection a) const;
        double capacity(Connection a) const;
        double cost(Connection a) const;
        void setWeight(Connection a, double value);
        void setCapacity(Connection a, double value);
        void setCost(Connection a, double value);
        bool hasReverse(Connection a) const;

        bool valid(Node n) const;
        bool valid(Connection a) const;
        Node source(Connection a) const;
        Node target(Connection a) const;
        const GraphBase &lemonGraph() const;

        MatrixRepresentation getAdjacencyMatrix(double no_edge_val = std::numeric_limits<double>::infinity()) const;

        Path findShortestPath(Node start,
                              Node end,
                              ShortestPathAlgorithm algo = ShortestPathAlgorithm::Dijkstra,
                              HeuristicFn heuristic = nullptr) const;

        Tree shortestPathTree(Node source, ShortestPathAlgorithm algo = ShortestPathAlgorithm::Dijkstra) const;

        AllPairsShortestPathResult<KeyType> getFloydWarshallMatrix() const;

        ArborescenceResult findMinimumSpanningArborescence(Node root) const;

        Flow findMaxFlow(Node source_node, Node sink_node) const;

        bool isAcyclic() const;
        std::vector<Node> getTopologicalOrder() const;
        std::vector<Connection> findCycle() const;

        bool isWeaklyConnected() const;
        bool isEulerian() const;
        Postman chinesePostman() const;

        Components stronglyConnectedComponents() const;

        template <typename Func>
        void forEachNode(Func func) const;

        template <typename Func>
        void forEachConnection(Func func) const;

        Connection findConnection(Node u, Node v) const;
        int nodeCount() const;
        int arcCount() const;
    private:
        struct Core {
            GraphBase digraph;
            typename GraphBase::template NodeMap<NodeProps> nodeProps;
            typename GraphBase::template ArcMap<ConnectionData> connectionProps;

            Core()
                : nodeProps(digraph),
                  connectionProps(digraph) {}

            Core(const Core &) = delete;
            Core &operator=(const Core &) = delete;
            Core(Core &&) = delete;
            Core &operator=(Core &&) = delete;
        };

        std::unique_ptr<Core> core;
        std::unordered_map<KeyType, Node> keyToNode;

        GraphBase &base();
        const GraphBase &base() const;
        KeyType getKey(const NodeProps &props) const;
    };

    template <typename NodeProps>
    class UndirectedGraph {
    public:
        using Node = typename UndirectedBase::Node;
        using Edge = typename UndirectedBase::Edge;
        using KeyType = std::decay_t<decltype(KeyExtractor<NodeProps>::get(std::declval<NodeProps>()))>;
        using HeuristicFn = std::function<double(const NodeProps &, const NodeProps &)>;
        using Path = PathResult<Node, Edge>;
        using Tree = ShortestPathTree<UndirectedBase, Node, Edge>;
        using Components = ComponentPartition<Node>;
        using Forest = SpanningForest<Edge>;
        using Postman = PostmanResult<Edge>;
        using Flow = MaxFlowResult<UndirectedBase, Edge>;

        struct MatrixRepresentation {
            std::vector<KeyType> node_keys;
            std::vector<std::vector<double>> matrix;
        };

        UndirectedGraph();
        UndirectedGraph(const UndirectedGraph &other);
        UndirectedGraph(UndirectedGraph &&other);
        UndirectedGraph &operator=(const UndirectedGraph &other);
        UndirectedGraph &operator=(UndirectedGraph &&other);
        ~UndirectedGraph();

        Node addNode(NodeProps props);
        Edge addEdge(Node u, Node v, double weight = 1);
        void updateNode(Node n, NodeProps props);
        void removeNode(Node n);
        void removeEdge(Edge e);

        Node getNodeByKey(const KeyType &key) const;
        bool hasNodeKey(const KeyType &key) const;

        const NodeProps &operator[](Node n) const;
        double &operator[](Edge e);
        const double &operator[](Edge e) const;

        double weight(Edge e) const;
        double capacity(Edge e) const;
        double cost(Edge e) const;
        void setWeight(Edge e, double value);
        void setCapacity(Edge e, double value);
        void setCost(Edge e, double value);

        bool valid(Node n) const;
        bool valid(Edge e) const;
        Node u(Edge e) const;
        Node v(Edge e) const;
        const UndirectedBase &lemonGraph() const;

        MatrixRepresentation getAdjacencyMatrix(double no_edge_val = std::numeric_limits<double>::infinity()) const;

        Path findShortestPath(Node start,
                              Node end,
                              ShortestPathAlgorithm algo = ShortestPathAlgorithm::Dijkstra,
                              HeuristicFn heuristic = nullptr) const;
        Tree shortestPathTree(Node source, ShortestPathAlgorithm algo = ShortestPathAlgorithm::Dijkstra) const;

        AllPairsShortestPathResult<KeyType> getFloydWarshallMatrix() const;

        Forest findMinimumSpanningForest() const;
        Components connectedComponents() const;

        Flow findMaxFlow(Node source_node, Node sink_node) const;

        bool isConnected() const;
        bool isEulerian() const;
        Postman chinesePostman() const;

        template <typename Func>
        void forEachNode(Func func) const;

        template <typename Func>
        void forEachEdge(Func func) const;

        Edge findEdge(Node u, Node v) const;
        int nodeCount() const;
        int edgeCount() const;
    private:
        struct Core {
            UndirectedBase graph;
            typename UndirectedBase::template NodeMap<NodeProps> nodeProps;
            typename UndirectedBase::template EdgeMap<ConnectionData> edgeProps;

            Core()
                : nodeProps(graph),
                  edgeProps(graph) {}

            Core(const Core &) = delete;
            Core &operator=(const Core &) = delete;
            Core(Core &&) = delete;
            Core &operator=(Core &&) = delete;
        };

        std::unique_ptr<Core> core;
        std::unordered_map<KeyType, Node> keyToNode;

        UndirectedBase &base();
        const UndirectedBase &base() const;
        KeyType getKey(const NodeProps &props) const;
    };

} // namespace graph

#include "Graph.inl"
