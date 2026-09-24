#pragma once

#include "GraphSpecific.hpp"

#include <functional>
#include <memory>
#include <type_traits>

namespace graph {

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

#include "UndirectedGraph.inl"
