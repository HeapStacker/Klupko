#pragma once

#include <lemon/list_graph.h>

#include <functional>
#include <limits>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace graph {

    using lemon::INVALID;
    using GraphBase = lemon::ListDigraph;
    using Node = typename GraphBase::Node;
    using Connection = typename GraphBase::Arc;
    using NodeIt = typename GraphBase::NodeIt;
    using ConnectionIt = typename GraphBase::ArcIt;
    using OutConnectionIt = typename GraphBase::OutArcIt;
    using InConnectionIt = typename GraphBase::InArcIt;

    // Trait za dohvat primarnog ključa čvora
    template <typename T>
    struct KeyExtractor {
        static decltype(auto) get(const T &val) {
            if constexpr (requires { val.get_key(); }) return val.get_key();
            else if constexpr (requires { val.name; }) return val.name;
            else return val;
        }
    };

    // Enum za odabir algoritma pretraživanja najkraćeg puta
    enum class ShortestPathAlgorithm {
        Dijkstra,
        BellmanFord,
        BFS,
        AStar
    };

    // Rezultat traženja putanje
    struct PathResult {
        std::vector<Node> path;
        double total_distance{0};
        bool found{false};
    };

    // Rezultat Floyd-Warshall algoritma
    template <typename KeyType>
    struct AllPairsShortestPathResult {
        std::vector<KeyType> node_keys;
        std::vector<std::vector<double>> dist_matrix;
    };

    // Rezultat za Maksimalni Protok (Ford-Fulkerson / Preflow)
    struct MaxFlowResult {
        double max_flow_value{0};
        std::unordered_map<Connection, double> flow_per_arc;
    };

    template <typename NodeProps>
    class Graph : public lemon::ListDigraph {
    public:
        using KeyType = std::decay_t<decltype(KeyExtractor<NodeProps>::get(std::declval<NodeProps>()))>;
        using HeuristicFn = std::function<double(Node, Node)>;

        struct MatrixRepresentation {
            std::vector<KeyType> node_keys;
            std::vector<std::vector<double>> matrix;
        };

        Graph();

        // ==========================================
        // 1. DODAVANJE I BRISANJE
        // ==========================================
        Node addNode(NodeProps props);
        Connection addConnection(Node u, Node v, double weight = static_cast<double>(1));
        void removeNode(Node n);
        void removeConnection(Connection a);

        // ==========================================
        // 2. PRISTUP PODACIMA
        // ==========================================
        Node getNodeByKey(const KeyType &key) const;
        bool hasNodeKey(const KeyType &key) const;

        NodeProps &operator[](Node n);
        const NodeProps &operator[](Node n) const;
        double &operator[](Connection a);
        const double &operator[](Connection a) const;

        bool isSymmetric(Connection a) const;

        // ==========================================
        // 3. MATRIČNA REPREZENTACIJA
        // ==========================================
        MatrixRepresentation getAdjacencyMatrix(
            double no_edge_val = std::numeric_limits<double>::has_infinity
                                     ? std::numeric_limits<double>::infinity()
                                     : std::numeric_limits<double>::max()) const;

        // ==========================================
        // 4. TRAŽENJE PUTANJE (Dijkstra, Bellman-Ford, BFS, A*)
        // ==========================================
        PathResult findShortestPath(Node start,
                                    Node end,
                                    ShortestPathAlgorithm algo = ShortestPathAlgorithm::Dijkstra,
                                    HeuristicFn heuristic = nullptr) const;

        // ==========================================
        // 5. FLOYD-WARSHALL (All-Pairs Shortest Path)
        // ==========================================
        AllPairsShortestPathResult<KeyType> getFloydWarshallMatrix() const;

        // ==========================================
        // 6. MINIMALNO RAZAPINJUĆE STABLO (MST - Kruskal)
        // ==========================================
        std::vector<Connection> findMinimumSpanningTree() const;

        // ==========================================
        // 7. MAKSIMALNI PROTOK (Ford-Fulkerson / Preflow)
        // ==========================================
        MaxFlowResult findMaxFlow(Node source_node, Node sink_node) const;

        // ==========================================
        // 8. DETEKCIJA CIKLUSA I TOPOLOŠKO SORTIRANJE
        // ==========================================
        bool isAcyclic() const;
        std::vector<Node> getTopologicalOrder() const;
        std::vector<Connection> findCycle() const;

        // ==========================================
        // 9. KINESKI POŠTAR (Chinese Postman Problem - CPP)
        // ==========================================
        bool isEulerian() const;
        std::vector<Connection> getChinesePostmanPath() const;

        // ==========================================
        // 10. JAKO POVEZANE KOMPONENTE (SCC)
        // ==========================================
        int countStronglyConnectedComponents() const;

        // ==========================================
        // 11. ITERATORI I POMOĆNE METODE
        // ==========================================
        template <typename Func>
        void forEachNode(Func func);

        template <typename Func>
        void forEachConnection(Func func);

        Connection findConnection(Node u, Node v) const;
        int nodeCount() const;
        int arcCount() const;

    private:
        struct InternalConnectionData {
            double weight{1};
            bool symmetric{false};
        };

        NodeMap<NodeProps> nodeProps;
        ArcMap<InternalConnectionData> connectionProps;
        std::unordered_map<KeyType, Node> keyToNode;

        KeyType getKey(const NodeProps &props) const;
    };

} // namespace graph

#include "Graph.inl"
