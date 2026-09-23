#pragma once

#include <lemon/bellman_ford.h>
#include <lemon/bfs.h>
#include <lemon/connectivity.h>
#include <lemon/core.h>
#include <lemon/dfs.h>
#include <lemon/dijkstra.h>
#include <lemon/euler.h>
#include <lemon/kruskal.h>
#include <lemon/list_graph.h>
#include <lemon/network_simplex.h>
#include <lemon/path.h>
#include <lemon/preflow.h>

#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <type_traits>
#include <unordered_map>
#include <utility>
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
    private:
        struct InternalConnectionData {
            double weight{1};
            bool symmetric{false};
        };

        NodeMap<NodeProps> nodeProps;
        ArcMap<InternalConnectionData> connectionProps;
        std::unordered_map<KeyType, Node> keyToNode;

        KeyType getKey(const NodeProps &props) const {
            return KeyExtractor<NodeProps>::get(props);
        }
    public:
        Graph()
            : GraphBase(),
              nodeProps(*this),
              connectionProps(*this) {}

        // ==========================================
        // 1. DODAVANJE I BRISANJE
        // ==========================================
        Node addNode(NodeProps props) {
            KeyType key = getKey(props);

            auto it = keyToNode.find(key);
            if (it != keyToNode.end()) {
                nodeProps[it->second] = std::move(props);
                return it->second;
            }

            Node n = GraphBase::addNode();
            nodeProps[n] = std::move(props);
            keyToNode[key] = n;
            return n;
        }

        Connection addConnection(Node u, Node v, double weight = static_cast<double>(1)) {
            Connection existing = findConnection(u, v);
            if (existing != INVALID) {
                connectionProps[existing].weight = weight;
                return existing;
            }

            Connection a = GraphBase::addArc(u, v);
            connectionProps[a] = InternalConnectionData{weight, false};

            Connection reverse = findConnection(v, u);
            if (reverse != INVALID) {
                connectionProps[a].symmetric = true;
                connectionProps[reverse].symmetric = true;
            }

            return a;
        }

        void removeNode(Node n) {
            if (!valid(n)) return;

            KeyType key = getKey(nodeProps[n]);
            keyToNode.erase(key);

            for (OutConnectionIt a(*this, n); a != INVALID; ++a) {
                Connection rev = findConnection(target(a), n);
                if (rev != INVALID) {
                    connectionProps[rev].symmetric = false;
                }
            }

            erase(n);
        }

        void removeConnection(Connection a) {
            if (!valid(a)) return;
            Connection reverse = findConnection(target(a), source(a));
            if (reverse != INVALID) {
                connectionProps[reverse].symmetric = false;
            }
            erase(a);
        }

        // ==========================================
        // 2. PRISTUP PODACIMA
        // ==========================================
        Node getNodeByKey(const KeyType &key) const {
            auto it = keyToNode.find(key);
            return (it != keyToNode.end()) ? it->second : INVALID;
        }

        bool hasNodeKey(const KeyType &key) const {
            return keyToNode.find(key) != keyToNode.end();
        }

        NodeProps &operator[](Node n) {
            return nodeProps[n];
        }

        const NodeProps &operator[](Node n) const {
            return nodeProps[n];
        }

        double &operator[](Connection a) {
            return connectionProps[a].weight;
        }

        const double &operator[](Connection a) const {
            return connectionProps[a].weight;
        }

        bool isSymmetric(Connection a) const {
            return connectionProps[a].symmetric;
        }

        // ==========================================
        // 3. MATRIČNA REPREZENTACIJA
        // ==========================================
        struct MatrixRepresentation {
            std::vector<KeyType> node_keys;
            std::vector<std::vector<double>> matrix;
        };

        MatrixRepresentation getAdjacencyMatrix(
            double no_edge_val = std::numeric_limits<double>::has_infinity
                                     ? std::numeric_limits<double>::infinity()
                                     : std::numeric_limits<double>::max()) const {
            MatrixRepresentation result;
            std::unordered_map<int, size_t> node_id_to_index;
            size_t index = 0;

            for (NodeIt n(*this); n != INVALID; ++n) {
                result.node_keys.push_back(getKey(nodeProps[n]));
                node_id_to_index[GraphBase::id(n)] = index++;
            }

            size_t N = result.node_keys.size();
            result.matrix.assign(N, std::vector<double>(N, no_edge_val));

            for (size_t i = 0; i < N; ++i) {
                result.matrix[i][i] = static_cast<double>(0);
            }

            for (ConnectionIt a(*this); a != INVALID; ++a) {
                size_t u_idx = node_id_to_index[GraphBase::id(source(a))];
                size_t v_idx = node_id_to_index[GraphBase::id(target(a))];
                result.matrix[u_idx][v_idx] = connectionProps[a].weight;
            }

            return result;
        }

        // ==========================================
        // 4. TRAŽENJE PUTANJE (Dijkstra, Bellman-Ford, BFS, A*)
        // ==========================================
        PathResult findShortestPath(Node start, Node end, ShortestPathAlgorithm algo = ShortestPathAlgorithm::Dijkstra, HeuristicFn heuristic = nullptr) const {
            PathResult result;
            if (!valid(start) || !valid(end)) return result;

            if (algo == ShortestPathAlgorithm::BFS) {
                lemon::Bfs<GraphBase> bfs(*this);
                if (!bfs.run(start, end)) return result;

                for (Node v = end; v != INVALID; v = bfs.predNode(v)) {
                    result.path.push_back(v);
                    if (v == start) break;
                }
                std::reverse(result.path.begin(), result.path.end());
                result.found = true;

                for (size_t i = 0; i + 1 < result.path.size(); ++i) {
                    Connection a = findConnection(result.path[i], result.path[i + 1]);
                    result.total_distance += connectionProps[a].weight;
                }
                return result;
            }

            if (algo == ShortestPathAlgorithm::AStar) {
                if (!heuristic) {
                    // Ako heuristika nije zadana, A* se ponaša kao Dijkstra (h = 0)
                    heuristic = [](Node, Node) { return static_cast<double>(0); };
                }

                using Pair = std::pair<double, Node>;
                std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> open_set;

                NodeMap<double> g_score(*this, std::numeric_limits<double>::max());
                NodeMap<Node> parent(*this, INVALID);

                g_score[start] = static_cast<double>(0);
                open_set.push({heuristic(start, end), start});

                while (!open_set.empty()) {
                    Node current = open_set.top().second;
                    open_set.pop();

                    if (current == end) {
                        for (Node v = end; v != INVALID; v = parent[v]) {
                            result.path.push_back(v);
                            if (v == start) break;
                        }
                        std::reverse(result.path.begin(), result.path.end());
                        result.total_distance = g_score[end];
                        result.found = true;
                        return result;
                    }

                    for (OutConnectionIt a(*this, current); a != INVALID; ++a) {
                        Node neighbor = target(a);
                        double tentative_g = g_score[current] + connectionProps[a].weight;

                        if (tentative_g < g_score[neighbor]) {
                            parent[neighbor] = current;
                            g_score[neighbor] = tentative_g;
                            double f_score = tentative_g + heuristic(neighbor, end);
                            open_set.push({f_score, neighbor});
                        }
                    }
                }
                return result;
            }

            // Za Dijkstru i Bellman-Ford
            typename GraphBase::template ArcMap<double> weight_map(*this);
            for (ConnectionIt a(*this); a != INVALID; ++a) {
                weight_map[a] = static_cast<double>(connectionProps[a].weight);
            }

            if (algo == ShortestPathAlgorithm::Dijkstra) {
                lemon::Dijkstra<GraphBase, decltype(weight_map)> dijkstra(*this, weight_map);
                if (!dijkstra.run(start, end)) return result;

                result.total_distance = static_cast<double>(dijkstra.dist(end));
                for (Node v = end; v != INVALID; v = dijkstra.predNode(v)) {
                    result.path.push_back(v);
                    if (v == start) break;
                }
                std::reverse(result.path.begin(), result.path.end());
                result.found = true;
            } else if (algo == ShortestPathAlgorithm::BellmanFord) {
                lemon::BellmanFord<GraphBase, decltype(weight_map)> bf(*this, weight_map);
                bf.init();
                bf.addSource(start);
                bf.start(); // ili jednostavno: bf.run(start);

                // Provjera je li krajnji čvor uopće dostižan
                if (!bf.reached(end)) return result;

                result.total_distance = static_cast<double>(bf.dist(end));
                for (Node v = end; v != INVALID; v = bf.predNode(v)) {
                    result.path.push_back(v);
                    if (v == start) break;
                }
                std::reverse(result.path.begin(), result.path.end());
                result.found = true;
            }

            return result;
        }

        // ==========================================
        // 5. FLOYD-WARSHALL (All-Pairs Shortest Path)
        // ==========================================
        AllPairsShortestPathResult<KeyType> getFloydWarshallMatrix() const {
            AllPairsShortestPathResult<KeyType> res;
            std::unordered_map<int, size_t> id_to_idx;
            size_t idx = 0;

            for (NodeIt n(*this); n != INVALID; ++n) {
                res.node_keys.push_back(getKey(nodeProps[n]));
                id_to_idx[GraphBase::id(n)] = idx++;
            }

            size_t N = res.node_keys.size();
            double inf = std::numeric_limits<double>::has_infinity
                             ? std::numeric_limits<double>::infinity()
                             : std::numeric_limits<double>::max();

            res.dist_matrix.assign(N, std::vector<double>(N, inf));

            for (size_t i = 0; i < N; ++i) res.dist_matrix[i][i] = static_cast<double>(0);

            for (ConnectionIt a(*this); a != INVALID; ++a) {
                size_t u = id_to_idx[GraphBase::id(source(a))];
                size_t v = id_to_idx[GraphBase::id(target(a))];
                res.dist_matrix[u][v] = std::min(res.dist_matrix[u][v], connectionProps[a].weight);
            }

            for (size_t k = 0; k < N; ++k) {
                for (size_t i = 0; i < N; ++i) {
                    for (size_t j = 0; j < N; ++j) {
                        if (res.dist_matrix[i][k] != inf && res.dist_matrix[k][j] != inf) {
                            res.dist_matrix[i][j] = std::min(res.dist_matrix[i][j],
                                                             res.dist_matrix[i][k] + res.dist_matrix[k][j]);
                        }
                    }
                }
            }

            return res;
        }

        // ==========================================
        // 6. MINIMALNO RAZAPINJUĆE STABLO (MST - Kruskal)
        // ==========================================
        std::vector<Connection> findMinimumSpanningTree() const {
            typename GraphBase::template ArcMap<double> weight_map(*this);
            for (ConnectionIt a(*this); a != INVALID; ++a) {
                weight_map[a] = static_cast<double>(connectionProps[a].weight);
            }

            typename GraphBase::template ArcMap<bool> mst_map(*this);
            lemon::kruskal(*this, weight_map, mst_map);

            std::vector<Connection> mst_edges;
            for (ConnectionIt a(*this); a != INVALID; ++a) {
                if (mst_map[a]) {
                    mst_edges.push_back(a);
                }
            }
            return mst_edges;
        }

        // ==========================================
        // 7. MAKSIMALNI PROTOK (Ford-Fulkerson / Preflow)
        // ==========================================
        MaxFlowResult findMaxFlow(Node source_node, Node sink_node) const {
            MaxFlowResult result;
            if (!valid(source_node) || !valid(sink_node)) return result;

            typename GraphBase::template ArcMap<double> capacity_map(*this);
            for (ConnectionIt a(*this); a != INVALID; ++a) {
                capacity_map[a] = static_cast<double>(connectionProps[a].weight);
            }

            lemon::Preflow<GraphBase, decltype(capacity_map)> preflow(*this, capacity_map, source_node, sink_node);
            preflow.run();

            result.max_flow_value = static_cast<double>(preflow.flowValue());
            for (ConnectionIt a(*this); a != INVALID; ++a) {
                result.flow_per_arc[a] = static_cast<double>(preflow.flow(a));
            }

            return result;
        }

        // ==========================================
        // 8. DETEKCIJA CIKLUSA I TOPOLOŠKO SORTIRANJE
        // ==========================================
        bool isAcyclic() const {
            typename GraphBase::template NodeMap<int> order(*this);
            return lemon::checkedTopologicalSort(*this, order);
        }

        std::vector<Node> getTopologicalOrder() const {
            typename GraphBase::template NodeMap<int> order(*this);
            std::vector<Node> sorted_nodes;
            if (!lemon::checkedTopologicalSort(*this, order)) return {};

            sorted_nodes.resize(nodeCount());
            for (NodeIt n(*this); n != INVALID; ++n) {
                sorted_nodes[order[n]] = n;
            }
            return sorted_nodes;
        }

        std::vector<Connection> findCycle() const {
            std::vector<Connection> cycle;
            typename GraphBase::template NodeMap<int> state(*this, 0); // 0: unvisited, 1: visiting, 2: visited
            typename GraphBase::template NodeMap<Connection> parent_arc(*this, INVALID);

            std::function<bool(Node)> dfs = [&](Node u) -> bool {
                state[u] = 1;
                for (OutConnectionIt a(*this, u); a != INVALID; ++a) {
                    Node v = target(a);
                    if (state[v] == 1) {
                        cycle.push_back(a);
                        for (Node curr = u; curr != v; curr = source(parent_arc[curr])) {
                            cycle.push_back(parent_arc[curr]);
                        }
                        std::reverse(cycle.begin(), cycle.end());
                        return true;
                    }
                    if (state[v] == 0) {
                        parent_arc[v] = a;
                        if (dfs(v)) return true;
                    }
                }
                state[u] = 2;
                return false;
            };

            for (NodeIt n(*this); n != INVALID; ++n) {
                if (state[n] == 0 && dfs(n)) break;
            }

            return cycle;
        }

        // ==========================================
        // 9. KINESKI POŠTAR (Chinese Postman Problem - CPP)
        // ==========================================
        bool isEulerian() const {
            for (NodeIt n(*this); n != INVALID; ++n) {
                if (countInArcs(*this, n) != countOutArcs(*this, n)) return false;
            }
            return true;
        }

        std::vector<Connection> getChinesePostmanPath() const {
            std::vector<Connection> tour;
            if (isEulerian()) {
                for (lemon::DiEulerIt<GraphBase> e(*this); e != INVALID; ++e) {
                    tour.push_back(e);
                }
                return tour;
            }

            // Ako nije Eulerov, koristi se Min-Cost Flow za uravnoteženje stupnjeva
            lemon::NetworkSimplex<GraphBase, double, double> ns(*this);
            typename GraphBase::template NodeMap<double> supply(*this, 0);

            for (NodeIt n(*this); n != INVALID; ++n) {
                int diff = countOutArcs(*this, n) - countInArcs(*this, n);
                supply[n] = diff;
            }

            typename GraphBase::template ArcMap<double> cost_map(*this);
            typename GraphBase::template ArcMap<double> cap_map(*this, std::numeric_limits<double>::max());

            for (ConnectionIt a(*this); a != INVALID; ++a) {
                cost_map[a] = static_cast<double>(connectionProps[a].weight);
            }

            ns.costMap(cost_map).upperMap(cap_map).supplyMap(supply);
            if (ns.run() == lemon::NetworkSimplex<GraphBase>::OPTIMAL) {
                for (ConnectionIt a(*this); a != INVALID; ++a) {
                    tour.push_back(a);
                    int extra_copies = static_cast<int>(ns.flow(a));
                    for (int i = 0; i < extra_copies; ++i) {
                        tour.push_back(a);
                    }
                }
            }

            return tour;
        }

        // ==========================================
        // 10. JAKO POVEZANE KOMPONENTE (SCC)
        // ==========================================
        int countStronglyConnectedComponents() const {
            typename GraphBase::template NodeMap<int> comp(*this);
            return lemon::stronglyConnectedComponents(*this, comp);
        }

        // ==========================================
        // 11. ITERATORI I POMOĆNE METODE
        // ==========================================
        template <typename Func>
        void forEachNode(Func func) {
            for (NodeIt n(*this); n != INVALID; ++n) func(n, nodeProps[n]);
        }

        template <typename Func>
        void forEachConnection(Func func) {
            for (ConnectionIt a(*this); a != INVALID; ++a) {
                func(a, connectionProps[a].weight, connectionProps[a].symmetric);
            }
        }

        Connection findConnection(Node u, Node v) const {
            return lemon::findArc(*this, u, v);
        }

        int nodeCount() const {
            return countNodes(*this);
        }

        int arcCount() const {
            return countArcs(*this);
        }
    };

} // namespace graph