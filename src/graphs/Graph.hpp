#pragma once

#include <lemon/bfs.h>
#include <lemon/core.h>
#include <lemon/dijkstra.h>
#include <lemon/list_graph.h>

#include <algorithm>
#include <limits>
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

    template <typename NodeProps, typename ConnectionWeight = double>
    class Graph : public lemon::ListDigraph {

        static_assert(std::is_arithmetic_v<ConnectionWeight>,
                      "ERROR: Graph ConnectionWeight must be a numeric type (e.g. int, float, double)!");
    public:
        using KeyType = std::decay_t<decltype(KeyExtractor<NodeProps>::get(std::declval<NodeProps>()))>;
    private:
        struct InternalConnectionData {
            ConnectionWeight weight{1};
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

        // 1. DODAVANJE I BRISANJE
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

        // Sada prosljeđujemo direktno numeričku težinu (podrazumijevano 1.0)
        Connection addConnection(Node u, Node v, ConnectionWeight weight = static_cast<ConnectionWeight>(1)) {
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

        // 2. PRISTUP PODACIMA
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

        // Pristup težini luka preko operatora []
        ConnectionWeight &operator[](Connection a) {
            return connectionProps[a].weight;
        }

        const ConnectionWeight &operator[](Connection a) const {
            return connectionProps[a].weight;
        }

        bool isSymmetric(Connection a) const {
            return connectionProps[a].symmetric;
        }

        // 3. MATRIČNA REPREZENTACIJA (Čista i bez lambda funkcija!)
        struct MatrixRepresentation {
            std::vector<KeyType> node_keys;
            std::vector<std::vector<ConnectionWeight>> matrix;
        };

        MatrixRepresentation getAdjacencyMatrix(
            ConnectionWeight no_edge_val = std::numeric_limits<ConnectionWeight>::has_infinity
                                               ? std::numeric_limits<ConnectionWeight>::infinity()
                                               : std::numeric_limits<ConnectionWeight>::max()) const {
            MatrixRepresentation result;

            std::unordered_map<int, size_t> node_id_to_index;
            size_t index = 0;

            for (NodeIt n(*this); n != INVALID; ++n) {
                result.node_keys.push_back(getKey(nodeProps[n]));
                node_id_to_index[GraphBase::id(n)] = index++;
            }

            size_t N = result.node_keys.size();
            result.matrix.assign(N, std::vector<ConnectionWeight>(N, no_edge_val));

            for (size_t i = 0; i < N; ++i) {
                result.matrix[i][i] = static_cast<ConnectionWeight>(0);
            }

            for (ConnectionIt a(*this); a != INVALID; ++a) {
                size_t u_idx = node_id_to_index[GraphBase::id(source(a))];
                size_t v_idx = node_id_to_index[GraphBase::id(target(a))];
                result.matrix[u_idx][v_idx] = connectionProps[a].weight;
            }

            return result;
        }

        // 4. DIJKSTRA (Automatski čita numeričke težine)
        std::vector<Node> findShortestPath(Node start, Node end) const {
            if (!valid(start) || !valid(end)) return {};

            typename GraphBase::template ArcMap<double> weight_map(*this);
            for (ConnectionIt a(*this); a != INVALID; ++a) {
                weight_map[a] = static_cast<double>(connectionProps[a].weight);
            }

            lemon::Dijkstra<GraphBase, typename GraphBase::template ArcMap<double>> dijkstra(*this, weight_map);
            if (!dijkstra.run(start, end)) return {};

            std::vector<Node> path;
            for (Node v = end; v != INVALID; v = dijkstra.predNode(v)) {
                path.push_back(v);
                if (v == start) break;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        // 5. ITERATORI
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
            return countConnections(*this);
        }
    };

} // namespace graph