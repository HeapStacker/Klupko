#pragma once

#include <lemon/bellman_ford.h>
#include <lemon/bfs.h>
#include <lemon/connectivity.h>
#include <lemon/core.h>
#include <lemon/dijkstra.h>
#include <lemon/euler.h>
#include <lemon/kruskal.h>
#include <lemon/matching.h>
#include <lemon/preflow.h>

#include <algorithm>
#include <queue>
#include <stdexcept>
#include <utility>

namespace graph {

    template <typename NodeProps>
    UndirectedGraph<NodeProps>::UndirectedGraph()
        : core(std::make_unique<Core>()) {}

    template <typename NodeProps>
    UndirectedGraph<NodeProps>::UndirectedGraph(const UndirectedGraph &other)
        : core(std::make_unique<Core>()) {
        typename UndirectedBase::template NodeMap<Node> node_ref(other.base());
        typename UndirectedBase::template EdgeMap<Edge> edge_ref(other.base());
        lemon::GraphCopy<UndirectedBase, UndirectedBase> copy(other.base(), base());
        copy.nodeRef(node_ref).edgeRef(edge_ref);
        copy.nodeMap(other.core->nodeProps, core->nodeProps);
        copy.edgeMap(other.core->edgeProps, core->edgeProps);
        copy.run();
        for (const auto &entry : other.keyToNode) keyToNode.emplace(entry.first, node_ref[entry.second]);
    }

    template <typename NodeProps>
    UndirectedGraph<NodeProps>::UndirectedGraph(UndirectedGraph &&other)
        : core(std::move(other.core)),
          keyToNode(std::move(other.keyToNode)) {
        other.core = std::make_unique<Core>();
        other.keyToNode.clear();
    }

    template <typename NodeProps>
    UndirectedGraph<NodeProps> &UndirectedGraph<NodeProps>::operator=(const UndirectedGraph &other) {
        if (this == &other) return *this;
        UndirectedGraph copy(other);
        return *this = std::move(copy);
    }

    template <typename NodeProps>
    UndirectedGraph<NodeProps> &UndirectedGraph<NodeProps>::operator=(UndirectedGraph &&other) {
        if (this == &other) return *this;
        core = std::move(other.core);
        keyToNode = std::move(other.keyToNode);
        other.core = std::make_unique<Core>();
        other.keyToNode.clear();
        return *this;
    }

    template <typename NodeProps>
    UndirectedGraph<NodeProps>::~UndirectedGraph() = default;

    template <typename NodeProps>
    UndirectedBase &UndirectedGraph<NodeProps>::base() {
        return core->graph;
    }

    template <typename NodeProps>
    const UndirectedBase &UndirectedGraph<NodeProps>::base() const {
        return core->graph;
    }

    template <typename NodeProps>
    typename UndirectedGraph<NodeProps>::KeyType UndirectedGraph<NodeProps>::getKey(const NodeProps &props) const {
        return KeyExtractor<NodeProps>::get(props);
    }

    template <typename NodeProps>
    typename UndirectedGraph<NodeProps>::Node UndirectedGraph<NodeProps>::addNode(NodeProps props) {
        KeyType key = getKey(props);
        if (keyToNode.find(key) != keyToNode.end()) {
            throw std::invalid_argument("addNode: a node with this key already exists");
        }
        Node n = base().addNode();
        core->nodeProps[n] = std::move(props);
        keyToNode.emplace(std::move(key), n);
        return n;
    }

    template <typename NodeProps>
    typename UndirectedGraph<NodeProps>::Edge UndirectedGraph<NodeProps>::addEdge(Node u, Node v, double weight) {
        if (!valid(u) || !valid(v)) throw std::invalid_argument("addEdge: invalid node");
        Edge existing = findEdge(u, v);
        if (existing != INVALID) {
            core->edgeProps[existing].weight = weight;
            return existing;
        }
        Edge e = base().addEdge(u, v);
        core->edgeProps[e] = ConnectionData{weight, weight, weight};
        return e;
    }

    template <typename NodeProps>
    void UndirectedGraph<NodeProps>::updateNode(Node n, NodeProps props) {
        if (!valid(n)) throw std::invalid_argument("updateNode: invalid node");
        KeyType new_key = getKey(props);
        KeyType old_key = getKey(core->nodeProps[n]);
        if (!(new_key == old_key)) {
            auto it = keyToNode.find(new_key);
            if (it != keyToNode.end() && it->second != n) {
                throw std::invalid_argument("updateNode: a node with this key already exists");
            }
            keyToNode.erase(old_key);
            keyToNode.emplace(std::move(new_key), n);
        }
        core->nodeProps[n] = std::move(props);
    }

    template <typename NodeProps>
    void UndirectedGraph<NodeProps>::removeNode(Node n) {
        if (!valid(n)) return;
        keyToNode.erase(getKey(core->nodeProps[n]));
        base().erase(n);
    }

    template <typename NodeProps>
    void UndirectedGraph<NodeProps>::removeEdge(Edge e) {
        if (!valid(e)) return;
        base().erase(e);
    }

    template <typename NodeProps>
    typename UndirectedGraph<NodeProps>::Node UndirectedGraph<NodeProps>::getNodeByKey(const KeyType &key) const {
        auto it = keyToNode.find(key);
        return it != keyToNode.end() ? it->second : INVALID;
    }

    template <typename NodeProps>
    bool UndirectedGraph<NodeProps>::hasNodeKey(const KeyType &key) const {
        return keyToNode.find(key) != keyToNode.end();
    }

    template <typename NodeProps>
    const NodeProps &UndirectedGraph<NodeProps>::operator[](Node n) const {
        return core->nodeProps[n];
    }

    template <typename NodeProps>
    double &UndirectedGraph<NodeProps>::operator[](Edge e) {
        return core->edgeProps[e].weight;
    }

    template <typename NodeProps>
    const double &UndirectedGraph<NodeProps>::operator[](Edge e) const {
        return core->edgeProps[e].weight;
    }

    template <typename NodeProps>
    double UndirectedGraph<NodeProps>::weight(Edge e) const {
        return core->edgeProps[e].weight;
    }

    template <typename NodeProps>
    double UndirectedGraph<NodeProps>::capacity(Edge e) const {
        return core->edgeProps[e].capacity;
    }

    template <typename NodeProps>
    double UndirectedGraph<NodeProps>::cost(Edge e) const {
        return core->edgeProps[e].cost;
    }

    template <typename NodeProps>
    void UndirectedGraph<NodeProps>::setWeight(Edge e, double value) {
        core->edgeProps[e].weight = value;
    }

    template <typename NodeProps>
    void UndirectedGraph<NodeProps>::setCapacity(Edge e, double value) {
        core->edgeProps[e].capacity = value;
    }

    template <typename NodeProps>
    void UndirectedGraph<NodeProps>::setCost(Edge e, double value) {
        core->edgeProps[e].cost = value;
    }

    template <typename NodeProps>
    bool UndirectedGraph<NodeProps>::valid(Node n) const {
        return base().valid(n);
    }

    template <typename NodeProps>
    bool UndirectedGraph<NodeProps>::valid(Edge e) const {
        return base().valid(e);
    }

    template <typename NodeProps>
    typename UndirectedGraph<NodeProps>::Node UndirectedGraph<NodeProps>::u(Edge e) const {
        return base().u(e);
    }

    template <typename NodeProps>
    typename UndirectedGraph<NodeProps>::Node UndirectedGraph<NodeProps>::v(Edge e) const {
        return base().v(e);
    }

    template <typename NodeProps>
    const UndirectedBase &UndirectedGraph<NodeProps>::lemonGraph() const {
        return base();
    }

    template <typename NodeProps>
    typename UndirectedGraph<NodeProps>::Edge UndirectedGraph<NodeProps>::findEdge(Node u, Node v) const {
        return lemon::findEdge(base(), u, v);
    }

    template <typename NodeProps>
    int UndirectedGraph<NodeProps>::nodeCount() const {
        return lemon::countNodes(base());
    }

    template <typename NodeProps>
    int UndirectedGraph<NodeProps>::edgeCount() const {
        return lemon::countEdges(base());
    }

    template <typename NodeProps>
    template <typename Func>
    void UndirectedGraph<NodeProps>::forEachNode(Func func) const {
        for (typename UndirectedBase::NodeIt n(base()); n != INVALID; ++n) func(n, core->nodeProps[n]);
    }

    template <typename NodeProps>
    template <typename Func>
    void UndirectedGraph<NodeProps>::forEachEdge(Func func) const {
        for (typename UndirectedBase::EdgeIt e(base()); e != INVALID; ++e) func(e, core->edgeProps[e]);
    }

    template <typename NodeProps>
    typename UndirectedGraph<NodeProps>::MatrixRepresentation
    UndirectedGraph<NodeProps>::getAdjacencyMatrix(double no_edge_val) const {
        MatrixRepresentation result;
        std::unordered_map<int, std::size_t> node_id_to_index;
        std::size_t index = 0;
        for (typename UndirectedBase::NodeIt n(base()); n != INVALID; ++n) {
            result.node_keys.push_back(getKey(core->nodeProps[n]));
            node_id_to_index[UndirectedBase::id(n)] = index++;
        }
        const std::size_t n = result.node_keys.size();
        result.matrix.assign(n, std::vector<double>(n, no_edge_val));
        for (std::size_t i = 0; i < n; ++i) result.matrix[i][i] = 0;
        for (typename UndirectedBase::EdgeIt e(base()); e != INVALID; ++e) {
            std::size_t u = node_id_to_index[UndirectedBase::id(base().u(e))];
            std::size_t v = node_id_to_index[UndirectedBase::id(base().v(e))];
            result.matrix[u][v] = core->edgeProps[e].weight;
            result.matrix[v][u] = core->edgeProps[e].weight;
        }
        return result;
    }

    template <typename NodeProps>
    typename UndirectedGraph<NodeProps>::Path UndirectedGraph<NodeProps>::findShortestPath(Node start,
                                                                                           Node end,
                                                                                           ShortestPathAlgorithm algo,
                                                                                           HeuristicFn heuristic) const {
        using Arc = typename UndirectedBase::Arc;
        Path result;
        if (!valid(start) || !valid(end)) {
            result.status = PathStatus::InvalidNodes;
            return result;
        }

        auto trace = [&](auto predecessor) -> Path {
            Path traced;
            if (start == end) {
                traced.status = PathStatus::Found;
                traced.found = true;
                traced.path.push_back(start);
                traced.total_weight = 0;
                traced.hops = 0;
                return traced;
            }
            std::vector<Node> nodes;
            std::vector<Edge> edges;
            Node v = end;
            nodes.push_back(v);
            for (int steps = 0; v != start && steps <= nodeCount(); ++steps) {
                Arc a = predecessor(v);
                if (a == INVALID) return {};
                edges.push_back(Edge(a));
                v = base().source(a);
                nodes.push_back(v);
            }
            if (v != start) return {};
            std::reverse(nodes.begin(), nodes.end());
            std::reverse(edges.begin(), edges.end());
            double sum = 0;
            for (Edge e : edges) sum += weight(e);
            traced.status = PathStatus::Found;
            traced.found = true;
            traced.path = std::move(nodes);
            traced.arcs = std::move(edges);
            traced.total_weight = sum;
            traced.hops = static_cast<int>(traced.arcs.size());
            return traced;
        };

        typename UndirectedBase::template ArcMap<double> weight_map(base());
        for (typename UndirectedBase::EdgeIt e(base()); e != INVALID; ++e) {
            weight_map[UndirectedBase::direct(e, true)] = weight(e);
            weight_map[UndirectedBase::direct(e, false)] = weight(e);
        }

        auto reachable_negative = [&]() {
            lemon::Bfs<UndirectedBase> bfs(base());
            bfs.run(start);
            for (typename UndirectedBase::EdgeIt e(base()); e != INVALID; ++e) {
                if (weight(e) < 0 && (bfs.reached(base().u(e)) || bfs.reached(base().v(e)))) return true;
            }
            return false;
        };

        if (algo == ShortestPathAlgorithm::Dijkstra || algo == ShortestPathAlgorithm::AStar) {
            if (reachable_negative()) {
                result.status = PathStatus::NegativeWeight;
                return result;
            }
        }

        if (algo == ShortestPathAlgorithm::BFS) {
            if (start == end) return trace([](Node) { return INVALID; });
            lemon::Bfs<UndirectedBase> bfs(base());
            bfs.run(start, end);
            if (!bfs.reached(end)) return result;
            return trace([&](Node v) { return bfs.predArc(v); });
        }

        if (algo == ShortestPathAlgorithm::AStar) {
            if (!heuristic) heuristic = [](const NodeProps &, const NodeProps &) { return 0.0; };

            struct OpenItem {
                double f;
                double g;
                Node node;

                bool operator>(const OpenItem &other) const {
                    return f > other.f;
                }
            };

            std::priority_queue<OpenItem, std::vector<OpenItem>, std::greater<OpenItem>> open;
            typename UndirectedBase::template NodeMap<double> g_score(base(), std::numeric_limits<double>::max());
            typename UndirectedBase::template NodeMap<Arc> parent(base(), INVALID);
            g_score[start] = 0;
            open.push({heuristic(core->nodeProps[start], core->nodeProps[end]), 0, start});
            while (!open.empty()) {
                OpenItem current = open.top();
                open.pop();
                if (current.g > g_score[current.node]) continue;
                if (current.node == end) return trace([&](Node v) { return parent[v]; });
                for (typename UndirectedBase::OutArcIt a(base(), current.node); a != INVALID; ++a) {
                    Node neighbor = base().target(a);
                    double tentative = current.g + weight(Edge(a));
                    if (tentative < g_score[neighbor]) {
                        parent[neighbor] = a;
                        g_score[neighbor] = tentative;
                        double f = tentative + heuristic(core->nodeProps[neighbor], core->nodeProps[end]);
                        open.push({f, tentative, neighbor});
                    }
                }
            }
            return result;
        }

        if (algo == ShortestPathAlgorithm::Dijkstra) {
            lemon::Dijkstra<UndirectedBase, decltype(weight_map)> dijkstra(base(), weight_map);
            if (!dijkstra.run(start, end)) return result;
            return trace([&](Node v) { return dijkstra.predArc(v); });
        }

        lemon::BellmanFord<UndirectedBase, decltype(weight_map)> bf(base(), weight_map);
        bf.init();
        bf.addSource(start);
        if (!bf.checkedStart()) {
            result.status = PathStatus::NegativeCycle;
            return result;
        }
        if (!bf.reached(end)) return result;
        return trace([&](Node v) { return bf.predArc(v); });
    }

    template <typename NodeProps>
    typename UndirectedGraph<NodeProps>::Tree
    UndirectedGraph<NodeProps>::shortestPathTree(Node source, ShortestPathAlgorithm algo) const {
        using Arc = typename UndirectedBase::Arc;
        Tree tree;
        tree.source = source;
        if (!valid(source)) {
            tree.status = PathStatus::InvalidNodes;
            return tree;
        }
        if (algo == ShortestPathAlgorithm::AStar) {
            tree.status = PathStatus::Unsupported;
            return tree;
        }

        typename UndirectedBase::template ArcMap<double> weight_map(base());
        for (typename UndirectedBase::EdgeIt e(base()); e != INVALID; ++e) {
            weight_map[UndirectedBase::direct(e, true)] = weight(e);
            weight_map[UndirectedBase::direct(e, false)] = weight(e);
        }

        auto reachable_negative = [&]() {
            lemon::Bfs<UndirectedBase> bfs(base());
            bfs.run(source);
            for (typename UndirectedBase::EdgeIt e(base()); e != INVALID; ++e) {
                if (weight(e) < 0 && (bfs.reached(base().u(e)) || bfs.reached(base().v(e)))) return true;
            }
            return false;
        };

        if (algo == ShortestPathAlgorithm::BFS) {
            lemon::Bfs<UndirectedBase> bfs(base());
            bfs.run(source);
            for (typename UndirectedBase::NodeIt n(base()); n != INVALID; ++n) {
                if (!bfs.reached(n)) continue;
                tree.distance[n] = static_cast<double>(bfs.dist(n));
                if (n != source) tree.predecessor[n] = Edge(bfs.predArc(n));
            }
            tree.status = PathStatus::Found;
            return tree;
        }

        if (algo == ShortestPathAlgorithm::Dijkstra) {
            if (reachable_negative()) {
                tree.status = PathStatus::NegativeWeight;
                return tree;
            }
            lemon::Dijkstra<UndirectedBase, decltype(weight_map)> dijkstra(base(), weight_map);
            dijkstra.run(source);
            for (typename UndirectedBase::NodeIt n(base()); n != INVALID; ++n) {
                if (!dijkstra.reached(n)) continue;
                tree.distance[n] = dijkstra.dist(n);
                if (n != source) tree.predecessor[n] = Edge(dijkstra.predArc(n));
            }
            tree.status = PathStatus::Found;
            return tree;
        }

        lemon::BellmanFord<UndirectedBase, decltype(weight_map)> bf(base(), weight_map);
        bf.init();
        bf.addSource(source);
        if (!bf.checkedStart()) {
            tree.status = PathStatus::NegativeCycle;
            return tree;
        }
        for (typename UndirectedBase::NodeIt n(base()); n != INVALID; ++n) {
            if (!bf.reached(n)) continue;
            tree.distance[n] = bf.dist(n);
            if (n != source && bf.predArc(n) != INVALID) tree.predecessor[n] = Edge(bf.predArc(n));
        }
        tree.status = PathStatus::Found;
        return tree;
    }

    template <typename NodeProps>
    AllPairsShortestPathResult<typename UndirectedGraph<NodeProps>::KeyType>
    UndirectedGraph<NodeProps>::getFloydWarshallMatrix() const {
        AllPairsShortestPathResult<KeyType> res;
        std::unordered_map<int, std::size_t> id_to_idx;
        std::size_t idx = 0;
        for (typename UndirectedBase::NodeIt n(base()); n != INVALID; ++n) {
            res.node_keys.push_back(getKey(core->nodeProps[n]));
            id_to_idx[UndirectedBase::id(n)] = idx++;
        }
        const std::size_t n = res.node_keys.size();
        const double inf = infinity();
        res.dist_matrix.assign(n, std::vector<double>(n, inf));
        res.next.assign(n, std::vector<int>(n, -1));
        for (std::size_t i = 0; i < n; ++i) {
            res.dist_matrix[i][i] = 0;
            res.next[i][i] = static_cast<int>(i);
        }
        auto relax_edge = [&](std::size_t u, std::size_t v, double w) {
            if (w < res.dist_matrix[u][v]) {
                res.dist_matrix[u][v] = w;
                res.next[u][v] = static_cast<int>(v);
            }
        };
        for (typename UndirectedBase::EdgeIt e(base()); e != INVALID; ++e) {
            std::size_t u = id_to_idx[UndirectedBase::id(base().u(e))];
            std::size_t v = id_to_idx[UndirectedBase::id(base().v(e))];
            relax_edge(u, v, weight(e));
            relax_edge(v, u, weight(e));
        }
        for (std::size_t k = 0; k < n; ++k) {
            for (std::size_t i = 0; i < n; ++i) {
                if (res.dist_matrix[i][k] == inf) continue;
                for (std::size_t j = 0; j < n; ++j) {
                    if (res.dist_matrix[k][j] == inf) continue;
                    double through = res.dist_matrix[i][k] + res.dist_matrix[k][j];
                    if (through < res.dist_matrix[i][j]) {
                        res.dist_matrix[i][j] = through;
                        res.next[i][j] = res.next[i][k];
                    }
                }
            }
        }
        for (std::size_t i = 0; i < n; ++i) {
            if (res.dist_matrix[i][i] < 0) res.has_negative_cycle = true;
        }
        return res;
    }

    template <typename NodeProps>
    typename UndirectedGraph<NodeProps>::Forest UndirectedGraph<NodeProps>::findMinimumSpanningForest() const {
        Forest result;
        typename UndirectedBase::template EdgeMap<double> weight_map(base());
        typename UndirectedBase::template EdgeMap<bool> chosen(base(), false);
        for (typename UndirectedBase::EdgeIt e(base()); e != INVALID; ++e) weight_map[e] = weight(e);
        lemon::kruskal(base(), weight_map, chosen);
        for (typename UndirectedBase::EdgeIt e(base()); e != INVALID; ++e) {
            if (!chosen[e]) continue;
            result.edges.push_back(e);
            result.total_weight += weight(e);
        }
        typename UndirectedBase::template NodeMap<int> comp(base());
        result.component_count = nodeCount() == 0 ? 0 : lemon::connectedComponents(base(), comp);
        result.is_tree = nodeCount() == 0 || result.component_count == 1;
        return result;
    }

    template <typename NodeProps>
    typename UndirectedGraph<NodeProps>::Components UndirectedGraph<NodeProps>::connectedComponents() const {
        Components result;
        if (nodeCount() == 0) return result;
        typename UndirectedBase::template NodeMap<int> comp(base());
        result.count = lemon::connectedComponents(base(), comp);
        result.components.resize(static_cast<std::size_t>(result.count));
        for (typename UndirectedBase::NodeIt n(base()); n != INVALID; ++n) result.components[comp[n]].push_back(n);
        return result;
    }

    template <typename NodeProps>
    typename UndirectedGraph<NodeProps>::Flow UndirectedGraph<NodeProps>::findMaxFlow(Node source_node, Node sink_node) const {
        Flow result;
        if (!valid(source_node) || !valid(sink_node)) return result;
        for (typename UndirectedBase::EdgeIt e(base()); e != INVALID; ++e) {
            if (capacity(e) < 0) return result;
        }
        if (source_node == sink_node) {
            result.feasible = true;
            result.max_flow_value = 0;
            return result;
        }

        lemon::ListDigraph flow_graph;
        typename UndirectedBase::template NodeMap<lemon::ListDigraph::Node> image(base());
        for (typename UndirectedBase::NodeIt n(base()); n != INVALID; ++n) image[n] = flow_graph.addNode();
        typename UndirectedBase::template EdgeMap<lemon::ListDigraph::Arc> forward(base());
        typename UndirectedBase::template EdgeMap<lemon::ListDigraph::Arc> backward(base());
        lemon::ListDigraph::ArcMap<double> capacity_map(flow_graph);
        for (typename UndirectedBase::EdgeIt e(base()); e != INVALID; ++e) {
            auto uv = flow_graph.addArc(image[base().u(e)], image[base().v(e)]);
            auto vu = flow_graph.addArc(image[base().v(e)], image[base().u(e)]);
            capacity_map[uv] = capacity(e);
            capacity_map[vu] = capacity(e);
            forward[e] = uv;
            backward[e] = vu;
        }
        lemon::Preflow<lemon::ListDigraph, decltype(capacity_map)> preflow(
            flow_graph, capacity_map, image[source_node], image[sink_node]);
        preflow.run();
        result.feasible = true;
        result.max_flow_value = preflow.flowValue();
        for (typename UndirectedBase::EdgeIt e(base()); e != INVALID; ++e) {
            result.flow[e] = preflow.flow(forward[e]) - preflow.flow(backward[e]);
        }
        return result;
    }

    template <typename NodeProps>
    bool UndirectedGraph<NodeProps>::isConnected() const {
        if (nodeCount() <= 1) return true;
        return connectedComponents().count == 1;
    }

    template <typename NodeProps>
    bool UndirectedGraph<NodeProps>::isEulerian() const {
        typename UndirectedBase::template NodeMap<int> comp(base(), 0);
        int count = nodeCount() == 0 ? 0 : lemon::connectedComponents(base(), comp);
        std::vector<bool> has_edge(static_cast<std::size_t>(std::max(count, 0)), false);
        for (typename UndirectedBase::EdgeIt e(base()); e != INVALID; ++e) has_edge[comp[base().u(e)]] = true;
        int nontrivial = 0;
        for (bool value : has_edge)
            if (value) ++nontrivial;
        if (nontrivial > 1) return false;
        for (typename UndirectedBase::NodeIt n(base()); n != INVALID; ++n) {
            if (lemon::countIncEdges(base(), n) % 2 != 0) return false;
        }
        return true;
    }

    template <typename NodeProps>
    typename UndirectedGraph<NodeProps>::Postman UndirectedGraph<NodeProps>::chinesePostman() const {
        using Arc = typename UndirectedBase::Arc;
        Postman result;
        if (!isEulerian() && edgeCount() > 0) {
            typename UndirectedBase::template NodeMap<int> comp(base(), 0);
            int count = lemon::connectedComponents(base(), comp);
            std::vector<bool> has_edge(static_cast<std::size_t>(std::max(count, 0)), false);
            for (typename UndirectedBase::EdgeIt e(base()); e != INVALID; ++e) has_edge[comp[base().u(e)]] = true;
            int nontrivial = 0;
            for (bool value : has_edge)
                if (value) ++nontrivial;
            if (nontrivial > 1) {
                result.status = PostmanStatus::Disconnected;
                return result;
            }
        }

        for (typename UndirectedBase::EdgeIt e(base()); e != INVALID; ++e) {
            if (cost(e) < 0) {
                result.status = PostmanStatus::NegativeCost;
                return result;
            }
        }

        auto closed_cost = [&](std::vector<Edge> tour) {
            double sum = 0;
            for (Edge e : tour) sum += cost(e);
            result.tour = std::move(tour);
            result.total_cost = sum;
            result.status = PostmanStatus::Found;
        };

        if (isEulerian()) {
            std::vector<Edge> tour;
            for (lemon::EulerIt<UndirectedBase> it(base()); it != INVALID; ++it) {
                Arc arc = it;
                tour.push_back(Edge(arc));
            }
            if (static_cast<int>(tour.size()) != edgeCount()) {
                result.status = PostmanStatus::Infeasible;
                return result;
            }
            closed_cost(std::move(tour));
            return result;
        }

        std::vector<Node> odd;
        for (typename UndirectedBase::NodeIt n(base()); n != INVALID; ++n) {
            if (lemon::countIncEdges(base(), n) % 2 != 0) odd.push_back(n);
        }

        typename UndirectedBase::template ArcMap<double> arc_cost(base());
        for (typename UndirectedBase::EdgeIt e(base()); e != INVALID; ++e) {
            arc_cost[UndirectedBase::direct(e, true)] = cost(e);
            arc_cost[UndirectedBase::direct(e, false)] = cost(e);
        }

        struct Search {
            std::unique_ptr<typename UndirectedBase::template NodeMap<double>> dist;
            std::unique_ptr<typename UndirectedBase::template NodeMap<Arc>> pred;
        };

        std::vector<Search> searches(odd.size());
        for (std::size_t i = 0; i < odd.size(); ++i) {
            lemon::Dijkstra<UndirectedBase, decltype(arc_cost)> dijkstra(base(), arc_cost);
            dijkstra.run(odd[i]);
            searches[i].dist = std::make_unique<typename UndirectedBase::template NodeMap<double>>(base(), infinity());
            searches[i].pred = std::make_unique<typename UndirectedBase::template NodeMap<Arc>>(base(), INVALID);
            for (typename UndirectedBase::NodeIt n(base()); n != INVALID; ++n) {
                if (!dijkstra.reached(n)) continue;
                (*searches[i].dist)[n] = dijkstra.dist(n);
                (*searches[i].pred)[n] = dijkstra.predArc(n);
            }
        }

        lemon::ListGraph match;
        std::vector<lemon::ListGraph::Node> match_nodes;
        for (std::size_t i = 0; i < odd.size(); ++i) match_nodes.push_back(match.addNode());
        lemon::ListGraph::EdgeMap<double> match_weight(match);
        lemon::ListGraph::EdgeMap<std::pair<int, int>> match_ends(match);
        for (std::size_t i = 0; i < odd.size(); ++i) {
            for (std::size_t j = i + 1; j < odd.size(); ++j) {
                double distance = (*searches[i].dist)[odd[j]];
                if (distance == infinity()) {
                    result.status = PostmanStatus::Infeasible;
                    return result;
                }
                auto edge = match.addEdge(match_nodes[i], match_nodes[j]);
                match_weight[edge] = -distance;
                match_ends[edge] = {static_cast<int>(i), static_cast<int>(j)};
            }
        }

        lemon::MaxWeightedPerfectMatching<lemon::ListGraph, lemon::ListGraph::EdgeMap<double>> matching(match, match_weight);
        matching.run();

        typename UndirectedBase::template EdgeMap<int> extra(base(), 0);
        for (lemon::ListGraph::EdgeIt e(match); e != INVALID; ++e) {
            if (!matching.matching(e)) continue;
            int i = match_ends[e].first;
            int j = match_ends[e].second;
            Node cur = odd[j];
            int guard = nodeCount();
            while (cur != odd[i] && guard-- > 0) {
                Arc arc = (*searches[i].pred)[cur];
                if (arc == INVALID) {
                    result.status = PostmanStatus::Infeasible;
                    return result;
                }
                extra[Edge(arc)] += 1;
                cur = base().source(arc);
            }
            if (cur != odd[i]) {
                result.status = PostmanStatus::Infeasible;
                return result;
            }
        }

        lemon::ListGraph multi;
        typename UndirectedBase::template NodeMap<lemon::ListGraph::Node> image(base(), INVALID);
        auto node_of = [&](Node n) {
            if (image[n] == INVALID) image[n] = multi.addNode();
            return image[n];
        };
        lemon::ListGraph::EdgeMap<Edge> back(multi);
        for (typename UndirectedBase::EdgeIt e(base()); e != INVALID; ++e) {
            int copies = 1 + extra[e];
            for (int c = 0; c < copies; ++c) {
                auto added = multi.addEdge(node_of(base().u(e)), node_of(base().v(e)));
                back[added] = e;
            }
        }

        std::vector<Edge> tour;
        for (lemon::EulerIt<lemon::ListGraph> it(multi); it != INVALID; ++it) {
            lemon::ListGraph::Arc arc = it;
            tour.push_back(back[lemon::ListGraph::Edge(arc)]);
        }
        if (static_cast<int>(tour.size()) != lemon::countEdges(multi)) {
            result.status = PostmanStatus::Infeasible;
            return result;
        }
        closed_cost(std::move(tour));
        return result;
    }

} // namespace graph
