#pragma once

#include <lemon/bellman_ford.h>
#include <lemon/bfs.h>
#include <lemon/capacity_scaling.h>
#include <lemon/connectivity.h>
#include <lemon/core.h>
#include <lemon/dijkstra.h>
#include <lemon/euler.h>
#include <lemon/kruskal.h>
#include <lemon/matching.h>
#include <lemon/min_cost_arborescence.h>
#include <lemon/preflow.h>

#include <algorithm>
#include <queue>
#include <stdexcept>
#include <utility>

namespace graph {

    namespace {

        double infinity() {
            return std::numeric_limits<double>::infinity();
        }

    } // namespace

    template <typename NodeProps>
    Graph<NodeProps>::Graph()
        : core(std::make_unique<Core>()) {}

    template <typename NodeProps>
    Graph<NodeProps>::Graph(const Graph &other)
        : core(std::make_unique<Core>()) {
        typename GraphBase::template NodeMap<Node> node_ref(other.base());
        typename GraphBase::template ArcMap<Connection> arc_ref(other.base());
        lemon::DigraphCopy<GraphBase, GraphBase> copy(other.base(), base());
        copy.nodeRef(node_ref).arcRef(arc_ref);
        copy.nodeMap(other.core->nodeProps, core->nodeProps);
        copy.arcMap(other.core->connectionProps, core->connectionProps);
        copy.run();
        for (const auto &entry : other.keyToNode) {
            keyToNode.emplace(entry.first, node_ref[entry.second]);
        }
    }

    template <typename NodeProps>
    Graph<NodeProps>::Graph(Graph &&other)
        : core(std::move(other.core)),
          keyToNode(std::move(other.keyToNode)) {
        other.core = std::make_unique<Core>();
        other.keyToNode.clear();
    }

    template <typename NodeProps>
    Graph<NodeProps> &Graph<NodeProps>::operator=(const Graph &other) {
        if (this == &other) return *this;
        Graph copy(other);
        return *this = std::move(copy);
    }

    template <typename NodeProps>
    Graph<NodeProps> &Graph<NodeProps>::operator=(Graph &&other) {
        if (this == &other) return *this;
        core = std::move(other.core);
        keyToNode = std::move(other.keyToNode);
        other.core = std::make_unique<Core>();
        other.keyToNode.clear();
        return *this;
    }

    template <typename NodeProps>
    Graph<NodeProps>::~Graph() = default;

    template <typename NodeProps>
    GraphBase &Graph<NodeProps>::base() {
        return core->digraph;
    }

    template <typename NodeProps>
    const GraphBase &Graph<NodeProps>::base() const {
        return core->digraph;
    }

    template <typename NodeProps>
    typename Graph<NodeProps>::KeyType Graph<NodeProps>::getKey(const NodeProps &props) const {
        return KeyExtractor<NodeProps>::get(props);
    }

    template <typename NodeProps>
    Node Graph<NodeProps>::addNode(NodeProps props) {
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
    Connection Graph<NodeProps>::addConnection(Node u, Node v, double weight) {
        if (!valid(u) || !valid(v)) {
            throw std::invalid_argument("addConnection: invalid node");
        }
        Connection existing = findConnection(u, v);
        if (existing != INVALID) {
            core->connectionProps[existing].weight = weight;
            return existing;
        }
        Connection a = base().addArc(u, v);
        core->connectionProps[a] = ConnectionData{weight, weight, weight};
        return a;
    }

    template <typename NodeProps>
    void Graph<NodeProps>::updateNode(Node n, NodeProps props) {
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
    void Graph<NodeProps>::removeNode(Node n) {
        if (!valid(n)) return;
        keyToNode.erase(getKey(core->nodeProps[n]));
        base().erase(n);
    }

    template <typename NodeProps>
    void Graph<NodeProps>::removeConnection(Connection a) {
        if (!valid(a)) return;
        base().erase(a);
    }

    template <typename NodeProps>
    Node Graph<NodeProps>::getNodeByKey(const KeyType &key) const {
        auto it = keyToNode.find(key);
        return it != keyToNode.end() ? it->second : INVALID;
    }

    template <typename NodeProps>
    bool Graph<NodeProps>::hasNodeKey(const KeyType &key) const {
        return keyToNode.find(key) != keyToNode.end();
    }

    template <typename NodeProps>
    const NodeProps &Graph<NodeProps>::operator[](Node n) const {
        return core->nodeProps[n];
    }

    template <typename NodeProps>
    double &Graph<NodeProps>::operator[](Connection a) {
        return core->connectionProps[a].weight;
    }

    template <typename NodeProps>
    const double &Graph<NodeProps>::operator[](Connection a) const {
        return core->connectionProps[a].weight;
    }

    template <typename NodeProps>
    double Graph<NodeProps>::weight(Connection a) const {
        return core->connectionProps[a].weight;
    }

    template <typename NodeProps>
    double Graph<NodeProps>::capacity(Connection a) const {
        return core->connectionProps[a].capacity;
    }

    template <typename NodeProps>
    double Graph<NodeProps>::cost(Connection a) const {
        return core->connectionProps[a].cost;
    }

    template <typename NodeProps>
    void Graph<NodeProps>::setWeight(Connection a, double value) {
        core->connectionProps[a].weight = value;
    }

    template <typename NodeProps>
    void Graph<NodeProps>::setCapacity(Connection a, double value) {
        core->connectionProps[a].capacity = value;
    }

    template <typename NodeProps>
    void Graph<NodeProps>::setCost(Connection a, double value) {
        core->connectionProps[a].cost = value;
    }

    template <typename NodeProps>
    bool Graph<NodeProps>::hasReverse(Connection a) const {
        return lemon::findArc(base(), base().target(a), base().source(a)) != INVALID;
    }

    template <typename NodeProps>
    bool Graph<NodeProps>::valid(Node n) const {
        return base().valid(n);
    }

    template <typename NodeProps>
    bool Graph<NodeProps>::valid(Connection a) const {
        return base().valid(a);
    }

    template <typename NodeProps>
    Node Graph<NodeProps>::source(Connection a) const {
        return base().source(a);
    }

    template <typename NodeProps>
    Node Graph<NodeProps>::target(Connection a) const {
        return base().target(a);
    }

    template <typename NodeProps>
    const GraphBase &Graph<NodeProps>::lemonGraph() const {
        return base();
    }

    template <typename NodeProps>
    Connection Graph<NodeProps>::findConnection(Node u, Node v) const {
        return lemon::findArc(base(), u, v);
    }

    template <typename NodeProps>
    int Graph<NodeProps>::nodeCount() const {
        return lemon::countNodes(base());
    }

    template <typename NodeProps>
    int Graph<NodeProps>::arcCount() const {
        return lemon::countArcs(base());
    }

    template <typename NodeProps>
    template <typename Func>
    void Graph<NodeProps>::forEachNode(Func func) const {
        for (NodeIt n(base()); n != INVALID; ++n) func(n, core->nodeProps[n]);
    }

    template <typename NodeProps>
    template <typename Func>
    void Graph<NodeProps>::forEachConnection(Func func) const {
        for (ConnectionIt a(base()); a != INVALID; ++a) func(a, core->connectionProps[a]);
    }

    template <typename NodeProps>
    typename Graph<NodeProps>::MatrixRepresentation Graph<NodeProps>::getAdjacencyMatrix(double no_edge_val) const {
        MatrixRepresentation result;
        std::unordered_map<int, std::size_t> node_id_to_index;
        std::size_t index = 0;
        for (NodeIt n(base()); n != INVALID; ++n) {
            result.node_keys.push_back(getKey(core->nodeProps[n]));
            node_id_to_index[GraphBase::id(n)] = index++;
        }
        const std::size_t n = result.node_keys.size();
        result.matrix.assign(n, std::vector<double>(n, no_edge_val));
        for (std::size_t i = 0; i < n; ++i) result.matrix[i][i] = 0;
        for (ConnectionIt a(base()); a != INVALID; ++a) {
            std::size_t u = node_id_to_index[GraphBase::id(base().source(a))];
            std::size_t v = node_id_to_index[GraphBase::id(base().target(a))];
            result.matrix[u][v] = core->connectionProps[a].weight;
        }
        return result;
    }

    template <typename NodeProps>
    typename Graph<NodeProps>::Path Graph<NodeProps>::findShortestPath(Node start,
                                                                       Node end,
                                                                       ShortestPathAlgorithm algo,
                                                                       HeuristicFn heuristic) const {
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
            std::vector<Connection> arcs;
            Node v = end;
            nodes.push_back(v);
            for (int steps = 0; v != start && steps <= nodeCount(); ++steps) {
                Connection a = predecessor(v);
                if (a == INVALID) return {};
                arcs.push_back(a);
                v = base().source(a);
                nodes.push_back(v);
            }
            if (v != start) return {};
            std::reverse(nodes.begin(), nodes.end());
            std::reverse(arcs.begin(), arcs.end());
            double sum = 0;
            for (Connection a : arcs) sum += weight(a);
            traced.status = PathStatus::Found;
            traced.found = true;
            traced.path = std::move(nodes);
            traced.arcs = std::move(arcs);
            traced.total_weight = sum;
            traced.hops = static_cast<int>(traced.arcs.size());
            return traced;
        };

        auto reachable_negative = [&]() {
            lemon::Bfs<GraphBase> bfs(base());
            bfs.run(start);
            for (ConnectionIt a(base()); a != INVALID; ++a) {
                if (weight(a) < 0 && bfs.reached(base().source(a))) return true;
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
            lemon::Bfs<GraphBase> bfs(base());
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
            typename GraphBase::template NodeMap<double> g_score(base(), std::numeric_limits<double>::max());
            typename GraphBase::template NodeMap<Connection> parent(base(), INVALID);
            g_score[start] = 0;
            open.push({heuristic(core->nodeProps[start], core->nodeProps[end]), 0, start});
            while (!open.empty()) {
                OpenItem current = open.top();
                open.pop();
                if (current.g > g_score[current.node]) continue;
                if (current.node == end) return trace([&](Node v) { return parent[v]; });
                for (OutConnectionIt a(base(), current.node); a != INVALID; ++a) {
                    Node neighbor = base().target(a);
                    double tentative = current.g + weight(a);
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

        typename GraphBase::template ArcMap<double> weight_map(base());
        for (ConnectionIt a(base()); a != INVALID; ++a) weight_map[a] = weight(a);

        if (algo == ShortestPathAlgorithm::Dijkstra) {
            lemon::Dijkstra<GraphBase, decltype(weight_map)> dijkstra(base(), weight_map);
            if (!dijkstra.run(start, end)) return result;
            return trace([&](Node v) { return dijkstra.predArc(v); });
        }

        lemon::BellmanFord<GraphBase, decltype(weight_map)> bf(base(), weight_map);
        bf.init();
        bf.addSource(start);
        if (!bf.checkedStart()) {
            lemon::Path<GraphBase> cycle = bf.negativeCycle();
            bool reaches = cycle.empty();
            if (!reaches) {
                std::unordered_map<Node, bool, LemonIdHash<GraphBase, Node>> on_cycle;
                for (int i = 0; i < cycle.length(); ++i) {
                    Connection a = cycle.nth(i);
                    on_cycle[base().source(a)] = true;
                    on_cycle[base().target(a)] = true;
                }
                lemon::Bfs<GraphBase> reach(base());
                reach.init();
                for (const auto &entry : on_cycle) reach.addSource(entry.first);
                reach.start();
                reaches = reach.reached(end);
            }
            if (reaches) {
                result.status = PathStatus::NegativeCycle;
                return result;
            }
        }
        if (!bf.reached(end)) return result;
        return trace([&](Node v) { return bf.predArc(v); });
    }

    template <typename NodeProps>
    typename Graph<NodeProps>::Tree Graph<NodeProps>::shortestPathTree(Node source, ShortestPathAlgorithm algo) const {
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

        auto reachable_negative = [&]() {
            lemon::Bfs<GraphBase> bfs(base());
            bfs.run(source);
            for (ConnectionIt a(base()); a != INVALID; ++a) {
                if (weight(a) < 0 && bfs.reached(base().source(a))) return true;
            }
            return false;
        };

        if (algo == ShortestPathAlgorithm::BFS) {
            lemon::Bfs<GraphBase> bfs(base());
            bfs.run(source);
            for (NodeIt n(base()); n != INVALID; ++n) {
                if (!bfs.reached(n)) continue;
                tree.distance[n] = static_cast<double>(bfs.dist(n));
                if (n != source) tree.predecessor[n] = bfs.predArc(n);
            }
            tree.status = PathStatus::Found;
            return tree;
        }

        typename GraphBase::template ArcMap<double> weight_map(base());
        for (ConnectionIt a(base()); a != INVALID; ++a) weight_map[a] = weight(a);

        if (algo == ShortestPathAlgorithm::Dijkstra) {
            if (reachable_negative()) {
                tree.status = PathStatus::NegativeWeight;
                return tree;
            }
            lemon::Dijkstra<GraphBase, decltype(weight_map)> dijkstra(base(), weight_map);
            dijkstra.run(source);
            for (NodeIt n(base()); n != INVALID; ++n) {
                if (!dijkstra.reached(n)) continue;
                tree.distance[n] = dijkstra.dist(n);
                if (n != source) tree.predecessor[n] = dijkstra.predArc(n);
            }
            tree.status = PathStatus::Found;
            return tree;
        }

        lemon::BellmanFord<GraphBase, decltype(weight_map)> bf(base(), weight_map);
        bf.init();
        bf.addSource(source);
        if (!bf.checkedStart()) {
            tree.status = PathStatus::NegativeCycle;
            return tree;
        }
        for (NodeIt n(base()); n != INVALID; ++n) {
            if (!bf.reached(n)) continue;
            tree.distance[n] = bf.dist(n);
            if (n != source) tree.predecessor[n] = bf.predArc(n);
        }
        tree.status = PathStatus::Found;
        return tree;
    }

    template <typename NodeProps>
    AllPairsShortestPathResult<typename Graph<NodeProps>::KeyType> Graph<NodeProps>::getFloydWarshallMatrix() const {
        AllPairsShortestPathResult<KeyType> res;
        std::unordered_map<int, std::size_t> id_to_idx;
        std::size_t idx = 0;
        for (NodeIt n(base()); n != INVALID; ++n) {
            res.node_keys.push_back(getKey(core->nodeProps[n]));
            id_to_idx[GraphBase::id(n)] = idx++;
        }
        const std::size_t n = res.node_keys.size();
        const double inf = infinity();
        res.dist_matrix.assign(n, std::vector<double>(n, inf));
        res.next.assign(n, std::vector<int>(n, -1));
        for (std::size_t i = 0; i < n; ++i) {
            res.dist_matrix[i][i] = 0;
            res.next[i][i] = static_cast<int>(i);
        }
        for (ConnectionIt a(base()); a != INVALID; ++a) {
            std::size_t u = id_to_idx[GraphBase::id(base().source(a))];
            std::size_t v = id_to_idx[GraphBase::id(base().target(a))];
            if (weight(a) < res.dist_matrix[u][v]) {
                res.dist_matrix[u][v] = weight(a);
                res.next[u][v] = static_cast<int>(v);
            }
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
    ArborescenceResult Graph<NodeProps>::findMinimumSpanningArborescence(Node root) const {
        ArborescenceResult result;
        if (!valid(root)) return result;
        typename GraphBase::template ArcMap<double> weight_map(base());
        for (ConnectionIt a(base()); a != INVALID; ++a) weight_map[a] = weight(a);
        lemon::MinCostArborescence<GraphBase, decltype(weight_map)> mca(base(), weight_map);
        mca.run(root);
        bool spans = true;
        for (NodeIt n(base()); n != INVALID; ++n) {
            if (n != root && !mca.reached(n)) spans = false;
        }
        for (ConnectionIt a(base()); a != INVALID; ++a) {
            if (!mca.arborescence(a)) continue;
            result.arcs.push_back(a);
            result.total_weight += weight(a);
        }
        result.spans_all = spans && (nodeCount() <= 1 || static_cast<int>(result.arcs.size()) + 1 == nodeCount());
        return result;
    }

    template <typename NodeProps>
    typename Graph<NodeProps>::Flow Graph<NodeProps>::findMaxFlow(Node source_node, Node sink_node) const {
        Flow result;
        if (!valid(source_node) || !valid(sink_node)) return result;
        for (ConnectionIt a(base()); a != INVALID; ++a) {
            if (capacity(a) < 0) return result;
        }
        if (source_node == sink_node) {
            result.feasible = true;
            result.max_flow_value = 0;
            return result;
        }
        typename GraphBase::template ArcMap<double> capacity_map(base());
        for (ConnectionIt a(base()); a != INVALID; ++a) capacity_map[a] = capacity(a);
        lemon::Preflow<GraphBase, decltype(capacity_map)> preflow(base(), capacity_map, source_node, sink_node);
        preflow.run();
        result.feasible = true;
        result.max_flow_value = preflow.flowValue();
        for (ConnectionIt a(base()); a != INVALID; ++a) result.flow[a] = preflow.flow(a);
        return result;
    }

    template <typename NodeProps>
    bool Graph<NodeProps>::isAcyclic() const {
        typename GraphBase::template NodeMap<int> order(base());
        return lemon::checkedTopologicalSort(base(), order);
    }

    template <typename NodeProps>
    std::vector<Node> Graph<NodeProps>::getTopologicalOrder() const {
        typename GraphBase::template NodeMap<int> order(base());
        if (!lemon::checkedTopologicalSort(base(), order)) return {};
        std::vector<Node> sorted_nodes(static_cast<std::size_t>(nodeCount()));
        for (NodeIt n(base()); n != INVALID; ++n) sorted_nodes[order[n]] = n;
        return sorted_nodes;
    }

    template <typename NodeProps>
    std::vector<Connection> Graph<NodeProps>::findCycle() const {
        std::vector<Connection> cycle;
        typename GraphBase::template NodeMap<int> state(base(), 0);
        typename GraphBase::template NodeMap<Connection> parent_arc(base(), INVALID);
        std::function<bool(Node)> dfs = [&](Node u) -> bool {
            state[u] = 1;
            for (OutConnectionIt a(base(), u); a != INVALID; ++a) {
                Node v = base().target(a);
                if (state[v] == 1) {
                    cycle.push_back(a);
                    for (Node curr = u; curr != v; curr = base().source(parent_arc[curr])) {
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
        for (NodeIt n(base()); n != INVALID; ++n) {
            if (state[n] == 0 && dfs(n)) break;
        }
        return cycle;
    }

    template <typename NodeProps>
    bool Graph<NodeProps>::isWeaklyConnected() const {
        int nontrivial = 0;
        Node start = INVALID;
        for (NodeIt n(base()); n != INVALID; ++n) {
            if (lemon::countInArcs(base(), n) + lemon::countOutArcs(base(), n) > 0) {
                ++nontrivial;
                if (start == INVALID) start = n;
            }
        }
        if (nontrivial <= 1) return true;

        typename GraphBase::template NodeMap<bool> seen(base(), false);
        std::queue<Node> q;
        q.push(start);
        seen[start] = true;
        int reached = 0;
        while (!q.empty()) {
            Node u = q.front();
            q.pop();
            if (lemon::countInArcs(base(), u) + lemon::countOutArcs(base(), u) > 0) ++reached;
            for (OutConnectionIt a(base(), u); a != INVALID; ++a) {
                Node v = base().target(a);
                if (!seen[v]) {
                    seen[v] = true;
                    q.push(v);
                }
            }
            for (InConnectionIt a(base(), u); a != INVALID; ++a) {
                Node v = base().source(a);
                if (!seen[v]) {
                    seen[v] = true;
                    q.push(v);
                }
            }
        }
        return reached == nontrivial;
    }

    template <typename NodeProps>
    bool Graph<NodeProps>::isEulerian() const {
        if (!isWeaklyConnected()) return false;
        for (NodeIt n(base()); n != INVALID; ++n) {
            if (lemon::countInArcs(base(), n) != lemon::countOutArcs(base(), n)) return false;
        }
        return true;
    }

    template <typename NodeProps>
    typename Graph<NodeProps>::Postman Graph<NodeProps>::chinesePostman() const {
        Postman result;
        if (!isWeaklyConnected()) {
            result.status = PostmanStatus::Disconnected;
            return result;
        }

        auto tour_cost = [&](const std::vector<Connection> &tour) {
            double sum = 0;
            for (Connection a : tour) sum += cost(a);
            return sum;
        };

        auto closed = [&](const std::vector<Connection> &tour) {
            if (tour.empty()) return true;
            for (std::size_t i = 0; i < tour.size(); ++i) {
                Connection next = tour[(i + 1) % tour.size()];
                if (base().target(tour[i]) != base().source(next)) return false;
            }
            return true;
        };

        if (isEulerian()) {
            for (lemon::DiEulerIt<GraphBase> e(base()); e != INVALID; ++e) result.tour.push_back(e);
            if (static_cast<int>(result.tour.size()) != arcCount() || !closed(result.tour)) {
                result.tour.clear();
                result.status = PostmanStatus::Infeasible;
                return result;
            }
            result.total_cost = tour_cost(result.tour);
            result.status = PostmanStatus::Found;
            return result;
        }

        typename GraphBase::template ArcMap<double> cost_map(base());
        for (ConnectionIt a(base()); a != INVALID; ++a) cost_map[a] = cost(a);
        lemon::BellmanFord<GraphBase, decltype(cost_map)> cycle_check(base(), cost_map);
        cycle_check.init();
        for (NodeIt n(base()); n != INVALID; ++n) cycle_check.addSource(n);
        if (!cycle_check.checkedStart()) {
            result.status = PostmanStatus::Unbounded;
            return result;
        }

        typename GraphBase::template NodeMap<int> supply(base(), 0);
        int positive = 0;
        for (NodeIt n(base()); n != INVALID; ++n) {
            int value = lemon::countInArcs(base(), n) - lemon::countOutArcs(base(), n);
            supply[n] = value;
            if (value > 0) positive += value;
        }

        typename GraphBase::template ArcMap<int> upper(base(), std::max(positive, 1));
        lemon::CapacityScaling<GraphBase, int, double> scaling(base());
        auto solved = scaling.upperMap(upper).costMap(cost_map).supplyMap(supply).run();
        if (solved == lemon::CapacityScaling<GraphBase, int, double>::UNBOUNDED) {
            result.status = PostmanStatus::Unbounded;
            return result;
        }
        if (solved != lemon::CapacityScaling<GraphBase, int, double>::OPTIMAL) {
            result.status = PostmanStatus::Infeasible;
            return result;
        }

        lemon::ListDigraph multi;
        typename GraphBase::template NodeMap<lemon::ListDigraph::Node> image(base(), INVALID);
        auto node_of = [&](Node n) {
            if (image[n] == INVALID) image[n] = multi.addNode();
            return image[n];
        };
        lemon::ListDigraph::ArcMap<Connection> back(multi);
        for (ConnectionIt a(base()); a != INVALID; ++a) {
            int copies = 1 + scaling.flow(a);
            for (int i = 0; i < copies; ++i) {
                auto added = multi.addArc(node_of(base().source(a)), node_of(base().target(a)));
                back[added] = a;
            }
        }

        for (lemon::DiEulerIt<lemon::ListDigraph> e(multi); e != INVALID; ++e) result.tour.push_back(back[e]);
        if (static_cast<int>(result.tour.size()) != lemon::countArcs(multi) || !closed(result.tour)) {
            result.tour.clear();
            result.status = PostmanStatus::Infeasible;
            return result;
        }
        result.total_cost = tour_cost(result.tour);
        result.status = PostmanStatus::Found;
        return result;
    }

    template <typename NodeProps>
    typename Graph<NodeProps>::Components Graph<NodeProps>::stronglyConnectedComponents() const {
        Components result;
        typename GraphBase::template NodeMap<int> comp(base());
        result.count = lemon::stronglyConnectedComponents(base(), comp);
        result.components.resize(static_cast<std::size_t>(result.count));
        for (NodeIt n(base()); n != INVALID; ++n) result.components[comp[n]].push_back(n);
        return result;
    }

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
