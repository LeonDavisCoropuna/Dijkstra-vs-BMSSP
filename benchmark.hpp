#ifndef BENCHMARK_HPP
#define BENCHMARK_HPP

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <limits>
#include <algorithm>
#include <cmath>
#include <functional>
#include <random>
#include <deque>

using NodeID = int;
using Dist = double;
constexpr Dist INF = std::numeric_limits<Dist>::infinity();

struct Edge {
    NodeID to;
    Dist weight;
    
    Edge(NodeID t, Dist w) : to(t), weight(w) {}
};

class Graph {
private:
    std::unordered_map<NodeID, std::vector<Edge>> adj;
    
public:
    void addEdge(NodeID from, NodeID to, Dist weight) {
        adj[from].emplace_back(to, weight);
    }
    
    const std::vector<Edge>& outEdges(NodeID u) const {
        static const std::vector<Edge> empty;
        auto it = adj.find(u);
        return (it != adj.end()) ? it->second : empty;
    }
    
    const std::unordered_map<NodeID, std::vector<Edge>>& getAdj() const {
        return adj;
    }
};

class NodeSet {
private:
    std::unordered_set<NodeID> nodes;
    
public:
    void add(NodeID v) {
        nodes.insert(v);
    }
    
    bool has(NodeID v) const {
        return nodes.find(v) != nodes.end();
    }
    
    size_t size() const {
        return nodes.size();
    }
    
    std::vector<NodeID> toVector() const {
        return std::vector<NodeID>(nodes.begin(), nodes.end());
    }
    
    auto begin() const { return nodes.begin(); }
    auto end() const { return nodes.end(); }
};

// =============================================================================
// DIJKSTRA'S ALGORITHM
// =============================================================================

struct DijkstraNode {
    NodeID id;
    Dist dist;
    
    bool operator>(const DijkstraNode& other) const {
        return dist > other.dist;
    }
};

std::unordered_map<NodeID, Dist> dijkstra(const Graph& g, NodeID source) {
    std::unordered_map<NodeID, Dist> dist;
    std::unordered_set<NodeID> visited;
    
    for (const auto& [u, edges] : g.getAdj()) {
        dist[u] = INF;
        for (const auto& edge : edges) {
            dist[edge.to] = INF;
        }
    }
    
    dist[source] = 0;
    
    std::priority_queue<DijkstraNode, std::vector<DijkstraNode>, 
                        std::greater<DijkstraNode>> pq;
    pq.push({source, 0});
    
    while (!pq.empty()) {
        DijkstraNode current = pq.top();
        pq.pop();
        
        NodeID u = current.id;
        
        if (visited.count(u)) continue;
        visited.insert(u);
        
        for (const auto& edge : g.outEdges(u)) {
            NodeID v = edge.to;
            Dist alt = dist[u] + edge.weight;
            
            if (alt < dist[v]) {
                dist[v] = alt;
                if (!visited.count(v)) {
                    pq.push({v, alt});
                }
            }
        }
    }
    
    return dist;
}

// =============================================================================
// A* ALGORITHM
// =============================================================================

using HeuristicFunc = std::function<Dist(NodeID, NodeID)>;

struct AStarNode {
    NodeID id;
    Dist gScore;
    Dist fScore;
    
    bool operator>(const AStarNode& other) const {
        return fScore > other.fScore;
    }
};

std::unordered_map<NodeID, Dist> astar(const Graph& g, NodeID source, NodeID goal,
                                        HeuristicFunc heuristic = nullptr) {
    std::unordered_map<NodeID, Dist> gScore;
    std::unordered_set<NodeID> visited;
    
    for (const auto& [u, edges] : g.getAdj()) {
        gScore[u] = INF;
        for (const auto& edge : edges) {
            gScore[edge.to] = INF;
        }
    }
    
    gScore[source] = 0;
    
    if (!heuristic) {
        heuristic = [](NodeID a, NodeID b) { return 0.0; };
    }
    
    std::priority_queue<AStarNode, std::vector<AStarNode>, 
                        std::greater<AStarNode>> pq;
    
    Dist h = heuristic(source, goal);
    pq.push({source, 0, h});
    
    while (!pq.empty()) {
        AStarNode current = pq.top();
        pq.pop();
        
        NodeID u = current.id;
        
        if (visited.count(u)) continue;
        visited.insert(u);
        
        if (u == goal) break;
        
        for (const auto& edge : g.outEdges(u)) {
            NodeID v = edge.to;
            Dist tentative_gScore = gScore[u] + edge.weight;
            
            if (tentative_gScore < gScore[v]) {
                gScore[v] = tentative_gScore;
                if (!visited.count(v)) {
                    Dist fScore = tentative_gScore + heuristic(v, goal);
                    pq.push({v, tentative_gScore, fScore});
                }
            }
        }
    }
    
    return gScore;
}

std::unordered_map<NodeID, Dist> astarMultiTarget(const Graph& g, NodeID source,
                                                   const std::unordered_set<NodeID>& goals,
                                                   HeuristicFunc heuristic = nullptr) {
    std::unordered_map<NodeID, Dist> gScore;
    std::unordered_set<NodeID> visited;
    std::unordered_set<NodeID> remainingGoals = goals;
    
    for (const auto& [u, edges] : g.getAdj()) {
        gScore[u] = INF;
        for (const auto& edge : edges) {
            gScore[edge.to] = INF;
        }
    }
    
    gScore[source] = 0;
    
    auto minDistToGoals = [&](NodeID node) -> Dist {
        if (!heuristic) return 0.0;
        
        Dist minH = INF;
        for (NodeID goal : remainingGoals) {
            Dist h = heuristic(node, goal);
            if (h < minH) minH = h;
        }
        return minH;
    };
    
    std::priority_queue<AStarNode, std::vector<AStarNode>, 
                        std::greater<AStarNode>> pq;
    
    Dist h = minDistToGoals(source);
    pq.push({source, 0, h});
    
    while (!pq.empty() && !remainingGoals.empty()) {
        AStarNode current = pq.top();
        pq.pop();
        
        NodeID u = current.id;
        
        if (visited.count(u)) continue;
        visited.insert(u);
        
        remainingGoals.erase(u);
        
        for (const auto& edge : g.outEdges(u)) {
            NodeID v = edge.to;
            Dist tentative_gScore = gScore[u] + edge.weight;
            
            if (tentative_gScore < gScore[v]) {
                gScore[v] = tentative_gScore;
                if (!visited.count(v)) {
                    Dist fScore = tentative_gScore + minDistToGoals(v);
                    pq.push({v, tentative_gScore, fScore});
                }
            }
        }
    }
    
    return gScore;
}

struct Coordinate {
    double x, y;
};

HeuristicFunc createEuclideanHeuristic(const std::unordered_map<NodeID, Coordinate>& coords) {
    return [coords](NodeID a, NodeID b) -> Dist {
        auto it_a = coords.find(a);
        auto it_b = coords.find(b);
        
        if (it_a == coords.end() || it_b == coords.end()) return 0.0;
        
        double dx = it_a->second.x - it_b->second.x;
        double dy = it_a->second.y - it_b->second.y;
        return std::sqrt(dx * dx + dy * dy);
    };
}

HeuristicFunc createManhattanHeuristic(const std::unordered_map<NodeID, Coordinate>& coords) {
    return [coords](NodeID a, NodeID b) -> Dist {
        auto it_a = coords.find(a);
        auto it_b = coords.find(b);
        
        if (it_a == coords.end() || it_b == coords.end()) return 0.0;
        
        double dx = std::abs(it_a->second.x - it_b->second.x);
        double dy = std::abs(it_a->second.y - it_b->second.y);
        return dx + dy;
    };
}

// =============================================================================
// BMSSP ALGORITHM
// =============================================================================

NodeID samplePivot(const NodeSet& S, const std::unordered_map<NodeID, Dist>& dhat) {
    if (S.size() <= 1) {
        return *S.begin();
    }
    
    // Sample up to 5 nodes randomly
    std::vector<NodeID> sample;
    sample.reserve(std::min<size_t>(5, S.size()));
    std::vector<NodeID> nodes = S.toVector();
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(nodes.begin(), nodes.end(), gen);
    
    for (size_t i = 0; i < std::min<size_t>(5, nodes.size()); ++i) {
        sample.push_back(nodes[i]);
    }
    
    // Select median distance
    std::sort(sample.begin(), sample.end(), [&](NodeID a, NodeID b) {
        return dhat.at(a) < dhat.at(b);
    });
    
    return sample[sample.size() / 2];
}

class BucketQueue {
private:
    std::vector<std::deque<NodeID>> buckets;
    Dist delta;
    size_t minIdx;
    std::unordered_map<NodeID, size_t> pos;
    
public:
    BucketQueue(Dist d) : delta(d), minIdx(0) {
        buckets.resize(100); // Initial size
    }
    
    void insert(NodeID v, Dist dist) {
        size_t idx = static_cast<size_t>(dist / delta);
        
        if (idx >= buckets.size()) {
            buckets.resize(idx + 1);
        }
        
        buckets[idx].push_back(v);
        pos[v] = idx;
    }
    
    bool extractMin(NodeID& v) {
        while (minIdx < buckets.size() && buckets[minIdx].empty()) {
            ++minIdx;
        }
        
        if (minIdx >= buckets.size()) {
            return false;
        }
        
        v = buckets[minIdx].front();
        buckets[minIdx].pop_front();
        pos.erase(v);
        
        return true;
    }
    
    void decreaseKey(NodeID v, Dist newDist) {
        if (pos.count(v)) {
            size_t oldIdx = pos[v];
            buckets[oldIdx].erase(std::remove(buckets[oldIdx].begin(), buckets[oldIdx].end(), v), buckets[oldIdx].end());
        }
        
        insert(v, newDist);
    }
    
    bool empty() const {
        return minIdx >= buckets.size() || buckets[minIdx].empty();
    }
};

void dijkstraDeltaStepping(const NodeSet& S, Dist B, const Graph& G,
                           std::unordered_map<NodeID, Dist>& dhat, Dist delta) {
    BucketQueue pq(delta);
    
    for (NodeID v : S) {
        if (dhat[v] < INF) {
            pq.insert(v, dhat[v]);
        }
    }
    
    std::unordered_set<NodeID> visited;
    
    NodeID u;
    while (pq.extractMin(u)) {
        if (visited.count(u)) continue;
        visited.insert(u);
        
        if (dhat[u] > B) continue;
        
        for (const auto& edge : G.outEdges(u)) {
            Dist newDist = dhat[u] + edge.weight;
            if (newDist < dhat[edge.to] && newDist <= B) {
                dhat[edge.to] = newDist;
                pq.decreaseKey(edge.to, newDist);
            }
        }
    }
}

void BMSSP(Dist B, const NodeSet& S, const Graph& G, 
           std::unordered_map<NodeID, Dist>& dhat) {
    if (S.size() == 0) return;
    
    if (S.size() <= 2 || B <= 1.0) {
        dijkstraDeltaStepping(S, B, G, dhat, 1.0);
        return;
    }
    
    NodeID pivot = samplePivot(S, dhat);
    Dist bound = std::min(B, dhat[pivot]);
    
    if (std::abs(bound - B) < 1e-9) {
        dijkstraDeltaStepping(S, B, G, dhat, 1.0);
        return;
    }
    
    // Dynamic delta based on average edge weight
    Dist delta = 1.0;
    size_t edgeCount = 0;
    Dist totalWeight = 0.0;
    for (const auto& [u, edges] : G.getAdj()) {
        for (const auto& edge : edges) {
            totalWeight += edge.weight;
            ++edgeCount;
        }
    }
    if (edgeCount > 0) {
        delta = totalWeight / edgeCount;
    }
    
    dijkstraDeltaStepping(S, bound, G, dhat, delta);
    
    NodeSet left, right;
    for (NodeID v : S) {
        if (dhat[v] <= bound && dhat[v] < INF) {
            left.add(v);
        } else if (dhat[v] < B && dhat[v] < INF) {
            right.add(v);
        }
    }
    
    if (left.size() > 0 && left.size() < S.size()) {
        BMSSP(bound, left, G, dhat);
    }
    if (right.size() > 0 && right.size() < S.size()) {
        BMSSP(B, right, G, dhat);
    }
}

std::unordered_map<NodeID, Dist> bmsspSingleSource(const Graph& G, NodeID source, Dist B) {
    std::unordered_map<NodeID, Dist> dhat;
    
    for (const auto& [u, edges] : G.getAdj()) {
        dhat[u] = INF;
        for (const auto& edge : edges) {
            dhat[edge.to] = INF;
        }
    }
    
    dhat[source] = 0;
    
    NodeSet S;
    S.add(source);
    
    BMSSP(B, S, G, dhat);
    
    return dhat;
}

#endif // BENCHMARK_HPP