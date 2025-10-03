#ifndef BENCHMARK_HPP
#define BENCHMARK_HPP

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <limits>
#include <algorithm>
#include <cmath>

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
    
    // Initialize all distances to infinity
    for (const auto& [u, edges] : g.getAdj()) {
        dist[u] = INF;
        // Also initialize destination nodes
        for (const auto& edge : edges) {
            if (dist.find(edge.to) == dist.end()) {
                dist[edge.to] = INF;
            }
        }
    }
    
    dist[source] = 0;
    
    // Priority queue (min-heap)
    std::priority_queue<DijkstraNode, std::vector<DijkstraNode>, 
                        std::greater<DijkstraNode>> pq;
    pq.push({source, 0});
    
    while (!pq.empty()) {
        DijkstraNode current = pq.top();
        pq.pop();
        
        NodeID u = current.id;
        
        if (visited.count(u)) {
            continue;
        }
        
        visited.insert(u);
        
        // Relax all outgoing edges
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
// BMSSP ALGORITHM
// =============================================================================

// Median-of-three pivot selection
NodeID medianOfThreePivot(const NodeSet& S, const std::unordered_map<NodeID, Dist>& dhat) {
    std::vector<NodeID> nodes = S.toVector();
    
    if (nodes.size() <= 3) {
        return nodes[nodes.size() / 2];
    }
    
    // Sort by distance
    std::sort(nodes.begin(), nodes.end(), [&](NodeID a, NodeID b) {
        return dhat.at(a) < dhat.at(b);
    });
    
    NodeID first = nodes[0];
    NodeID middle = nodes[nodes.size() / 2];
    NodeID last = nodes[nodes.size() - 1];
    
    // Find median of three
    std::vector<NodeID> candidates = {first, middle, last};
    std::sort(candidates.begin(), candidates.end(), [&](NodeID a, NodeID b) {
        return dhat.at(a) < dhat.at(b);
    });
    
    return candidates[1];
}

class BucketQueue {
private:
    std::vector<std::vector<NodeID>> buckets;
    Dist delta;
    size_t minIdx;
    std::unordered_map<NodeID, int> pos;
    
public:
    BucketQueue(Dist d) : delta(d), minIdx(0) {}
    
    void insert(NodeID v, Dist dist) {
        int idx = static_cast<int>(dist / delta);
        
        // Expand buckets if necessary
        while (idx >= static_cast<int>(buckets.size())) {
            buckets.emplace_back();
        }
        
        buckets[idx].push_back(v);
        pos[v] = idx;
    }
    
    bool extractMin(NodeID& v) {
        // Find next non-empty bucket
        while (minIdx < buckets.size() && buckets[minIdx].empty()) {
            minIdx++;
        }
        
        if (minIdx >= buckets.size()) {
            return false;
        }
        
        // Extract node
        v = buckets[minIdx].back();
        buckets[minIdx].pop_back();
        pos.erase(v);
        
        return true;
    }
    
    void decreaseKey(NodeID v, Dist newDist) {
        auto it = pos.find(v);
        if (it != pos.end()) {
            int oldIdx = it->second;
            auto& bucket = buckets[oldIdx];
            bucket.erase(std::remove(bucket.begin(), bucket.end(), v), bucket.end());
        }
        
        insert(v, newDist);
    }
};

// Delta-stepping Dijkstra for bounded shortest paths
void dijkstraDeltaStepping(const NodeSet& S, Dist B, const Graph& G,
                          std::unordered_map<NodeID, Dist>& dhat, Dist delta) {
    BucketQueue pq(delta);
    
    for (NodeID v : S) {
        pq.insert(v, dhat[v]);
    }
    
    std::unordered_set<NodeID> visited;
    
    NodeID u;
    while (pq.extractMin(u)) {
        if (visited.count(u)) {
            continue;
        }
        
        visited.insert(u);
        
        // Stop if beyond bound
        if (dhat[u] > B) {
            continue;
        }
        
        for (const auto& edge : G.outEdges(u)) {
            Dist newDist = dhat[u] + edge.weight;
            if (newDist < dhat[edge.to]) {
                dhat[edge.to] = newDist;
                pq.decreaseKey(edge.to, newDist);
            }
        }
    }
}

void BMSSP(Dist B, const NodeSet& S, const Graph& G, 
           std::unordered_map<NodeID, Dist>& dhat) {
    if (S.size() == 0) {
        return;
    }
    
    // Base case
    if (S.size() == 1 || B <= 1.0) {
        dijkstraDeltaStepping(S, B, G, dhat, 1.0);
        return;
    }
    
    // Select pivot
    NodeID pivot = medianOfThreePivot(S, dhat);
    double bound = std::min(static_cast<double>(B), static_cast<double>(dhat[pivot]));
    
    if (std::abs(bound - static_cast<double>(B)) < 1e-9) {
        dijkstraDeltaStepping(S, B, G, dhat, 1.0);
        return;
    }
    
    // Run bounded Dijkstra
    dijkstraDeltaStepping(S, Dist(bound), G, dhat, 1.0);
    
    // Partition nodes
    NodeSet left, right;
    
    for (const auto& [v, edges] : G.getAdj()) {
        if (dhat[v] < INF) {
            if (dhat[v] <= Dist(bound)) {
                left.add(v);
            } else if (dhat[v] < B) {
                right.add(v);
            }
        }
        
        // Check destination nodes
        for (const auto& edge : edges) {
            NodeID dst = edge.to;
            if (dhat[dst] < INF) {
                if (dhat[dst] <= Dist(bound)) {
                    left.add(dst);
                } else if (dhat[dst] < B) {
                    right.add(dst);
                }
            }
        }
    }
    
    if (left.size() > 0 && left.size() < S.size()) {
        BMSSP(Dist(bound), left, G, dhat);
    }
    if (right.size() > 0 && right.size() < S.size()) {
        BMSSP(B, right, G, dhat);
    }
}

// Single-source BMSSP
std::unordered_map<NodeID, Dist> bmsspSingleSource(const Graph& G, NodeID source, Dist B) {
    std::unordered_map<NodeID, Dist> dhat;
    
    for (const auto& [u, edges] : G.getAdj()) {
        dhat[u] = INF;
        for (const auto& edge : edges) {
            if (dhat.find(edge.to) == dhat.end()) {
                dhat[edge.to] = INF;
            }
        }
    }
    
    dhat[source] = 0;
    
    NodeSet S;
    S.add(source);
    
    BMSSP(B, S, G, dhat);
    
    return dhat;
}

#endif // BENCHMARK_HPP