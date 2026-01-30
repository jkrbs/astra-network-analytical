/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "congestion_aware/ExpanderGraph.h"
#include <cassert>
#include <ctime>
#include <cstdlib>
#include <algorithm>
#include <queue>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <random>
#include <functional>
#include <set>
#include "../../../helper/json/json.hpp"

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionAware;


void ExpanderGraph::connect(DeviceId src, DeviceId dest) {
    assert(0 <= src && src < npus_count);
    assert(0 <= dest && dest < npus_count);
    if (src == dest) {
        std::cout << "[Error] Cannot connect a node to itself: " << src << std::endl;
        return;
        // throw std::invalid_argument("Cannot connect a node to itself");
    }
    if (std::find(adjacency_list[src].begin(), adjacency_list[src].end(), dest) != adjacency_list[src].end()) {
        std::cout << "[Error] Connection already exists between nodes " << src << " and " << dest << std::endl;
        return;
        // throw std::invalid_argument("Connection already exists between the specified nodes");
    }
    // create bidirectional connection between src and dest in adjacency list
    adjacency_list[src].push_back(dest);
    adjacency_list[dest].push_back(src);
    
    // create actual Device links (bidirectional)
    Topology::connect(src, dest, bandwidth, latency, true);
}

ExpanderGraph::RoutingAlgorithm ExpanderGraph::str2RoutingAlgorithm(const std::string& algo_str) {
    if (algo_str == "ShortestPath" || algo_str.empty()) {
        return RoutingAlgorithm::ShortestPath;
    } else if (algo_str == "RandomTopK") {
        return RoutingAlgorithm::RandomTopK;
    } else {
        std::cerr << "[Error] Unknown routing algorithm: " << algo_str << ". Defaulting to ShortestPath." << std::endl;
        return RoutingAlgorithm::ShortestPath;
    }
}


ExpanderGraph::ExpanderGraph(const int npus_count, const Bandwidth bandwidth, const Latency latency, const std::string& inputfile, const std::string& routing_algorithm, bool use_resiliency) noexcept
    : BasicTopology(npus_count, npus_count + (npus_count/8), bandwidth, latency) {
    assert(npus_count > 0);
    assert(bandwidth > 0);
    assert(latency >= 0);
    int degree = 0;

    this->routing_algorithm = str2RoutingAlgorithm(routing_algorithm);

    std::string inputfile_str = std::string(inputfile);
    // set the building block type
    basic_topology_type = TopologyBuildingBlock::ExpanderGraph;

    // initialize adjacency list vectors
    for (DeviceId i = 0; i < npus_count + (npus_count / 8); ++i) {
        adjacency_list[i] = std::vector<DeviceId>();
    }

    // load graph from input json file
    if (!inputfile_str.empty()) {
        std::ifstream file(inputfile_str);
        if (!file.is_open()) {
            std::cerr << "[Error] Failed to open expander graph JSON file: " << inputfile_str << std::endl;
            std::exit(-1);
        }

        nlohmann::json j;
        file >> j;
        file.close();

        int node_count = j["node_count"];
        degree = j["degree"];
        
        // Check if we should use split graph (when npus_count is exactly half)
        std::cout << "Use Resiliency: " << use_resiliency
                << ", node count in graph: " << node_count
                << ", npus_count: " << npus_count 
                << ", input file: " << inputfile_str << std::endl; 
        bool use_split = false;
        if (not use_resiliency) {
            use_split = (npus_count * 2 == node_count);
        } else {
            use_split = ((npus_count + (npus_count/8)) * 2 == node_count);
        }
        if (use_split) {
            std::cout << "[ExpanderGraph] Using split graph: " << npus_count << " NPUs from " 
                      << node_count << " node graph" << std::endl;
            
            // Determine which group to use (group A: first half)
            auto group_a = j["groups"]["A"].get<std::vector<int>>();
            
            // Build node mapping: original node ID -> local NPU ID (0 to npus_count-1)
            std::map<int, int> node_to_npu;
            std::set<int> group_a_set;
            for (size_t i = 0; i < group_a.size(); ++i) {
                node_to_npu[group_a[i]] = i;
                group_a_set.insert(group_a[i]);
            }
            
            // Use connected_graph_adjacency to build the graph
            // This is an adjacency list where index i contains neighbors of node i
            auto adjacency = j["split_graph_adjacency"];
            
            for (size_t node_id = 0; node_id < adjacency.size(); ++node_id) {
                // Only process nodes in group A
                if (group_a_set.find(node_id) == group_a_set.end()) {
                    continue;
                }
                
                int npu_id = node_to_npu[node_id];
                auto neighbors = adjacency[node_id].get<std::vector<int>>();
                
                for (int neighbor_node_id : neighbors) {
                    // Only add edges between nodes in group A
                    if (group_a_set.find(neighbor_node_id) != group_a_set.end()) {
                        int neighbor_npu_id = node_to_npu[neighbor_node_id];
                        // connect() adds bidirectional edges, so only connect if npu_id < neighbor_npu_id
                        // to avoid adding each edge twice
                        if (npu_id < neighbor_npu_id) {
                            this->connect(npu_id, neighbor_npu_id);
                        }
                    }
                }
            }
        } else {
            // Use full graph
            if (!use_resiliency && npus_count != node_count) {
                std::cerr << "[Error] Not using Resiliency Nodes. NPU count (" << npus_count << ") does not match graph node count (" 
                          << node_count << ") and is not half for split mode" << std::endl;
                std::exit(-1);
            }
            if (use_resiliency && node_count != npus_count + (npus_count)/8) {
                std::cerr << "[Error] Using Resiliency Nodes. NPU count (" << npus_count << ") does not match graph node count (" 
                          << npus_count + (npus_count)/8 << ") and is not half for split mode" << std::endl;
                std::exit(-1);
            }
            
            std::cout << "[ExpanderGraph] Using full graph: " << node_count << " nodes" << std::endl;
            
            // Use connected_graph_adjacency to build the full graph
            auto adjacency = j["connected_graph_adjacency"];
            
            for (size_t node_id = 0; node_id < adjacency.size(); ++node_id) {
                auto neighbors = adjacency[node_id].get<std::vector<int>>();
                
                for (int neighbor_id : neighbors) {
                    // connect() adds bidirectional edges, so only connect if node_id < neighbor_id
                    // to avoid adding each edge twice
                    if ((int)node_id < neighbor_id) {
                        this->connect(node_id, neighbor_id);
                    }
                }
            }
        }
        
        // Verify the graph degree
        for (int i = 0; i < npus_count; ++i) {
            if (adjacency_list[i].size() != degree) {
                std::cerr << "[Warning] Node " << i << " has degree " << adjacency_list[i].size() 
                          << " but expected " << degree << std::endl;
            }
        }
    } else {
        std::cerr << "[Error] ExpanderGraph requires an input JSON file" << std::endl;
        std::exit(-1);
    }
}

unsigned int ExpanderGraph::get_distance(const DeviceId src, const DeviceId dest, std::set<DeviceId> visited, unsigned int current_distance) const noexcept {
    // Use distance cache
    std::pair<DeviceId, DeviceId> node_pair = std::make_pair(src, dest);
    if (distance_cache.find(node_pair) != distance_cache.end()) {
        return distance_cache[node_pair];
    }
    
    if (src == dest) {
        return 0;
    }
    
    // Dijkstra's algorithm using priority queue
    std::vector<unsigned int> dist(npus_count, UINT32_MAX);
    std::priority_queue<std::pair<unsigned int, DeviceId>, 
                       std::vector<std::pair<unsigned int, DeviceId>>,
                       std::greater<std::pair<unsigned int, DeviceId>>> pq;
    
    dist[src] = 0;
    pq.push({0, src});
    
    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        
        if (u == dest) {
            distance_cache[node_pair] = d;
            return d;
        }
        
        if (d > dist[u]) {
            continue;
        }
        
        for (const auto& neighbor : adjacency_list.at(u)) {
            unsigned int new_dist = dist[u] + 1;
            if (new_dist < dist[neighbor]) {
                dist[neighbor] = new_dist;
                pq.push({new_dist, neighbor});
            }
        }
    }
    
    assert(dist[dest] != UINT32_MAX);
    assert(dist[dest] > 0);
    distance_cache[node_pair] = dist[dest];
    return dist[dest];
}   

int ExpanderGraph::compute_hops_count(const DeviceId src, const DeviceId dest) const noexcept {
    assert(0 <= src && src < npus_count);
    assert(0 <= dest && dest < npus_count);
    assert(src != dest);

    return this->get_distance(src, dest, std::set<DeviceId>(), 0);
}

Route ExpanderGraph::route(DeviceId src, DeviceId dest) const noexcept {
    switch(routing_algorithm) {
        case RoutingAlgorithm::ShortestPath:
            return route_shortest_path(src, dest);
        case RoutingAlgorithm::RandomTopK:
            return route_random_topk(src, dest);
        default:
            // default to shortest path
            return route_shortest_path(src, dest);
    }
}

Route ExpanderGraph::route_shortest_path(DeviceId src, DeviceId dest) const noexcept {
    // assert npus are in valid range
    assert(0 <= src && src < npus_count);
    assert(0 <= dest && dest < npus_count);

    
    std::pair<DeviceId, DeviceId> node_pair = std::make_pair(src, dest);
    // Check route cache
    if (shortest_route_cache.find(node_pair) != shortest_route_cache.end()) {
        Route cached_route;
        for (const auto& device_id : shortest_route_cache.at(node_pair)) {
            if (device_id >= static_cast<DeviceId>(devices.size())) {
                std::cerr << "[ERROR] device_id " << device_id << " >= devices.size() " << devices.size() << std::endl;
                std::exit(-1);
            }
            cached_route.push_back(devices[device_id]);
        }
        return cached_route;
    }

    // construct empty route
    auto route = Route();

    // route via shortest path (BFS)
    std::map<DeviceId, DeviceId> parent; // to reconstruct path
    std::set<DeviceId> visited;
    std::vector<DeviceId> queue;
    queue.push_back(src);
    visited.insert(src);
    parent[src] = src;
    bool found = false;
    while (!queue.empty() && !found) {
        DeviceId current = queue.front();
        queue.erase(queue.begin());

        for (const auto& neighbor : adjacency_list.at(current)) {
            if (visited.find(neighbor) == visited.end()) {
                visited.insert(neighbor);
                parent[neighbor] = current;
                queue.push_back(neighbor);

                if (neighbor == dest) {
                    found = true;
                    break;
                }
            }
        }
    }
    // reconstruct path
    if (found) {
        std::vector<DeviceId> path;
        DeviceId current = dest;
        while (current != src) {
            path.push_back(current);
            current = parent[current];
        }
        path.push_back(src);
        std::reverse(path.begin(), path.end());
        
        // convert device IDs to device pointers
        for (const auto& device_id : path) {
            if (device_id >= static_cast<DeviceId>(devices.size())) {
                std::cerr << "[ERROR] device_id " << device_id << " >= devices.size() " << devices.size() << std::endl;
                std::exit(-1);
            }
            route.push_back(devices[device_id]);
        }
    }

    // Cache the computed route
    std::vector<DeviceId> device_id_path;
    for (const auto& device_ptr : route) {
        device_id_path.push_back(device_ptr->get_id());
    }
    shortest_route_cache[node_pair] = device_id_path;

    return route;
}


Route ExpanderGraph::route_random_topk(DeviceId src, DeviceId dest) const noexcept {
    // assert npus are in valid range
    assert(0 <= src && src < npus_count);
    assert(0 <= dest && dest < npus_count);

    
    std::pair<DeviceId, DeviceId> node_pair = std::make_pair(src, dest);
    // Check route cache
    if (topk_route_cache.find(node_pair) != topk_route_cache.end()) {
        static thread_local std::mt19937 rng(std::random_device{}());
        const auto& cached_paths = topk_route_cache.at(node_pair);
        size_t start_index = cached_paths.size() > 4 ? 4 : 0;
        std::uniform_int_distribution<size_t> dist(start_index, cached_paths.size() - 1);
        const auto& chosen_path = cached_paths[dist(rng)];
        Route cached_route;
        for (const auto& device_id : chosen_path) {
            if (device_id >= static_cast<DeviceId>(devices.size())) {
                std::cerr << "[ERROR] device_id " << device_id << " >= devices.size() " << devices.size() << std::endl;
                std::exit(-1);
            }
            cached_route.push_back(devices[device_id]);
        }
        return cached_route;
    }

    // construct empty route
    auto route = Route();

    using Edge = std::pair<DeviceId, DeviceId>;
    auto normalize_edge = [](DeviceId a, DeviceId b) {
        return a < b ? std::make_pair(a, b) : std::make_pair(b, a);
    };

    auto bfs_path = [&](DeviceId start,
                        DeviceId goal,
                        const std::set<DeviceId>& banned_nodes,
                        const std::set<Edge>& banned_edges) {
        std::vector<int> dist(npus_count, -1);
        std::vector<DeviceId> parent(npus_count, -1);
        std::queue<DeviceId> q;

        if (banned_nodes.find(start) != banned_nodes.end()) {
            return std::vector<DeviceId>();
        }

        dist[start] = 0;
        parent[start] = start;
        q.push(start);

        while (!q.empty()) {
            DeviceId current = q.front();
            q.pop();

            if (current == goal) {
                break;
            }

            for (const auto& neighbor : adjacency_list.at(current)) {
                if (banned_nodes.find(neighbor) != banned_nodes.end()) {
                    continue;
                }
                if (banned_edges.find(normalize_edge(current, neighbor)) != banned_edges.end()) {
                    continue;
                }
                if (dist[neighbor] == -1) {
                    dist[neighbor] = dist[current] + 1;
                    parent[neighbor] = current;
                    q.push(neighbor);
                }
            }
        }

        if (dist[goal] == -1) {
            return std::vector<DeviceId>();
        }

        std::vector<DeviceId> path;
        DeviceId current = goal;
        while (current != start) {
            path.push_back(current);
            current = parent[current];
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());
        return path;
    };

    auto k_shortest_paths = [&](DeviceId start, DeviceId goal, size_t k_max) {
        std::vector<std::vector<DeviceId>> shortest_paths;
        std::vector<std::vector<DeviceId>> candidates;

        auto first = bfs_path(start, goal, {}, {});
        if (first.empty()) {
            return shortest_paths;
        }
        shortest_paths.push_back(first);

        for (size_t k = 1; k < k_max; ++k) {
            const auto& prev_path = shortest_paths[k - 1];
            for (size_t i = 0; i + 1 < prev_path.size(); ++i) {
                DeviceId spur_node = prev_path[i];
                std::vector<DeviceId> root_path(prev_path.begin(), prev_path.begin() + i + 1);

                std::set<DeviceId> banned_nodes(root_path.begin(), root_path.end() - 1);
                std::set<Edge> banned_edges;

                for (const auto& p : shortest_paths) {
                    if (p.size() > i && std::equal(root_path.begin(), root_path.end(), p.begin())) {
                        banned_edges.insert(normalize_edge(p[i], p[i + 1]));
                    }
                }

                auto spur_path = bfs_path(spur_node, goal, banned_nodes, banned_edges);
                if (spur_path.empty()) {
                    continue;
                }

                std::vector<DeviceId> total_path = root_path;
                total_path.insert(total_path.end(), spur_path.begin() + 1, spur_path.end());

                bool duplicate = false;
                for (const auto& p : shortest_paths) {
                    if (p == total_path) {
                        duplicate = true;
                        break;
                    }
                }
                if (!duplicate) {
                    for (const auto& p : candidates) {
                        if (p == total_path) {
                            duplicate = true;
                            break;
                        }
                    }
                }
                if (!duplicate) {
                    candidates.push_back(std::move(total_path));
                }
            }

            if (candidates.empty()) {
                break;
            }

            auto best_it = std::min_element(
                candidates.begin(),
                candidates.end(),
                [](const std::vector<DeviceId>& a, const std::vector<DeviceId>& b) {
                    return a.size() < b.size();
                });

            shortest_paths.push_back(*best_it);
            candidates.erase(best_it);
        }

        return shortest_paths;
    };

    constexpr size_t k_max_paths = 16;
    auto paths = k_shortest_paths(src, dest, k_max_paths);
    if (paths.empty()) {
        std::cerr << "[ERROR] No route found from " << src << " to " << dest << std::endl;
        std::exit(-1);
    }

    // Cache k-shortest paths
    topk_route_cache[node_pair] = paths;

    // Pick a random path beyond the 4 shortest if possible
    static thread_local std::mt19937 rng(std::random_device{}());
    size_t start_index = (paths.size() > 4) ? 4 : 0;
    size_t end_index = paths.size() - 1;
    
    // Ensure start_index doesn't exceed available paths
    if (start_index > end_index) {
        start_index = 0;
    }
    
    std::uniform_int_distribution<size_t> pick(start_index, end_index);
    const auto& chosen_path = paths[pick(rng)];

    // convert device IDs to device pointers
    for (const auto& device_id : chosen_path) {
        if (device_id >= static_cast<DeviceId>(devices.size())) {
            std::cerr << "[ERROR] device_id " << device_id << " >= devices.size() " << devices.size() << std::endl;
            std::exit(-1);
        }
        route.push_back(devices[device_id]);
    }

    return route;
}