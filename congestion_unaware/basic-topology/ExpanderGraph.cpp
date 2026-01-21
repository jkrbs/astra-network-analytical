/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "congestion_unaware/ExpanderGraph.h"
#include <cassert>
#include <ctime>
#include <cstdlib>
#include <algorithm>
#include <queue>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include "../../../helper/json/json.hpp"

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionUnaware;

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
    
    // create actual topology connection (for distance calculations)
    BasicTopology::connect(src, dest);
}

ExpanderGraph::ExpanderGraph(const int npus_count, const unsigned int degree, const Bandwidth bandwidth, const Latency latency, const std::string& inputfile) noexcept
    : BasicTopology(npus_count, bandwidth, latency) {
    assert(npus_count > 0);
    assert(bandwidth > 0);
    assert(latency >= 0);
    
    std::string inputfile_str = std::string(inputfile);
    // set the building block type
    basic_topology_type = TopologyBuildingBlock::ExpanderGraph;

    // initialize adjacency list vectors
    for (DeviceId i = 0; i < npus_count; ++i) {
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
        int graph_degree = j["degree"];
        
        // Check if we should use split graph (when npus_count is exactly half)
        bool use_split = (npus_count * 2 == node_count);
        
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
            auto adjacency = j["connected_graph_adjacency"];
            
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
            if (npus_count != node_count) {
                std::cerr << "[Error] NPU count (" << npus_count << ") does not match graph node count (" 
                          << node_count << ") and is not half for split mode" << std::endl;
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
            if (adjacency_list[i].size() != graph_degree) {
                std::cerr << "[Warning] Node " << i << " has degree " << adjacency_list[i].size() 
                          << " but expected " << graph_degree << std::endl;
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
