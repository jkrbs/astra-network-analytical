/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "congestion_aware/EpExpanderTopology.h"
#include <cassert>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include "../../../helper/json/json.hpp"

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionAware;

EpExpanderTopology::EpExpanderTopology(const std::string& routes_file, 
                                       const Bandwidth bandwidth, 
                                       const Latency latency) noexcept
    : BasicTopology(1, 1, bandwidth, latency),  // Will be updated after loading
      routes_file_path(routes_file),
      node_count(0),
      degree(0),
      rng(std::random_device{}()) {
    
    assert(!routes_file.empty());
    assert(bandwidth > 0);
    assert(latency >= 0);

    // Load routes from JSON
    load_routes(routes_file);

    // Update BasicTopology fields after loading
    this->npus_count = node_count;
    this->devices_count = node_count;
    this->dims_count = 1;
    this->npus_count_per_dim.clear();
    this->npus_count_per_dim.push_back(node_count);
    this->bandwidth_per_dim.clear();
    this->bandwidth_per_dim.push_back(bandwidth);

    // Clear and re-instantiate devices with correct count
    this->devices.clear();
    instantiate_devices();

    // Build the actual links
    build_links_from_routes();

    std::cout << "[EpExpanderTopology] Loaded " << node_count << " nodes, degree " 
              << degree << " from " << routes_file << std::endl;
}

void EpExpanderTopology::load_routes(const std::string& routes_file) {
    std::ifstream file(routes_file);
    if (!file.is_open()) {
        std::cerr << "[Error] Failed to open EP routes JSON file: " << routes_file << std::endl;
        std::exit(-1);
    }

    nlohmann::json j;
    try {
        file >> j;
    } catch (const nlohmann::json::parse_error& e) {
        std::cerr << "[Error] Failed to parse EP routes JSON: " << e.what() << std::endl;
        std::exit(-1);
    }
    file.close();

    // Read metadata
    if (j.contains("metadata")) {
        node_count = j["metadata"]["node_count"].get<int>();
        degree = j["metadata"]["degree"].get<int>();
        // Read ep_nodes if present (for switch topology), otherwise use node_count
        if (j["metadata"].contains("ep_nodes")) {
            ep_node_count = j["metadata"]["ep_nodes"].get<int>();
        } else {
            ep_node_count = node_count;
        }
    } else {
        std::cerr << "[Error] EP routes JSON missing metadata" << std::endl;
        std::exit(-1);
    }

    // Initialize adjacency list
    for (int i = 0; i < node_count; ++i) {
        adjacency_list[i] = std::vector<DeviceId>();
    }

    // Read routes
    if (!j.contains("routes")) {
        std::cerr << "[Error] EP routes JSON missing routes" << std::endl;
        std::exit(-1);
    }

    for (auto& [src_str, dst_map] : j["routes"].items()) {
        DeviceId src = std::stoi(src_str);
        routes[src] = std::map<DeviceId, std::vector<RouteInfo>>();

        for (auto& [dst_str, route_list] : dst_map.items()) {
            DeviceId dst = std::stoi(dst_str);
            routes[src][dst] = std::vector<RouteInfo>();

            for (auto& route_json : route_list) {
                RouteInfo info;
                info.path = route_json["path"].get<std::vector<DeviceId>>();
                info.hops = route_json["hops"].get<int>();
                info.weight = route_json["weight"].get<double>();
                routes[src][dst].push_back(info);

                // Build adjacency list from route paths
                for (size_t i = 0; i + 1 < info.path.size(); ++i) {
                    DeviceId a = info.path[i];
                    DeviceId b = info.path[i + 1];
                    
                    // Add to adjacency list if not already present
                    if (std::find(adjacency_list[a].begin(), adjacency_list[a].end(), b) 
                        == adjacency_list[a].end()) {
                        adjacency_list[a].push_back(b);
                    }
                    if (std::find(adjacency_list[b].begin(), adjacency_list[b].end(), a) 
                        == adjacency_list[b].end()) {
                        adjacency_list[b].push_back(a);
                    }
                }
            }
        }
    }

    std::cout << "[EpExpanderTopology] Loaded routes for " << routes.size() << " source nodes" << std::endl;
}

void EpExpanderTopology::build_links_from_routes() {
    // Create bidirectional links for all adjacencies
    for (const auto& [src, neighbors] : adjacency_list) {
        for (DeviceId dst : neighbors) {
            // Only connect if src < dst to avoid double-connecting
            if (src < dst) {
                Topology::connect(src, dst, bandwidth, latency, true);
            }
        }
    }
}

const RouteInfo& EpExpanderTopology::select_route(DeviceId src, DeviceId dest) const noexcept {
    assert(routes.count(src) > 0);
    assert(routes.at(src).count(dest) > 0);
    
    const auto& route_options = routes.at(src).at(dest);
    assert(!route_options.empty());

    if (route_options.size() == 1) {
        return route_options[0];
    }

    // Select route probabilistically based on weights
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    double r = dist(rng);
    double cumulative = 0.0;

    for (const auto& route_info : route_options) {
        cumulative += route_info.weight;
        if (r < cumulative) {
            return route_info;
        }
    }

    // Fallback to last route (shouldn't happen with proper weights)
    return route_options.back();
}

Route EpExpanderTopology::route(DeviceId src, DeviceId dest) const noexcept {
    assert(0 <= src && src < npus_count);
    assert(0 <= dest && dest < npus_count);

    if (src == dest) {
        // Self-send - just return source device
        Route r;
        r.push_back(devices[src]);
        return r;
    }

    // Select a route based on weights
    const RouteInfo& selected = select_route(src, dest);

    // Convert device IDs to device pointers
    Route r;
    for (DeviceId device_id : selected.path) {
        assert(device_id < static_cast<DeviceId>(devices.size()));
        r.push_back(devices[device_id]);
    }

    return r;
}

const std::vector<int>& EpExpanderTopology::get_permutation(int layer_id) const noexcept {
    // Compute effective layer ID based on num_permutation_layers setting
    // If num_permutation_layers > 0, permutations repeat every N layers
    // If num_permutation_layers == 0, each layer gets a unique permutation
    int effective_layer_id = (num_permutation_layers > 0) 
        ? (layer_id % num_permutation_layers) 
        : layer_id;
    
    // Check if permutation is already cached
    if (layer_permutations.find(effective_layer_id) != layer_permutations.end()) {
        return layer_permutations.at(effective_layer_id);
    }

    // Generate new permutation for this layer
    // Use ep_node_count (not node_count) to exclude switch nodes from permutation
    std::vector<int> perm(ep_node_count);
    std::iota(perm.begin(), perm.end(), 0);

    // Use effective_layer_id as seed for deterministic permutation
    std::mt19937 layer_rng(effective_layer_id);
    std::shuffle(perm.begin(), perm.end(), layer_rng);

    // Cache and return
    layer_permutations[effective_layer_id] = perm;
    return layer_permutations.at(effective_layer_id);
}

Route EpExpanderTopology::route_with_permutation(DeviceId src, DeviceId dest, int layer_id) const noexcept {
    assert(0 <= src && src < npus_count);
    assert(0 <= dest && dest < npus_count);

    if (src == dest) {
        Route r;
        r.push_back(devices[src]);
        return r;
    }

    // Apply permutation to map local ranks to expander nodes
    const auto& perm = get_permutation(layer_id);
    DeviceId permuted_src = perm[src];
    DeviceId permuted_dst = perm[dest];

    // Route through the permuted nodes
    return route(permuted_src, permuted_dst);
}

std::vector<Route> EpExpanderTopology::get_all_routes_with_permutation(DeviceId src, DeviceId dest, int layer_id) const noexcept {
    assert(0 <= src && src < npus_count);
    assert(0 <= dest && dest < npus_count);

    std::vector<Route> all_routes;

    if (src == dest) {
        // Self-send - return single route with just the source device
        Route r;
        r.push_back(devices[src]);
        all_routes.push_back(r);
        return all_routes;
    }

    // Apply permutation to map local ranks to expander nodes
    const auto& perm = get_permutation(layer_id);
    DeviceId permuted_src = perm[src];
    DeviceId permuted_dst = perm[dest];

    // Get all route options for this src-dst pair
    assert(routes.count(permuted_src) > 0);
    assert(routes.at(permuted_src).count(permuted_dst) > 0);
    
    const auto& route_options = routes.at(permuted_src).at(permuted_dst);
    assert(!route_options.empty());

    // Convert each RouteInfo to a Route (device pointers)
    for (const auto& route_info : route_options) {
        Route r;
        for (DeviceId device_id : route_info.path) {
            assert(device_id < static_cast<DeviceId>(devices.size()));
            r.push_back(devices[device_id]);
        }
        all_routes.push_back(r);
    }

    return all_routes;
}

std::unique_ptr<BasicTopology> EpExpanderTopology::clone() const noexcept {
    return std::make_unique<EpExpanderTopology>(routes_file_path, bandwidth, latency);
}
