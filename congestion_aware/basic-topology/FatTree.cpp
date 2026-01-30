#include "congestion_aware/FatTree.h"
#include <cassert>
#include <ctime>
#include <cstdlib>
#include <algorithm>
#include <queue>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <random>
#include "../../../helper/json/json.hpp"

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionAware;

FatTree::RoutingAlgorithm FatTree::str2RoutingAlgorithm(const std::string& algo_str) noexcept {
    if (algo_str == "Deterministic" || algo_str.empty()) {
        return RoutingAlgorithm::Deterministic;
    } else if (algo_str == "Random") {
        return RoutingAlgorithm::Random;
    } else {
        std::cerr << "[Error] Unknown FatTree routing algorithm: " << algo_str << ". Defaulting to Deterministic." << std::endl;
        return RoutingAlgorithm::Deterministic;
    }
}

FatTree::FatTree(int npus_count, int k, Bandwidth bandwidth, Latency latency, 
                 const std::string& routing_algorithm_str) noexcept
    : BasicTopology(npus_count, npus_count + ((k * k) / 2) + ((k * k) / 4) + ((k / 2) * (k / 2)), 
                    bandwidth, latency), k(k) {
    assert(npus_count > 0);
    assert(k > 0 && k % 2 == 0);  // k must be even and positive
    assert(bandwidth > 0);
    assert(latency >= 0);

    basic_topology_type = TopologyBuildingBlock::FatTree;
    this->routing_algorithm = str2RoutingAlgorithm(routing_algorithm_str);

    const int pods = k;
    const int num_leaf_switches = (k * k) / 2;
    const int num_spine_switches = (k * k) / 4;
    const int num_core_switches = (k / 2) * (k / 2);
    const int npus_per_leaf_ideal = k / 2;

    // Create leaf switches
    for (int i = 0; i < num_leaf_switches; ++i) {
        leaf_switches.emplace_back(Switch(k, bandwidth, latency));
    }

    // Create spine switches
    for (int i = 0; i < num_spine_switches; ++i) {
        spine_switches.emplace_back(Switch(k, bandwidth, latency));
    }

    // Create core switches
    for (int i = 0; i < num_core_switches; ++i) {
        core_switches.emplace_back(Switch(k, bandwidth, latency));
    }

    // Distribute NPUs across leaves
    npus_per_leaf.resize(num_leaf_switches, 0);
    npu_to_leaf.resize(npus_count);
    
    int npu_id = 0;
    for (int leaf = 0; leaf < num_leaf_switches && npu_id < npus_count; ++leaf) {
        int npus_for_this_leaf = std::min(npus_per_leaf_ideal, npus_count - npu_id);
        npus_per_leaf[leaf] = npus_for_this_leaf;
        
        for (int i = 0; i < npus_for_this_leaf && npu_id < npus_count; ++i) {
            npu_to_leaf[npu_id] = leaf;
            npu_id++;
        }
    }

    // Instantiate devices after setting devices_count via parent class
    instantiate_devices();

    // Device ID layout:
    // [0, npus_count) = NPUs
    // [npus_count, npus_count + num_leaf_switches) = leaf switches
    // [npus_count + num_leaf_switches, npus_count + num_leaf_switches + num_spine_switches) = spine switches
    // [npus_count + num_leaf_switches + num_spine_switches, ...) = core switches
    const int leaf_switch_offset = npus_count;
    const int spine_switch_offset = npus_count + num_leaf_switches;
    const int core_switch_offset = npus_count + num_leaf_switches + num_spine_switches;

    // Connect leaf switches to NPUs
    npu_id = 0;
    for (int leaf = 0; leaf < num_leaf_switches; ++leaf) {
        for (int i = 0; i < npus_per_leaf[leaf]; ++i) {
            int leaf_switch_id = leaf_switch_offset + leaf;
            Topology::connect(npu_id, leaf_switch_id, bandwidth, latency);
            npu_id++;
        }
    }

    // Connect leaf switches to spine switches
    // Each leaf switch in a pod connects to all spine switches in that pod
    for (int pod = 0; pod < pods; ++pod) {
        for (int i = 0; i < k / 2; ++i) {  // leaf switches in pod
            for (int j = 0; j < k / 2; ++j) {  // spine switches in pod
                int leaf_index = pod * (k / 2) + i;
                int spine_index = pod * (k / 2) + j;
                int leaf_device_id = leaf_switch_offset + leaf_index;
                int spine_device_id = spine_switch_offset + spine_index;
                Topology::connect(leaf_device_id, spine_device_id, bandwidth, latency);
            }
        }
    }  
    
    // Connect spine switches to core switches
    // Each spine switch i connects to all core switches in column i
    for (int i = 0; i < k / 2; ++i) {
        for (int j = 0; j < k / 2; ++j) {
            for (int pod = 0; pod < pods; ++pod) {
                int spine_index = pod * (k / 2) + i;
                int core_index = i * (k / 2) + j;
                int spine_device_id = spine_switch_offset + spine_index;
                int core_device_id = core_switch_offset + core_index;
                Topology::connect(spine_device_id, core_device_id, bandwidth, latency);
            }
        }
    }
}   

Route FatTree::route(DeviceId src, DeviceId dest) const noexcept {
    // assert npus are in valid range
    assert(0 <= src && src < npus_count);
    assert(0 <= dest && dest < npus_count);

    // construct empty route
    auto route = Route();

    const int num_leaf_switches = (k * k) / 2;
    const int num_spine_switches = (k * k) / 4;
    const int leaf_switch_offset = npus_count;
    const int spine_switch_offset = npus_count + num_leaf_switches;
    const int core_switch_offset = npus_count + num_leaf_switches + num_spine_switches;

    int src_leaf = npu_to_leaf[src];
    int dest_leaf = npu_to_leaf[dest];

    // If src and dest are under the same leaf switch, route directly
    if (src_leaf == dest_leaf) {
        route.push_back(devices[src]);
        route.push_back(devices[leaf_switch_offset + src_leaf]);
        route.push_back(devices[dest]);
        return route;
    }

    // Determine which pod each leaf belongs to
    int src_pod = src_leaf / (k / 2);
    int dest_pod = dest_leaf / (k / 2);
    
    // Get spine switch indices within each pod
    int src_leaf_in_pod = src_leaf % (k / 2);
    int dest_leaf_in_pod = dest_leaf % (k / 2);
    
    // If src and dest are in the same pod, route through a spine switch within the pod
    if (src_pod == dest_pod) {
        // In deterministic mode: pick first spine switch, in random mode: pick random spine
        int spine_in_pod = src_leaf_in_pod;
        
        if (routing_algorithm == RoutingAlgorithm::Random) {
            static thread_local std::mt19937 gen(std::random_device{}());
            std::uniform_int_distribution<> dis(0, k / 2 - 1);
            spine_in_pod = dis(gen);
        }
        
        int spine_index = src_pod * (k / 2) + spine_in_pod;
        
        // Route: src -> src_leaf -> spine -> dest_leaf -> dest (5 hops)
        route.push_back(devices[src]);
        route.push_back(devices[leaf_switch_offset + src_leaf]);
        route.push_back(devices[spine_switch_offset + spine_index]);
        route.push_back(devices[leaf_switch_offset + dest_leaf]);
        route.push_back(devices[dest]);
        return route;
    }
    
    // Otherwise, route up through spine and core switches
    // Determine spine switches based on routing algorithm
    int src_spine_in_pod = src_leaf_in_pod;
    int dest_spine_in_pod = dest_leaf_in_pod;
    
    if (routing_algorithm == RoutingAlgorithm::Random) {
        static thread_local std::mt19937 gen(std::random_device{}());
        std::uniform_int_distribution<> dis(0, k / 2 - 1);
        src_spine_in_pod = dis(gen);
        dest_spine_in_pod = dis(gen);
    }
    
    int src_spine_index = src_pod * (k / 2) + src_spine_in_pod;
    int dest_spine_index = dest_pod * (k / 2) + dest_spine_in_pod;
    
    // In random mode, pick a random core switch that connects src_spine and dest_spine
    // In deterministic mode, use the core switch based on spine indices
    int core_row = src_spine_in_pod;
    int core_col = dest_spine_in_pod;
    
    if (routing_algorithm == RoutingAlgorithm::Random) {
        static thread_local std::mt19937 gen(std::random_device{}());
        std::uniform_int_distribution<> dis(0, k / 2 - 1);
        core_row = dis(gen);
        core_col = dis(gen);
    }
    
    int core_index = core_row * (k / 2) + core_col;

    // Build the route: src -> src_leaf -> src_spine -> core -> dest_spine -> dest_leaf -> dest (7 hops)
    route.push_back(devices[src]);
    route.push_back(devices[leaf_switch_offset + src_leaf]);
    route.push_back(devices[spine_switch_offset + src_spine_index]);
    route.push_back(devices[core_switch_offset + core_index]);
    route.push_back(devices[spine_switch_offset + dest_spine_index]);
    route.push_back(devices[leaf_switch_offset + dest_leaf]);
    route.push_back(devices[dest]);

    return route;
}