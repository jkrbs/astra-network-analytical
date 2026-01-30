#include "congestion_aware/SwitchOrExpander.h"
#include "congestion_aware/BasicTopology.h"
#include <cassert>
#include <iostream>

// avoid depending on top-level include paths in this external tree; declare
// the global flag here instead of including the simulator header.
std::shared_ptr<std::map<DeviceId, bool>> use_moe_routing;

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionAware;

SwitchOrExpander::SwitchOrExpander(int npus_count, Bandwidth bandwidth, Latency latency, const std::string& inputfile, const std::string& routing_algorithm, bool use_resiliency) noexcept
        : BasicTopology(npus_count, npus_count + (npus_count/8), bandwidth, latency),
            switch_topology(npus_count, bandwidth, latency) {
    assert(npus_count > 0);
    assert(bandwidth > 0);
    assert(latency >= 0);

    basic_topology_type = TopologyBuildingBlock::SwitchOrExpander;
    use_moe_routing = std::make_shared<std::map<DeviceId, bool>>();
  
    // create switch topology
    switch_topology = Switch(npus_count, bandwidth, latency);
    std::cout << "[SwitchOrExpander] Switch topology created with " << npus_count << " NPUs." << std::endl;
    
    // build expander graph from file (if provided)
    if (!inputfile.empty()) {
        expander_topology = std::make_unique<ExpanderGraph>(npus_count, bandwidth, latency, inputfile, routing_algorithm, use_resiliency);
        std::cout << "[SwitchOrExpander] Expander graph loaded from file: " << inputfile << std::endl;
    }

    for (DeviceId id : expander_topology->get_all_device_ids()) {
        (*use_moe_routing)[id] = false;  // default to switch routing
    }
}

unsigned int SwitchOrExpander::get_distance(const DeviceId src, const DeviceId dest) const noexcept {
    if (src == dest) {
        return 0;
    }

    assert(*use_moe_routing[src] == *use_moe_routing[dest]); // both src and dest should use the same mode
    bool use_moe = (*use_moe_routing)[src];
    
    if (use_moe && expander_topology) {
        return expander_topology->get_distance(src, dest, std::set<DeviceId>(), 0);
    } else {
        // use switch topology distance
        return switch_topology.route(src, dest).size() - 1;
    }
}

int SwitchOrExpander::compute_hops_count(const DeviceId src, const DeviceId dest) const noexcept {
    assert(src != dest);

    assert(*use_moe_routing[src] == *use_moe_routing[dest]); // both src and dest should use the same mode
    bool use_moe = (*use_moe_routing)[src];
    
    if (use_moe && expander_topology) {
        return expander_topology->route(src, dest).size() - 1;
    } else {
        // use switch topology hops count
        return switch_topology.route(src, dest).size() - 1;
    }
}

Route SwitchOrExpander::route(DeviceId src, DeviceId dest) const noexcept {
    assert(0 <= src && src < npus_count);
    assert(0 <= dest && dest < npus_count);

    assert(*use_moe_routing[src] == *use_moe_routing[dest]); // both src and dest should use the same mode
    bool use_moe = (*use_moe_routing)[src];

    if (use_moe && expander_topology) {
        Route r = expander_topology->route(src, dest);
        for (const auto& device_id : r) {
            assert(use_moe_routing[device_id->get_id()] == true;); // all devices in the route should be in moe mode 
        }
        return r;
    }

    return switch_topology.route(src, dest);
}

std::map<DeviceId, std::vector<DeviceId>> SwitchOrExpander::get_adjacency_list() const noexcept {
    //use moe result if moe mode is enabled for any device
    bool use_moe = false;
    for (const auto& [device_id, moe_flag] : *use_moe_routing) {
        if (moe_flag) {
            use_moe = true;
            break;
        }
    }

    if (use_moe && expander_topology) {
        return expander_topology->adjacency_list;
    }
    return switch_topology.adjacency_list;
}