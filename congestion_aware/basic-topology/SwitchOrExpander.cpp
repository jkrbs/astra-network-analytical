#include "congestion_aware/SwitchOrExpander.h"
#include "congestion_aware/BasicTopology.h"
#include <cassert>
#include <iostream>

// avoid depending on top-level include paths in this external tree; declare
// the global flag here instead of including the simulator header.
bool use_moe_routing;

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionAware;

SwitchOrExpander::SwitchOrExpander(int npus_count, Bandwidth bandwidth, Latency latency, const std::string& inputfile) noexcept
        : BasicTopology(npus_count, npus_count + 1, bandwidth, latency),
            switch_topology(npus_count, bandwidth, latency) {
    assert(npus_count > 0);
    assert(bandwidth > 0);
    assert(latency >= 0);

    basic_topology_type = TopologyBuildingBlock::SwitchOrExpander;

    // create switch topology
    switch_topology = Switch(npus_count, bandwidth, latency);
    std::cout << "[SwitchOrExpander] Switch topology created with " << npus_count << " NPUs." << std::endl;
    
    // build expander graph from file (if provided)
    if (!inputfile.empty()) {
        expander_topology = std::make_unique<ExpanderGraph>(npus_count, bandwidth, latency, inputfile);
        std::cout << "[SwitchOrExpander] Expander graph loaded from file: " << inputfile << std::endl;
    }
}

unsigned int SwitchOrExpander::get_distance(const DeviceId src, const DeviceId dest) const noexcept {
    if (src == dest) {
        return 0;
    }

    if (use_moe_routing && expander_topology) {
        return expander_topology->get_distance(src, dest, std::set<DeviceId>(), 0);
    } else {
        // use switch topology distance
        return switch_topology.route(src, dest).size() - 1;
    }
}

int SwitchOrExpander::compute_hops_count(const DeviceId src, const DeviceId dest) const noexcept {
    assert(src != dest);
    if (use_moe_routing && expander_topology) {
        return expander_topology->route(src, dest).size() - 1;
    } else {
        // use switch topology hops count
        return switch_topology.route(src, dest).size() - 1;
    }
}

Route SwitchOrExpander::route(DeviceId src, DeviceId dest) const noexcept {
    assert(0 <= src && src < npus_count);
    assert(0 <= dest && dest < npus_count);

    if (use_moe_routing && expander_topology) {
        return expander_topology->route(src, dest);
    }

    return switch_topology.route(src, dest);
}

std::map<DeviceId, std::vector<DeviceId>> SwitchOrExpander::get_adjacency_list() const noexcept {
    if (use_moe_routing && expander_topology) {
        return expander_topology->adjacency_list;
    }
    return switch_topology.adjacency_list;
}