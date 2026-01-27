#include "congestion_unaware/SwitchOrExpander.h"
#include "congestion_unaware/ExpanderGraph.h"
#include "congestion_unaware/Switch.h"

#include <cassert>
#include <iostream>
#include <set>

// avoid depending on top-level include paths in this external tree; declare
// the global flag here instead of including the simulator header.
bool use_moe_routing;

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionUnaware;

SwitchOrExpander::SwitchOrExpander(int npus_count, Bandwidth bandwidth, Latency latency,
                                   const std::string& inputfile) noexcept
    : BasicTopology(npus_count, bandwidth, latency),
      switch_topology(npus_count, bandwidth, latency) {
    assert(npus_count > 0);
    assert(bandwidth > 0);
    assert(latency >= 0);

    basic_topology_type = TopologyBuildingBlock::SwitchOrExpander;

    // create switch topology
    switch_topology = Switch(npus_count, bandwidth, latency);
    std::cout << "[SwitchOrExpander] Switch topology created with " << npus_count << " NPUs." << std::endl;

    if (!inputfile.empty()) {
        expander_topology = std::make_unique<ExpanderGraph>(npus_count, 0, bandwidth, latency, inputfile);
        std::cout << "[SwitchOrExpander] Expander graph loaded from file: " << inputfile << std::endl;
    }
}

unsigned int SwitchOrExpander::get_distance(const DeviceId src, const DeviceId dest) const noexcept {
    if (src == dest) {
        return 0;
    }

    if (use_moe_routing && expander_topology) {
        return expander_topology->get_distance(src, dest, std::set<DeviceId>(), 0);
    }

    return 2;
}

int SwitchOrExpander::compute_hops_count(DeviceId src, DeviceId dest) const noexcept {
    assert(src >= 0 && dest >= 0);
    if (src == dest) return 0;

    if (use_moe_routing && expander_topology) {
        return static_cast<int>(expander_topology->get_distance(src, dest, std::set<DeviceId>(), 0));
    }

    // default to switch hops (src -> switch -> dest)
    return 2;
}

std::map<DeviceId, std::vector<DeviceId>> SwitchOrExpander::get_adjacency_list() const noexcept {
    if (use_moe_routing && expander_topology) {
        return expander_topology->adjacency_list;
    }
    return {};
}
