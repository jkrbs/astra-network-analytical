#pragma once
#include "congestion_aware/BasicTopology.h"
#include "congestion_aware/ExpanderGraph.h"
#include "congestion_aware/Switch.h"
#include <map>
#include <memory>
#include <string>

namespace NetworkAnalyticalCongestionAware {

class SwitchOrExpander final : public BasicTopology {
  public:
    SwitchOrExpander(int npus_count, Bandwidth bandwidth, Latency latency, const std::string& inputfile = std::string(), const std::string& routing_algorithm = std::string(), bool use_resiliency = false) noexcept;
    [[nodiscard]] Route route(DeviceId src, DeviceId dest) const noexcept override;
    unsigned int get_distance(const DeviceId src, const DeviceId dest) const noexcept;
    int compute_hops_count(const DeviceId src, const DeviceId dest) const noexcept;
    std::map<DeviceId, std::vector<DeviceId>> get_adjacency_list() const noexcept;
  private:
    Route remap_route_to_local(const Route& foreign_route) const noexcept;
    Switch switch_topology;
    std::unique_ptr<ExpanderGraph> expander_topology;
    DeviceId switch_id;
};

} // namespace NetworkAnalyticalCongestionAware
