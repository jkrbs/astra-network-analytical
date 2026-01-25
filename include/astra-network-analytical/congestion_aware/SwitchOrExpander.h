#pragma once
#include "congestion_aware/BasicTopology.h"
#include <string>
#include <map>

namespace NetworkAnalyticalCongestionAware {

class SwitchOrExpander final : public BasicTopology {
  public:
    SwitchOrExpander(int npus_count, Bandwidth bandwidth, Latency latency, const std::string& inputfile = std::string()) noexcept;
    [[nodiscard]] Route route(DeviceId src, DeviceId dest) const noexcept override;
    unsigned int get_distance(const DeviceId src, const DeviceId dest) const noexcept;
    int compute_hops_count(const DeviceId src, const DeviceId dest) const noexcept;
    std::map<DeviceId, std::vector<DeviceId>> adjacency_list;
  private:
    // adjacency list for expander routing
    mutable std::map<std::pair<DeviceId, DeviceId>, std::vector<DeviceId>> route_cache;
    void build_expander_from_file(const std::string& inputfile);
    DeviceId switch_id;
};

} // namespace NetworkAnalyticalCongestionAware
