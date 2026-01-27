#pragma once

#include "common/Type.h"
#include "congestion_unaware/BasicTopology.h"
#include "congestion_unaware/ExpanderGraph.h"
#include "congestion_unaware/Switch.h"

#include <map>
#include <memory>
#include <string>

namespace NetworkAnalyticalCongestionUnaware {

class SwitchOrExpander final : public BasicTopology {
  public:
    SwitchOrExpander(int npus_count, Bandwidth bandwidth, Latency latency,
                     const std::string& inputfile = std::string()) noexcept;
    unsigned int get_distance(const DeviceId src, const DeviceId dest) const noexcept;
    std::map<DeviceId, std::vector<DeviceId>> get_adjacency_list() const noexcept;
    [[nodiscard]] int compute_hops_count(DeviceId src, DeviceId dest) const noexcept override;
  private:
    Switch switch_topology;
    std::unique_ptr<ExpanderGraph> expander_topology;
};

} // namespace NetworkAnalyticalCongestionUnaware
