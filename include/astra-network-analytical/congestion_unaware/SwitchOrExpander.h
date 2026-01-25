#pragma once

#include "common/Type.h"
#include "congestion_unaware/BasicTopology.h"

#include <string>
#include <vector>

namespace NetworkAnalyticalCongestionUnaware {

class SwitchOrExpander final : public BasicTopology {
  public:
    SwitchOrExpander(int npus_count, Bandwidth bandwidth, Latency latency,
                     const std::string& inputfile = std::string()) noexcept;

  private:
    [[nodiscard]] int compute_hops_count(DeviceId src, DeviceId dest) const noexcept override;
    void build_expander_from_file(const std::string& inputfile);

    bool has_expander;
    std::vector<std::vector<DeviceId>> adjacency_list;
};

} // namespace NetworkAnalyticalCongestionUnaware
