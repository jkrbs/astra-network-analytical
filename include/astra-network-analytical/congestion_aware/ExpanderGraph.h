/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#pragma once

#include <map>
#include <vector>
#include <set>
#include <string>

#include "common/Type.h"
#include "congestion_aware/BasicTopology.h"

using namespace NetworkAnalytical;

namespace NetworkAnalyticalCongestionAware {
/**
 * Implements a ExpanderGraph topology.
 *
 * ExpanderGraph(4) example:
 *
 * Therefore, arbitrary send between two pair of NPUs will take N hops on average.
 */
class ExpanderGraph final : public BasicTopology {
  public:
    /**
     * Constructor.
     *
     * @param npus_count number of NPUs in the ExpanderGraph topology
     * @param degree degree of the expander graph (unused if inputfile provided)
     * @param bandwidth bandwidth of each link
     * @param latency latency of each link
     * @param inputfile path to JSON file defining the expander graph topology
     */
    ExpanderGraph(int npus_count, unsigned int degree, Bandwidth bandwidth, Latency latency, const std::string& inputfile = std::string()) noexcept;
    unsigned int get_distance(const DeviceId src, const DeviceId dest, std::set<DeviceId> visited, unsigned int current_distance) const noexcept;
    std::map<DeviceId, std::vector<DeviceId>> adjacency_list;
    [[nodiscard]] Route route(DeviceId src, DeviceId dest) const noexcept override;
  private:
    /**
     * Implements the compute_hops_count method of BasicTopology.
     */
    [[nodiscard]] int compute_hops_count(DeviceId src, DeviceId dest) const noexcept;
    // distance cache to speed up distance calculations
    mutable std::map<std::pair<DeviceId, DeviceId>, unsigned int> distance_cache;
    void connect(DeviceId src, DeviceId dest);
    mutable std::map<std::pair<DeviceId, DeviceId>, std::vector<DeviceId>> route_cache;
};

}  // namespace NetworkAnalyticalCongestionAware
