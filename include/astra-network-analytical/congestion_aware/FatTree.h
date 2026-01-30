#pragma once

#include <map>
#include <vector>
#include <set>
#include <string>

#include "common/Type.h"
#include "congestion_aware/BasicTopology.h"
#include "congestion_aware/Switch.h"

using namespace NetworkAnalytical;

namespace NetworkAnalyticalCongestionAware {

class FatTree final: public BasicTopology {
    public:
        /**
         * Routing algorithm types for FatTree
         */
        enum class RoutingAlgorithm {
            Deterministic,  // Use deterministic routing (based on source/dest indices)
            Random          // Randomly select among valid paths
        };

        /**
         * Constructor for FatTree topology.
         * 
         * @param npus_count number of NPUs (can be less than k^3/4 for non-fully subscribed)
         * @param k the radix of the fat tree (determines number of pod levels and switch degrees)
         * @param bandwidth bandwidth of each link
         * @param latency latency of each link
         * @param routing_algorithm routing algorithm to use (default: Deterministic)
         */
        FatTree(int npus_count, int k, Bandwidth bandwidth, Latency latency, 
                const std::string& routing_algorithm = "Deterministic") noexcept;
        [[nodiscard]] Route route(DeviceId src, DeviceId dest) const noexcept override;
    private:
        [[nodiscard]] int compute_hops_count(DeviceId src, DeviceId dest) const noexcept;
        [[nodiscard]] static RoutingAlgorithm str2RoutingAlgorithm(const std::string& algo_str) noexcept;
        
        std::vector<Switch> leaf_switches;
        std::vector<Switch> spine_switches;
        std::vector<Switch> core_switches;
        int k;  // radix of the fat tree
        std::vector<int> npus_per_leaf;  // number of NPUs connected to each leaf switch
        std::vector<int> npu_to_leaf;  // mapping from NPU ID to leaf switch index
        RoutingAlgorithm routing_algorithm;  // routing algorithm mode
};
} // namespace NetworkAnalyticalCongestionAware