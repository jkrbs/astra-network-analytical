/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#pragma once

#include "common/Type.h"
#include "congestion_aware/Topology.h"

using namespace NetworkAnalytical;

namespace NetworkAnalyticalCongestionAware {

/**
 * BasicTopology defines 1D topology
 * such as Ring, FullyConnected, and Switch topology,
 * which can be used to construct multi-dimensional topology.
 */
class BasicTopology : public Topology {
  public:
    /**
     * Constructor.
     *
     * @param npus_count number of NPUs in the topology
     * @param devices_count number of devices in the topology
     * @param bandwidth bandwidth of each link
     * @param latency latency of each link
     */
    BasicTopology(int npus_count, int devices_count, Bandwidth bandwidth, Latency latency) noexcept;

    /**
     * Destructor.
     */
    virtual ~BasicTopology() noexcept;

    /**
     * Return the type of the basic topology
     * as a TopologyBuildingBlock enum class element.
     *
     * @return type of the basic topology
     */
    [[nodiscard]] TopologyBuildingBlock get_basic_topology_type() const noexcept;

    /**
     * Get the link latency of this basic topology.
     *
     * @return latency per link
     */
    [[nodiscard]] Latency get_latency() const noexcept;

    /**
     * Create a deep copy of this topology instance.
     * Used to instantiate per-slice topologies in multi-dimensional routing.
     */
    [[nodiscard]] virtual std::unique_ptr<BasicTopology> clone() const noexcept = 0;

  protected:
    /// bandwidth of each link
    Bandwidth bandwidth;

    /// latency of each link
    Latency latency;

    /// basic topology type
    TopologyBuildingBlock basic_topology_type;
};

}  // namespace NetworkAnalyticalCongestionAware
