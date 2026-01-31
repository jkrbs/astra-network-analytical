/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#pragma once

#include "common/Type.h"
#include "congestion_aware/BasicTopology.h"
#include "congestion_aware/Topology.h"
#include <memory>
#include <vector>

using namespace NetworkAnalytical;

namespace NetworkAnalyticalCongestionAware {

/**
 * MultiDimTopology implements multi-dimensional network topologies
 * which can be constructed by stacking up multiple BasicTopology instances.
 */
class MultiDimTopology : public Topology {
  public:
    /**
     * Constructor.
     */
    MultiDimTopology() noexcept;
    /// Each NPU ID can be broken down into multiple dimensions.
    /// for example, if the topology size is [2, 8, 4] and the NPU ID is 31,
    /// then the NPU ID can be broken down into [1, 7, 1].
    using MultiDimAddress = std::vector<DeviceId>;
    /**
     * Implement the route method of Topology.
     */
    [[nodiscard]] Route route(DeviceId src, DeviceId dest) const noexcept override;

    /**
     * Override send to use the device from the chunk's route.
     * Since MultiDimTopology doesn't maintain a global device pool,
     * we use the device that's already in the chunk's route.
     */
    void send(std::unique_ptr<Chunk> chunk) noexcept override;

    /**
     * Add a dimension to the multi-dimensional topology.
     *
     * @param topology BasicTopology instance to be added.
     */
    void append_dimension(std::unique_ptr<BasicTopology> basic_topology) noexcept;
  /**
     * Initialize per-slice topologies on first use.
     */
    void ensure_slices_initialized() const noexcept;

    /**
     * Build per-slice topologies and validate reachability.
     */
    void build_slices_and_validate() const noexcept;

    /**
     * Compute the slice index for a given dimension and address.
     */
    [[nodiscard]] size_t get_slice_index(int dim, const MultiDimAddress& address) const noexcept;

    /**
     * Translate the NPU ID into a multi-dimensional address.
     *
     * @param npu_id id of the NPU
     * @return the same NPU in multi-dimensional address representation
     */
    [[nodiscard]] MultiDimAddress translate_address(DeviceId npu_id) const noexcept;

    /**
     * Given src and dest address in multi-dimensional form,
     * return the dimension where the transfer should happen.
     * i.e., the dimension where the src and dest addresses differ.
     *
     * @param src_address src NPU ID in multi-dimensional form
     * @param dest_address dest NPU ID in multi-dimensional form
     * @return the dimension where the transfer should happen
     */
    [[nodiscard]] int get_dim_to_transfer(const MultiDimAddress& src_address,
                                          const MultiDimAddress& dest_address) const noexcept;
  private:
    /// BasicTopology instances per dimension.
    std::vector<std::unique_ptr<BasicTopology>> topology_per_dim;

    /// Per-slice topologies for each dimension.
    mutable std::vector<std::vector<std::unique_ptr<BasicTopology>>> topology_slices_per_dim;

    /// Whether per-slice topologies have been instantiated.
    mutable bool slices_initialized = false;

  
};

}  // namespace NetworkAnalyticalCongestionAware
