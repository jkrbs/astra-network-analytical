/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "congestion_aware/MultiDimTopology.h"
#include "congestion_aware/Chunk.h"
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <algorithm>

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionAware;

MultiDimTopology::MultiDimTopology() noexcept : Topology() {
    // initialize values
    topology_per_dim.clear();
    npus_count_per_dim = {};
    topology_slices_per_dim.clear();
    latency_per_dim.clear();
    slices_initialized = false;

    // initialize topology shape
    npus_count = 1;
    devices_count = 1;
    dims_count = 0;

    // validate reachability (no-op until dimensions are appended)
    for (auto dim = 0; dim < dims_count; dim++) {
        assert(npus_count_per_dim[dim] > 0);
    }
}

Route MultiDimTopology::route(const DeviceId src, const DeviceId dest) const noexcept {
    // translate src and dest to multi-dim address
    auto current_address = translate_address(src);
    const auto dest_address = translate_address(dest);

    // helper: convert multi-dim address to global id
    const auto address_to_global = [this](const MultiDimAddress& address) -> DeviceId {
        DeviceId global_id = 0;
        for (int d = dims_count - 1; d >= 0; --d) {
            global_id = global_id * npus_count_per_dim[d] + address[d];
        }
        return global_id;
    };

    Route route;
    route.push_back(get_device_from_global_id(src));

    // Dimension-ordered routing: traverse each dimension independently
    for (int dim = 0; dim < dims_count; ++dim) {
        if (current_address[dim] == dest_address[dim]) {
            continue;
        }

        auto* const topology = topology_per_dim[dim].get();
        const auto src_local_id = current_address[dim];
        const auto dest_local_id = dest_address[dim];
        auto local_route = topology->route(src_local_id, dest_local_id);

        std::vector<DeviceId> local_ids;
        local_ids.reserve(local_route.size() + 2);
        for (const auto& local_dev : local_route) {
            DeviceId local_id = local_dev->get_id();
            if (local_id >= npus_count_per_dim[dim]) {
                // Treat as a global id and extract this dimension's coordinate
                const auto addr = translate_address(local_id);
                local_id = addr[dim];
            }
            local_ids.push_back(local_id);
        }
        if (local_ids.empty()) {
            local_ids.push_back(src_local_id);
        }
        if (local_ids.back() != dest_local_id) {
            local_ids.push_back(dest_local_id);
        }

        bool first = true;
        for (const auto local_id : local_ids) {
            assert(local_id >= 0 && local_id < npus_count_per_dim[dim]);
            if (first && local_id == current_address[dim]) {
                first = false;
                continue;  // skip duplicate current node
            }
            first = false;

            current_address[dim] = local_id;
            const auto global_id = address_to_global(current_address);

            if (route.empty() || route.back()->get_id() != global_id) {
                // lazily connect consecutive nodes along this dimension
                if (!route.empty()) {
                    const auto prev_id = route.back()->get_id();
                    if (!devices[prev_id]->connected(global_id)) {
                        const_cast<MultiDimTopology*>(this)->connect(
                            prev_id,
                            global_id,
                            bandwidth_per_dim[dim],
                            latency_per_dim[dim],
                            true);
                    }
                }
                route.push_back(get_device_from_global_id(global_id));
            }
        }
    }

    assert(route.front()->get_id() == src);
    assert(route.back()->get_id() == dest);

    // Assert all devices in the route are non-null
    for (const auto& dev : route) {
        assert(dev != nullptr && "Null device in route");
    }

    // Assert each consecutive pair is connected
    for (auto it = route.begin(); it != route.end();) {
        auto curr = it++;
        if (it == route.end()) break;
        auto next = it;
        assert((*curr)->connected((*next)->get_id()) && "Consecutive devices in route are not connected");
    }

    return route;
}

// Helper: get the Device pointer for a global DeviceId
std::shared_ptr<Device> MultiDimTopology::get_device_from_global_id(DeviceId global_id) const {
    assert(global_id >= 0 && global_id < devices_count);
    return devices[global_id];
}

void MultiDimTopology::send(std::unique_ptr<Chunk> chunk) noexcept {
    assert(chunk != nullptr);

    // MultiDimTopology doesn't maintain a global device pool.
    // Instead, the route contains devices from the specific dimension's topology.
    // We simply delegate to the current device from the chunk's route.
    auto current_device = chunk->current_device();
    assert(current_device != nullptr);
    current_device->send(std::move(chunk));
}

void MultiDimTopology::append_dimension(std::unique_ptr<BasicTopology> topology) noexcept {
    // increment dims_count
    dims_count++;

    // increase npus_count
    const auto topology_size = topology->get_npus_count();
    npus_count *= topology_size;
    devices_count = npus_count;  // MultiDimTopology has same devices as NPUs

    // append bandwidth
    const auto bandwidth = topology->get_bandwidth_per_dim()[0];
    bandwidth_per_dim.push_back(bandwidth);

    const auto latency = topology->get_latency();
    latency_per_dim.push_back(latency);

    // push back topology and npus_count
    topology_per_dim.push_back(std::move(topology));
    npus_count_per_dim.push_back(topology_size);

    // reset per-slice initialization (dimensions changed)
    slices_initialized = false;

    // rebuild global devices for the updated NPU count
    devices.clear();
    instantiate_devices();

    // eagerly build per-slice topologies for all dimensions
    build_slices_and_validate();
}

void MultiDimTopology::ensure_slices_initialized() const noexcept {
    if (slices_initialized) {
        return;
    }

    build_slices_and_validate();
}

void MultiDimTopology::build_slices_and_validate() const noexcept {
    topology_slices_per_dim.clear();
    topology_slices_per_dim.resize(dims_count);

    for (auto dim = 0; dim < dims_count; dim++) {
        size_t slices_count = 1;
        for (auto d = 0; d < dims_count; d++) {
            if (d == dim) {
                continue;
            }
            slices_count *= static_cast<size_t>(npus_count_per_dim[d]);
        }

        topology_slices_per_dim[dim].reserve(slices_count);
        for (size_t i = 0; i < slices_count; i++) {
            topology_slices_per_dim[dim].push_back(topology_per_dim[dim]->clone());
        }
    }

    // validate reachability in each slice
    for (auto dim = 0; dim < dims_count; dim++) {
        const auto npus = npus_count_per_dim[dim];
        for (const auto& slice : topology_slices_per_dim[dim]) {
            for (DeviceId src = 0; src < npus; src++) {
                for (DeviceId dest = 0; dest < npus; dest++) {
                    const auto route = slice->route(src, dest);
                    assert(!route.empty());
                    assert(route.front()->get_id() == src);
                    assert(route.back()->get_id() == dest);
                }
            }
        }
    }

    slices_initialized = true;
}

size_t MultiDimTopology::get_slice_index(const int dim, const MultiDimAddress& address) const noexcept {
    size_t index = 0;
    for (auto d = 0; d < dims_count; d++) {
        if (d == dim) {
            continue;
        }
        index = index * static_cast<size_t>(npus_count_per_dim[d]) + static_cast<size_t>(address[d]);
    }

    return index;
}

MultiDimTopology::MultiDimAddress MultiDimTopology::translate_address(const DeviceId npu_id) const noexcept {
    // If units-count if [2, 8, 4], and the given id is 47, then the id should be
    // 47 // 16 = 2, leftover = 47 % 16 = 15
    // 15 // 2 = 7, leftover = 15 % 2 = 1
    // 1 // 1 = 1, leftover = 0
    // therefore the address is [1, 7, 2]

    // create empty address
    auto multi_dim_address = MultiDimAddress();
    for (auto i = 0; i < dims_count; i++) {
        multi_dim_address.push_back(-1);
    }

    auto leftover = npu_id;
    auto denominator = npus_count;

    for (auto dim = dims_count - 1; dim >= 0; dim--) {
        // change denominator
        denominator /= npus_count_per_dim[dim];

        // get and update address
        const auto quotient = leftover / denominator;
        leftover %= denominator;

        // update address
        multi_dim_address[dim] = quotient;
    }

    // check address translation
    for (auto i = 0; i < dims_count; i++) {
        assert(0 <= multi_dim_address[i]);
        assert(multi_dim_address[i] < npus_count_per_dim[i]);
    }

    // return retrieved address
    return multi_dim_address;
}

int MultiDimTopology::get_dim_to_transfer(const MultiDimAddress& src_address,
                                          const MultiDimAddress& dest_address) const noexcept {
    for (auto dim = 0; dim < dims_count; dim++) {
        // check the dim that has different address
        if (src_address[dim] != dest_address[dim]) {
            return dim;
        }
    }

    // shouldn't reach here
    std::cerr << "[Error] (network/analytical/congestion_unaware): " << "src and dest have the same address"
              << std::endl;
    std::exit(-1);
}


// MultiDimTopology::MultiDimAddress MultiDimTopology::translate_address(const DeviceId npu_id) const noexcept {
//     // If units-count if [2, 8, 4], and the given id is 47, then the id should be
//     // 47 // 16 = 2, leftover = 47 % 16 = 15
//     // 15 // 2 = 7, leftover = 15 % 2 = 1
//     // 1 // 1 = 1, leftover = 0
//     // therefore the address is [1, 7, 2]

//     // create empty address
//     auto multi_dim_address = MultiDimAddress();
//     for (auto i = 0; i < dims_count; i++) {
//         multi_dim_address.push_back(-1);
//     }

//     auto leftover = npu_id;
//     auto denominator = npus_count;

//     for (auto dim = dims_count - 1; dim >= 0; dim--) {
//         // change denominator
//         denominator /= npus_count_per_dim[dim];

//         // get and update address
//         const auto quotient = leftover / denominator;
//         leftover %= denominator;

//         // update address
//         multi_dim_address[dim] = quotient;
//     }

//     // check address translation
//     for (auto i = 0; i < dims_count; i++) {
//         assert(0 <= multi_dim_address[i]);
//         assert(multi_dim_address[i] < npus_count_per_dim[i]);
//     }

//     // return retrieved address
//     return multi_dim_address;
// }

// int MultiDimTopology::get_dim_to_transfer(const MultiDimAddress& src_address,
//                                           const MultiDimAddress& dest_address) const noexcept {
//     auto dim_to_transfer = -1;
//     auto diffs = 0;

//     for (auto dim = 0; dim < dims_count; dim++) {
//         // check the dim that has different address
//         if (src_address[dim] != dest_address[dim]) {
//             dim_to_transfer = dim;
//             diffs++;
//         }
//     }

//     if (diffs == 1) {
//         return dim_to_transfer;
//     }

//     if (diffs == 0) {
//         std::cerr << "[Error] (network/analytical/congestion_aware): src and dest have the same address"
//                   << std::endl;
//     } else {
//         std::cerr << "[Error] (network/analytical/congestion_aware): src and dest differ in " << diffs
//                   << " dimensions. MultiDimTopology only supports single-dimension transfers." << std::endl;
//     }

//     std::exit(-1);
// }
