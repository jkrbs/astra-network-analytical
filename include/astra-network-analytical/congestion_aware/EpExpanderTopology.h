/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#pragma once

#include <map>
#include <vector>
#include <string>
#include <random>

#include "common/Type.h"
#include "congestion_aware/BasicTopology.h"

using namespace NetworkAnalytical;

namespace NetworkAnalyticalCongestionAware {

/**
 * RouteInfo holds information about a single route between two nodes.
 */
struct RouteInfo {
    std::vector<DeviceId> path;  // List of device IDs in the path
    int hops;                     // Number of hops
    double weight;                // Weight for probabilistic selection
};

/**
 * EpExpanderTopology implements an expander topology for EP P2P communication.
 * 
 * Unlike ExpanderGraph which builds routes via BFS, this topology uses
 * pre-computed weighted routes from a JSON file. Routes are selected
 * probabilistically based on their weights.
 * 
 * The topology also supports per-layer permutation of node mappings to
 * distribute load across different layers.
 */
class EpExpanderTopology final : public BasicTopology {
  public:
    /**
     * Constructor.
     *
     * @param routes_file path to JSON file containing pre-computed routes
     * @param bandwidth bandwidth of each link
     * @param latency latency of each link
     */
    EpExpanderTopology(const std::string& routes_file, Bandwidth bandwidth, Latency latency) noexcept;

    /**
     * Implementation of route function in Topology.
     * Selects a route probabilistically based on weights.
     */
    [[nodiscard]] Route route(DeviceId src, DeviceId dest) const noexcept override;

    /**
     * Route with layer-specific permutation.
     * Maps src/dst through the layer's permutation before routing.
     *
     * @param src source device (local EP rank, 0 to N-1)
     * @param dest destination device (local EP rank, 0 to N-1)
     * @param layer_id layer ID for determining permutation
     * @return route through the expander
     */
    [[nodiscard]] Route route_with_permutation(DeviceId src, DeviceId dest, int layer_id) const noexcept;

    /**
     * Get all routes with layer-specific permutation for packet spraying.
     * Returns all available routes for the src-dst pair (after permutation).
     *
     * @param src source device (local EP rank, 0 to N-1)
     * @param dest destination device (local EP rank, 0 to N-1)
     * @param layer_id layer ID for determining permutation
     * @return vector of all routes through the expander
     */
    [[nodiscard]] std::vector<Route> get_all_routes_with_permutation(DeviceId src, DeviceId dest, int layer_id) const noexcept;

    /**
     * Get or generate the permutation for a given layer.
     * Permutations are cached for efficiency.
     *
     * @param layer_id layer ID
     * @return reference to permutation vector
     */
    [[nodiscard]] const std::vector<int>& get_permutation(int layer_id) const noexcept;

    /**
     * Clone this topology instance.
     */
    [[nodiscard]] std::unique_ptr<BasicTopology> clone() const noexcept override;

    /**
     * Get the number of nodes in the expander.
     */
    [[nodiscard]] int get_node_count() const noexcept { return node_count; }

    /**
     * Get the degree of the expander.
     */
    [[nodiscard]] int get_degree() const noexcept { return degree; }

    /**
     * Set the number of unique permutation layers.
     * When set to N > 0, permutations repeat every N layers (layer_id % N).
     * When set to 0, each layer gets a unique permutation (no repetition).
     * Default is 0 (no repetition).
     *
     * @param num_layers number of unique permutation layers (0 = no limit)
     */
    void set_num_permutation_layers(int num_layers) noexcept { 
        num_permutation_layers = num_layers;
        // Clear cache when changing this setting
        layer_permutations.clear();
    }

    /**
     * Get the number of unique permutation layers.
     */
    [[nodiscard]] int get_num_permutation_layers() const noexcept { return num_permutation_layers; }

  private:
    /// Path to the routes JSON file
    std::string routes_file_path;

    /// Number of nodes in the expander (including switch if present)
    int node_count;

    /// Number of EP nodes (excluding switch, used for permutation)
    int ep_node_count;

    /// Degree of the expander graph
    int degree;

    /// Pre-computed routes: routes[src][dst] = vector of RouteInfo
    std::map<DeviceId, std::map<DeviceId, std::vector<RouteInfo>>> routes;

    /// Adjacency list for building links
    std::map<DeviceId, std::vector<DeviceId>> adjacency_list;

    /// Cached permutations per layer: layer_permutations[layer_id] = permutation
    mutable std::map<int, std::vector<int>> layer_permutations;

    /// Number of unique permutation layers (0 = no limit, N > 0 = repeat every N layers)
    /// Default 0 means each layer gets a unique permutation
    int num_permutation_layers = 0;

    /// Random number generator for route selection
    mutable std::mt19937 rng;

    /**
     * Load routes from JSON file.
     */
    void load_routes(const std::string& routes_file);

    /**
     * Build the expander graph links from the routes.
     */
    void build_links_from_routes();

    /**
     * Select a route probabilistically based on weights.
     */
    [[nodiscard]] const RouteInfo& select_route(DeviceId src, DeviceId dest) const noexcept;
};

}  // namespace NetworkAnalyticalCongestionAware
