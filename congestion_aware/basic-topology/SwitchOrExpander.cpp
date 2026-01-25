#include "congestion_aware/SwitchOrExpander.h"
#include "congestion_aware/Switch.h"
#include "congestion_aware/ExpanderGraph.h"
#include "congestion_aware/BasicTopology.h"
#include <fstream>
#include <iostream>
#include <queue>
#include <algorithm>
#include <cassert>
// Simple local JSON-lite parsing to avoid dependency on json headers in this
// external tree. We only need `node_count` and `connected_graph_adjacency`.

// avoid depending on top-level include paths in this external tree; declare
// the global flag here instead of including the simulator header.
extern bool use_moe_routing;

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionAware;

SwitchOrExpander::SwitchOrExpander(int npus_count, Bandwidth bandwidth, Latency latency, const std::string& inputfile) noexcept
    : BasicTopology(npus_count, npus_count + 1, bandwidth, latency) {
    assert(npus_count > 0);
    assert(bandwidth > 0);
    assert(latency >= 0);

    // create switch device id (last device)
    switch_id = (DeviceId)npus_count;

    // connect npus to switch (bidirectional)
    for (auto i = 0; i < npus_count; i++) {
        connect(i, switch_id, bandwidth, latency, true);
    }

    // build expander adjacency from file (if provided)
    if (!inputfile.empty()) {
        build_expander_from_file(inputfile);
    }
}

void SwitchOrExpander::build_expander_from_file(const std::string& inputfile) {
    std::ifstream file(inputfile);
    if (!file.is_open()) {
        std::cerr << "[SwitchOrExpander] Failed to open expander JSON: " << inputfile << std::endl;
        return;
    }
    std::string s((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    file.close();

    // find node_count
    int node_count = 0;
    auto pos_nc = s.find("\"node_count\"");
    if (pos_nc != std::string::npos) {
        auto colon = s.find(':', pos_nc);
        if (colon != std::string::npos) {
            size_t i = colon + 1;
            while (i < s.size() && isspace((unsigned char)s[i])) i++;
            int sign = 1;
            if (s[i] == '-') { sign = -1; i++; }
            int val = 0;
            while (i < s.size() && isdigit((unsigned char)s[i])) { val = val*10 + (s[i]-'0'); i++; }
            node_count = val * sign;
        }
    }

    // find connected_graph_adjacency array and parse as vector<vector<int>>
    std::vector<std::vector<int>> adjacency;
    auto pos_adj = s.find("\"connected_graph_adjacency\"");
    if (pos_adj != std::string::npos) {
        auto start = s.find('[', pos_adj);
        if (start != std::string::npos) {
            size_t i = start;
            // parse nested arrays
            std::vector<int> current;
            int level = 0;
            std::string numbuf;
            for (; i < s.size(); ++i) {
                char c = s[i];
                if (c == '[') {
                    level++;
                    if (level == 2) current.clear();
                } else if (c == ']') {
                    if (!numbuf.empty()) { current.push_back(std::stoi(numbuf)); numbuf.clear(); }
                    if (level == 2) adjacency.push_back(current);
                    level--;
                } else if (c == ',' || isspace((unsigned char)c)) {
                    if (!numbuf.empty()) { current.push_back(std::stoi(numbuf)); numbuf.clear(); }
                } else if (c == '-' || isdigit((unsigned char)c)) {
                    numbuf.push_back(c);
                }
                // stop after closing the top-level array
                if (level == 0 && i > start) break;
            }
        }
    }

    // Use parsed adjacency vector; assume adjacency indexes map to 0..npus_count-1
    for (size_t node_id = 0; node_id < adjacency.size(); ++node_id) {
        for (int nb : adjacency[node_id]) {
            if ((DeviceId)node_id < (DeviceId)npus_count && (DeviceId)nb < (DeviceId)npus_count) {
                if ((int)node_id < nb) {
                    adjacency_list[(DeviceId)node_id].push_back((DeviceId)nb);
                    adjacency_list[(DeviceId)nb].push_back((DeviceId)node_id);
                    connect((DeviceId)node_id, (DeviceId)nb, bandwidth_per_dim[0], 0, true);
                }
            }
        }
    }
}

unsigned int SwitchOrExpander::get_distance(const DeviceId src, const DeviceId dest) const noexcept {
    if (use_moe_routing) {
        std::pair<DeviceId, DeviceId> node_pair = std::make_pair(src, dest);
        if (route_cache.find(node_pair) != route_cache.end()) {
            return (unsigned int)route_cache.at(node_pair).size();
        }

        if (src == dest) return 0;

        std::vector<unsigned int> dist(npus_count, UINT32_MAX);
        std::priority_queue<std::pair<unsigned int, DeviceId>, std::vector<std::pair<unsigned int, DeviceId>>, std::greater<std::pair<unsigned int, DeviceId>>> pq;
        dist[src] = 0; pq.push({0, src});
        while (!pq.empty()) {
            auto [d,u] = pq.top(); pq.pop();
            if (u == dest) return d;
            if (d > dist[u]) continue;
            for (const auto& nb : adjacency_list.at(u)) {
                unsigned int nd = d + 1;
                if (nd < dist[nb]) { dist[nb] = nd; pq.push({nd, nb}); }
            }
        }
        return UINT32_MAX;
    } else {
        return src == dest ? 0 : 2; // switch mode: src -> switch -> dest
    }
}

int SwitchOrExpander::compute_hops_count(const DeviceId src, const DeviceId dest) const noexcept {
    assert(src != dest);
    return (int)get_distance(src, dest);
}

Route SwitchOrExpander::route(DeviceId src, DeviceId dest) const noexcept {
    assert(0 <= src && src < npus_count);
    assert(0 <= dest && dest < npus_count);

    // If MoE routing enabled, use expander shortest path (via adjacency_list)
    if (use_moe_routing && !adjacency_list.empty()) {
        std::pair<DeviceId, DeviceId> node_pair = std::make_pair(src, dest);
        if (route_cache.find(node_pair) != route_cache.end()) {
            Route cached_route;
            for (const auto& device_id : route_cache.at(node_pair)) {
                cached_route.push_back(devices[device_id]);
            }
            return cached_route;
        }

        // BFS to find path
        std::map<DeviceId, DeviceId> parent;
        std::set<DeviceId> visited;
        std::vector<DeviceId> q;
        q.push_back(src); visited.insert(src); parent[src]=src;
        bool found=false;
        while (!q.empty() && !found) {
            DeviceId cur = q.front(); q.erase(q.begin());
            for (auto nb : adjacency_list.at(cur)) {
                if (visited.find(nb)==visited.end()) {
                    visited.insert(nb); parent[nb]=cur; q.push_back(nb);
                    if (nb==dest) { found=true; break; }
                }
            }
        }

        Route route;
        if (found) {
            std::vector<DeviceId> path; DeviceId cur = dest;
            while (cur != src) { path.push_back(cur); cur = parent[cur]; }
            path.push_back(src); std::reverse(path.begin(), path.end());
            for (auto d : path) route.push_back(devices[d]);
            // cache
            std::vector<DeviceId> device_id_path; for (auto d : path) device_id_path.push_back(d);
            route_cache[node_pair] = device_id_path;
        }
        return route;
    }

    // Otherwise emulate switch behavior: src -> switch -> dest
    Route route;
    route.push_back(devices[src]);
    route.push_back(devices[switch_id]);
    route.push_back(devices[dest]);
    return route;
}
