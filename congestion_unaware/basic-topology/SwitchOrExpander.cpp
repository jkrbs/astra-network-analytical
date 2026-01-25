#include "congestion_unaware/SwitchOrExpander.h"

#include <algorithm>
#include <cassert>
#include <cctype>
#include <fstream>
#include <iostream>
#include <queue>

// avoid depending on top-level include paths in this external tree; declare
// the global flag here instead of including the simulator header.
extern bool use_moe_routing;

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionUnaware;

SwitchOrExpander::SwitchOrExpander(int npus_count, Bandwidth bandwidth, Latency latency,
                                   const std::string& inputfile) noexcept
    : BasicTopology(npus_count, bandwidth, latency),
      has_expander(false) {
    assert(npus_count > 0);
    assert(bandwidth > 0);
    assert(latency >= 0);

    basic_topology_type = TopologyBuildingBlock::SwitchOrExpander;

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

    int node_count = 0;
    auto pos_nc = s.find("\"node_count\"");
    if (pos_nc != std::string::npos) {
        auto colon = s.find(':', pos_nc);
        if (colon != std::string::npos) {
            size_t i = colon + 1;
            while (i < s.size() && isspace((unsigned char)s[i])) i++;
            int sign = 1;
            if (i < s.size() && s[i] == '-') { sign = -1; i++; }
            int val = 0;
            while (i < s.size() && isdigit((unsigned char)s[i])) { val = val * 10 + (s[i] - '0'); i++; }
            node_count = val * sign;
        }
    }

    std::vector<std::vector<int>> adjacency;
    auto pos_adj = s.find("\"connected_graph_adjacency\"");
    if (pos_adj != std::string::npos) {
        auto start = s.find('[', pos_adj);
        if (start != std::string::npos) {
            size_t i = start;
            std::vector<int> current;
            int level = 0;
            std::string numbuf;
            for (; i < s.size(); ++i) {
                char c = s[i];
                if (c == '[') {
                    level++;
                    if (level == 2) current.clear();
                } else if (c == ']') {
                    if (!numbuf.empty()) {
                        current.push_back(std::stoi(numbuf));
                        numbuf.clear();
                    }
                    if (level == 2) {
                        adjacency.push_back(current);
                    }
                    level--;
                    if (level == 0) break;
                } else if (isdigit((unsigned char)c) || c == '-') {
                    numbuf.push_back(c);
                } else {
                    if (!numbuf.empty()) {
                        current.push_back(std::stoi(numbuf));
                        numbuf.clear();
                    }
                }
            }
        }
    }

    if (node_count <= 0 || adjacency.empty()) {
        std::cerr << "[SwitchOrExpander] Failed to parse expander JSON: " << inputfile << std::endl;
        return;
    }

    adjacency_list.clear();
    adjacency_list.resize((size_t)node_count);
    for (int i = 0; i < (int)adjacency.size(); i++) {
        for (int nb : adjacency[i]) {
            if (nb >= 0 && nb < node_count) {
                adjacency_list[i].push_back(nb);
            }
        }
    }

    has_expander = !adjacency_list.empty();
}

int SwitchOrExpander::compute_hops_count(DeviceId src, DeviceId dest) const noexcept {
    assert(src >= 0 && dest >= 0);
    if (src == dest) return 0;

    if (use_moe_routing && has_expander &&
        src < static_cast<DeviceId>(adjacency_list.size()) &&
        dest < static_cast<DeviceId>(adjacency_list.size())) {
        std::queue<DeviceId> q;
        std::vector<int> dist(adjacency_list.size(), -1);
        dist[src] = 0;
        q.push(src);
        while (!q.empty()) {
            auto u = q.front();
            q.pop();
            if (u == dest) break;
            for (auto v : adjacency_list[u]) {
                if (dist[v] < 0) {
                    dist[v] = dist[u] + 1;
                    q.push(v);
                }
            }
        }
        if (dist[dest] >= 0) {
            return dist[dest];
        }
    }

    // default to switch hops (src -> switch -> dest)
    return 2;
}
