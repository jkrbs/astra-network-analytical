/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "common/EventQueue.h"
#include "common/NetworkParser.h"
#include "common/Type.h"
#include "congestion_aware/Chunk.h"
#include "congestion_aware/Helper.h"
#include "congestion_aware/ExpanderGraph.h"
#include "congestion_aware/SwitchOrExpander.h"
#include <gtest/gtest.h>

extern std::shared_ptr<std::map<NetworkAnalytical::DeviceId, bool>> use_moe_routing;

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionAware;

class TestNetworkAnalyticalCongestionAware : public ::testing::Test {
  protected:
    void SetUp() override {
        // set event queue
        event_queue = std::make_shared<EventQueue>();
        Topology::set_event_queue(event_queue);

        // set chunk size
        chunk_size = 1'048'576;  // 1 MB
    }

    std::shared_ptr<EventQueue> event_queue;

    static void callback(void* const arg) {}

    ChunkSize chunk_size;
};

TEST_F(TestNetworkAnalyticalCongestionAware, Ring) {
    /// setup
    const auto network_parser = NetworkParser("../../input/Ring.yml");
    const auto topology = construct_topology(network_parser);

    /// message settings
    auto route = topology->route(1, 4);
    auto chunk = std::make_unique<Chunk>(chunk_size, route, callback, nullptr);

    // send a chunk
    topology->send(std::move(chunk));

    /// Run simulation
    while (!event_queue->finished()) {
        event_queue->proceed();
    }

    /// test
    const auto simulation_time = event_queue->get_current_time();
    EXPECT_EQ(simulation_time, 60'093);
}

TEST_F(TestNetworkAnalyticalCongestionAware, FullyConnected) {
    /// setup
    const auto network_parser = NetworkParser("../../input/FullyConnected.yml");
    const auto topology = construct_topology(network_parser);

    /// message settings
    auto route = topology->route(1, 4);
    auto chunk = std::make_unique<Chunk>(chunk_size, route, callback, nullptr);

    // send a chunk
    topology->send(std::move(chunk));

    /// Run simulation
    while (!event_queue->finished()) {
        event_queue->proceed();
    }

    /// test
    const auto simulation_time = event_queue->get_current_time();
    EXPECT_EQ(simulation_time, 20'031);
}

TEST_F(TestNetworkAnalyticalCongestionAware, Switch) {
    /// setup
    const auto network_parser = NetworkParser("../../input/Switch.yml");
    const auto topology = construct_topology(network_parser);

    /// message settings
    auto route = topology->route(1, 4);
    auto chunk = std::make_unique<Chunk>(chunk_size, route, callback, nullptr);

    // send a chunk
    topology->send(std::move(chunk));

    /// Run simulation
    while (!event_queue->finished()) {
        event_queue->proceed();
    }

    /// test
    const auto simulation_time = event_queue->get_current_time();
    EXPECT_EQ(simulation_time, 40'062);
}

TEST_F(TestNetworkAnalyticalCongestionAware, AllGatherOnRing) {
    /// setup
    const auto network_parser = NetworkParser("../../input/Ring.yml");
    const auto topology = construct_topology(network_parser);
    const auto npus_count = topology->get_npus_count();

    /// message settings
    const auto chunk_size = 1'048'576;  // 1 MB

    /// Run All-Gather
    for (int i = 0; i < npus_count; i++) {
        for (int j = 0; j < npus_count; j++) {
            if (i == j) {
                continue;
            }

            // crate a chunk
            auto route = topology->route(i, j);
            auto* event_queue_ptr = static_cast<void*>(event_queue.get());
            auto chunk = std::make_unique<Chunk>(chunk_size, route, callback, nullptr);

            // send a chunk
            topology->send(std::move(chunk));
        }
    }

    /// Run simulation
    while (!event_queue->finished()) {
        event_queue->proceed();
    }

    /// test
    const auto simulation_time = event_queue->get_current_time();
    EXPECT_EQ(simulation_time, 704'116);
}

TEST_F(TestNetworkAnalyticalCongestionAware, ExpanderGraph) {
    // create network
    const auto network_parser = NetworkParser("../../input/ExpanderGraph.yml");
    const auto topology = construct_topology(network_parser);
    
    // assert topology type
    auto graph = std::dynamic_pointer_cast<ExpanderGraph>(topology);
    ASSERT_NE(graph, nullptr);

    // validate that every node has degree 8
    for (DeviceId i = 0; i < network_parser.get_npus_counts_per_dim()[0]; ++i) {
        const auto& neighbors = graph->adjacency_list.at(i);
        EXPECT_EQ(neighbors.size(), 8);
    }

    unsigned int total_distance = 0;
    unsigned int count = 0;

    for (DeviceId i = 0; i < network_parser.get_npus_counts_per_dim()[0]/2; ++i) {
        for (DeviceId j = 0; j < network_parser.get_npus_counts_per_dim()[0]; ++j) {
            if (i == j) {
                continue;
            }
            // all distances should be <= N/2
            const auto route = graph->route(i, j);
            std::cout << std::endl;
            EXPECT_LE(route.size(), network_parser.get_npus_counts_per_dim()[0]/2);

            total_distance += route.size();
            count++;
        
            auto chunk = std::make_unique<Chunk>(1, route, callback, nullptr);
            //validate communicatio delay
            topology->send(std::move(chunk));
            auto send_time = event_queue->get_current_time();
            /// Run simulation
            while (!event_queue->finished()) {
                event_queue->proceed();
            }
            auto comm_delay = event_queue->get_current_time() - send_time;
            // event_queue->reset();

            // expected delay = (link count) * latency per link, link count = route.size() - 1
            const auto expected_delay = (route.size() - 1) * network_parser.get_latencies_per_dim()[0];
            EXPECT_EQ(comm_delay, expected_delay);
        }
    }

    // average distance
    const double average_distance = static_cast<double>(total_distance) / static_cast<double>(count);
    std::cout << "Average distance in ExpanderGraph: " << average_distance << std::endl;
    EXPECT_LE(average_distance, network_parser.get_npus_counts_per_dim()[0]/4.0);
}

TEST_F(TestNetworkAnalyticalCongestionAware, ExpanderGraph_Splitted) {
    // create network
    const auto network_parser = NetworkParser("../../input/ExpanderGraph_Splitted.yml");
    const auto topology = construct_topology(network_parser);
    
    // assert topology type
    auto graph = std::dynamic_pointer_cast<ExpanderGraph>(topology);
    ASSERT_NE(graph, nullptr);

    // using resilient expander. Ensure devices = npus + (npus/8)
    auto devices = graph->get_devices_count();
    auto npus = graph->get_npus_count();
    EXPECT_EQ(devices, 36);
    EXPECT_EQ(npus + (npus/8), devices);

    // validate that every node has degree 8
    for (DeviceId i = 0; i < network_parser.get_npus_counts_per_dim()[0]; ++i) {
        const auto& neighbors = graph->adjacency_list.at(i);
        EXPECT_EQ(neighbors.size(), 8);
    }

    unsigned int total_distance = 0;
    unsigned int count = 0;

    for (DeviceId i = 0; i < network_parser.get_npus_counts_per_dim()[0]/2; ++i) {
        for (DeviceId j = 0; j < network_parser.get_npus_counts_per_dim()[0]; ++j) {
            if (i == j) {
                continue;
            }
            // all distances should be <= N/2
            const auto route = graph->route(i, j);
            std::cout << std::endl;
            EXPECT_LE(route.size(), network_parser.get_npus_counts_per_dim()[0]/2 +1);

            total_distance += route.size();
            count++;
        
            auto chunk = std::make_unique<Chunk>(1, route, callback, nullptr);
            //validate communicatio delay
            topology->send(std::move(chunk));
            auto send_time = event_queue->get_current_time();
            /// Run simulation
            while (!event_queue->finished()) {
                event_queue->proceed();
            }
            auto comm_delay = event_queue->get_current_time() - send_time;
            // event_queue->reset();

            // expected delay = (link count) * latency per link, link count = route.size() - 1
            const auto expected_delay = (route.size() - 1) * network_parser.get_latencies_per_dim()[0];
            EXPECT_EQ(comm_delay, expected_delay);
        }
    }

    // average distance
    const double average_distance = static_cast<double>(total_distance) / static_cast<double>(count);
    std::cout << "Average distance in ExpanderGraph: " << average_distance << std::endl;
    EXPECT_LE(average_distance, network_parser.get_npus_counts_per_dim()[0]/4.0);
}

TEST_F(TestNetworkAnalyticalCongestionAware, SwitchOrExpander) {
    // create network
    const auto network_parser = NetworkParser("../../input/SwitchOrExpander.yml");
    const auto topology = construct_topology(network_parser);

    // assert topology type
    auto graph = std::dynamic_pointer_cast<SwitchOrExpander>(topology);
    ASSERT_NE(graph, nullptr);

       // using resilient expander. Ensure devices = npus + (npus/8)
    auto devices = graph->get_devices_count();
    auto npus = graph->get_npus_count();
    EXPECT_EQ(devices, 18);
    EXPECT_EQ(npus + (npus/8), devices);

    // test moe mode
    for (auto device_id : topology->get_all_device_ids()) {
        (*use_moe_routing)[device_id] = true;
    }

    // validate that every node has degree 8
    for (DeviceId i = 0; i < network_parser.get_npus_counts_per_dim()[0]; ++i) {
        const auto& neighbors = graph->get_adjacency_list().at(i);
        EXPECT_EQ(neighbors.size(), 4);
    }


    for (DeviceId i = 0; i < network_parser.get_npus_counts_per_dim()[0]; ++i) {
        for (DeviceId j = 0; j < network_parser.get_npus_counts_per_dim()[0]; ++j) {
            if (i == j) {
                continue;
            }
            const auto distance = graph->compute_hops_count(i, j);
            // in moe mode, distances should be <= log8(N)
            EXPECT_LE(distance, 4);
            const auto route = graph->route(i, j);
            
            EXPECT_LE(route.size(), 5);
            EXPECT_EQ(distance, route.size() -1);
        }
    }

    // test switch mode
    for (auto device_id : topology->get_all_device_ids()) {
        (*use_moe_routing)[device_id] = false;
    }

    for (DeviceId i = 0; i < network_parser.get_npus_counts_per_dim()[0]; ++i) {
        for (DeviceId j = 0; j < network_parser.get_npus_counts_per_dim()[0]; ++j) {
            if (i == j) {
                continue;
            }
            const auto distance = graph->compute_hops_count(i, j);
            const auto route = graph->route(i, j);
            // in switch mode, all routes should be direct via switch hop
            EXPECT_EQ(route.size(), 3);
            EXPECT_EQ(distance, 2);
        }
    }
}

// test splitted expander graph
TEST_F(TestNetworkAnalyticalCongestionAware, SwitchOrExpander_Splitted) {
    // create network
    const auto network_parser = NetworkParser("../../input/SwitchOrExpander_Splitted.yml");
    const auto topology = construct_topology(network_parser);

    // assert topology type
    auto graph = std::dynamic_pointer_cast<SwitchOrExpander>(topology);
    ASSERT_NE(graph, nullptr);

    // test moe mode
    for (auto device_id : topology->get_all_device_ids()) {
        (*use_moe_routing)[device_id] = true;
    }

       // using resilient expander. Ensure devices = npus + (npus/8)
    auto devices = graph->get_devices_count();
    auto npus = graph->get_npus_count();
    EXPECT_EQ(devices, 18);
    EXPECT_EQ(npus + (npus/8), devices);

    // validate that every node has degree 8
    for (DeviceId i = 0; i < network_parser.get_npus_counts_per_dim()[0]; ++i) {
        const auto& neighbors = graph->get_adjacency_list().at(i);
        EXPECT_EQ(neighbors.size(), 8);
    }


    for (DeviceId i = 0; i < network_parser.get_npus_counts_per_dim()[0]; ++i) {
        for (DeviceId j = 0; j < network_parser.get_npus_counts_per_dim()[0]; ++j) {
            if (i == j) {
                continue;
            }
            const auto distance = graph->compute_hops_count(i, j);
            // in moe mode, distances should be <= log8(N)
            EXPECT_LE(distance, 3);
            const auto route = graph->route(i, j);
            
            EXPECT_LE(route.size(), 4);
            EXPECT_EQ(distance, route.size() -1);
        }
    }

    // test switch mode
    for (auto device_id : topology->get_all_device_ids()) {
        (*use_moe_routing)[device_id] = false;
    }

    for (DeviceId i = 0; i < network_parser.get_npus_counts_per_dim()[0]; ++i) {
        for (DeviceId j = 0; j < network_parser.get_npus_counts_per_dim()[0]; ++j) {
            if (i == j) {
                continue;
            }
            const auto distance = graph->compute_hops_count(i, j);
            const auto route = graph->route(i, j);
            // in switch mode, all routes should be direct via switch hop
            EXPECT_EQ(route.size(), 3);
            EXPECT_EQ(distance, 2);
        }
    }
}