/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "common/NetworkParser.h"
#include "common/Type.h"
#include "congestion_unaware/Helper.h"
#include "congestion_unaware/ExpanderGraph.h"
#include <gtest/gtest.h>

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionUnaware;

class TestNetworkAnalyticalCongestionUnaware : public ::testing::Test {
  protected:
    void SetUp() override {
        // set chunk size
        chunk_size = 1'048'576;  // 1 MB
    }

    ChunkSize chunk_size;
};

TEST_F(TestNetworkAnalyticalCongestionUnaware, Ring) {
    // create network
    const auto network_parser = NetworkParser("../../input/Ring.yml");
    const auto topology = construct_topology(network_parser);

    // run communication
    const auto comm_delay = topology->send(1, 4, chunk_size);
    EXPECT_EQ(comm_delay, 21'031);
}

TEST_F(TestNetworkAnalyticalCongestionUnaware, FullyConnected) {
    // create network
    const auto network_parser = NetworkParser("../../input/FullyConnected.yml");
    const auto topology = construct_topology(network_parser);

    // run communication
    const auto comm_delay = topology->send(1, 4, chunk_size);
    EXPECT_EQ(comm_delay, 20'031);
}

TEST_F(TestNetworkAnalyticalCongestionUnaware, Switch) {
    // create network
    const auto network_parser = NetworkParser("../../input/Switch.yml");
    const auto topology = construct_topology(network_parser);

    // run communication
    const auto comm_delay = topology->send(1, 4, chunk_size);
    EXPECT_EQ(comm_delay, 20'531);
}

TEST_F(TestNetworkAnalyticalCongestionUnaware, Ring_FullyConnected_Switch) {
    // create network
    const auto network_parser = NetworkParser("../../input/Ring_FullyConnected_Switch.yml");
    const auto topology = construct_topology(network_parser);

    // run on dim 1
    const auto comm_delay_dim1 = topology->send(0, 1, chunk_size);
    EXPECT_EQ(comm_delay_dim1, 4'932);

    // run on dim 2
    const auto comm_delay_dim2 = topology->send(37, 41, chunk_size);
    EXPECT_EQ(comm_delay_dim2, 10'265);

    // run on dim 3
    const auto comm_delay_dim3 = topology->send(26, 42, chunk_size);
    EXPECT_EQ(comm_delay_dim3, 23'531);
}

TEST_F(TestNetworkAnalyticalCongestionUnaware, ExpanderGraph) {
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
            const auto distance = graph->get_distance(i, j, std::set<DeviceId>(), 0);
            EXPECT_LE(distance, network_parser.get_npus_counts_per_dim()[0]/2);

            total_distance += distance;
            count++;

            //validate communicatio delay
            const auto comm_delay = graph->send(i, j, 1);
            const auto expected_delay = distance * network_parser.get_latencies_per_dim()[0];
            EXPECT_EQ(comm_delay, expected_delay);
        }
    }

    // average distance
    const double average_distance = static_cast<double>(total_distance) / static_cast<double>(count);
    std::cout << "Average distance in ExpanderGraph: " << average_distance << std::endl;
    EXPECT_LE(average_distance, network_parser.get_npus_counts_per_dim()[0]/4.0);
}