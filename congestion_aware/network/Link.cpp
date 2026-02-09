/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "congestion_aware/Link.h"
#include "common/NetworkFunction.h"
#include "congestion_aware/Chunk.h"
#include "congestion_aware/Device.h"
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <map>
#include <vector>
#include <algorithm>

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionAware;

// declaring static event_queue
std::shared_ptr<EventQueue> Link::event_queue;
bool Link::random_queue_enabled = false;

// debug congestion tracking
static int congestion_log_count = 0;
static int total_queued = 0;
static std::map<std::pair<int,int>, int> link_queue_counts;  // (src,dst) -> times queued

void Link::link_become_free(void* const link_ptr) noexcept {
    assert(link_ptr != nullptr);

    // cast to Link*
    auto* const link = static_cast<Link*>(link_ptr);

    // set link free
    link->set_free();

    // process pending chunks if one exist
    if (link->pending_chunk_exists()) {
        link->process_pending_transmission();
    }

    // Periodic congestion summary
    static int free_count = 0;
    free_count++;
    if (free_count % 500000 == 0 && total_queued > 0) {
        std::cout << "[LINK_CONGESTION_SUMMARY] total_queued=" << total_queued << std::endl;
        // Sort by count descending
        std::vector<std::pair<std::pair<int,int>, int>> sorted_links(link_queue_counts.begin(), link_queue_counts.end());
        std::sort(sorted_links.begin(), sorted_links.end(), [](const auto& a, const auto& b) { return a.second > b.second; });
        int shown = 0;
        for (const auto& [link_pair, count] : sorted_links) {
            std::cout << "  link " << link_pair.first << "->" << link_pair.second << ": queued " << count << " times" << std::endl;
            if (++shown >= 20) break;
        }
    }
}

void Link::set_event_queue(std::shared_ptr<EventQueue> event_queue_ptr) noexcept {
    assert(event_queue_ptr != nullptr);

    // set the event queue
    Link::event_queue = std::move(event_queue_ptr);
}

void Link::set_random_queue(bool enabled) noexcept {
    random_queue_enabled = enabled;
    if (enabled) {
        std::cout << "[CONFIG] Link random queue: ENABLED (shuffled packet ordering)" << std::endl;
    }
}

Link::Link(const Bandwidth bandwidth, const Latency latency) noexcept
    : bandwidth(bandwidth),
      latency(latency),
      pending_chunks(),
      busy(false) {
    assert(bandwidth > 0);
    assert(latency >= 0);

    // convert bandwidth from GB/s to B/ns
    bandwidth_Bpns = bw_GBps_to_Bpns(bandwidth);
}

void Link::send(std::unique_ptr<Chunk> chunk) noexcept {
    assert(chunk != nullptr);

    if (busy) {
        // link is busy, add to pending chunks
        total_queued++;
        int src_id = chunk->current_device()->get_id();
        int dst_id = chunk->next_device()->get_id();
        link_queue_counts[{src_id, dst_id}]++;
        if (congestion_log_count < 50) {
            auto current_time = Link::event_queue->get_current_time();
            std::cout << "[LINK_QUEUE] t=" << current_time 
                      << " link " << src_id << "->" << dst_id
                      << " BUSY, chunk_size=" << chunk->get_size()
                      << " pending=" << pending_chunks.size()
                      << std::endl;
            congestion_log_count++;
        }
        pending_chunks.push_back(std::move(chunk));
    } else {
        // service this chunk immediately
        schedule_chunk_transmission(std::move(chunk));
    }
}

void Link::process_pending_transmission() noexcept {
    // pending chunk should exist
    assert(pending_chunk_exists());

    std::unique_ptr<Chunk> chunk;
    if (random_queue_enabled && pending_chunks.size() > 1) {
        // Pick random chunk from queue (shuffled ordering)
        int idx = rand() % pending_chunks.size();
        auto it = pending_chunks.begin();
        std::advance(it, idx);
        chunk = std::move(*it);
        pending_chunks.erase(it);
    } else {
        // FIFO ordering (original behavior)
        chunk = std::move(pending_chunks.front());
        pending_chunks.pop_front();
    }

    // service this chunk
    schedule_chunk_transmission(std::move(chunk));
}

bool Link::pending_chunk_exists() const noexcept {
    // check pending chunks is not empty
    return !pending_chunks.empty();
}

void Link::set_busy() noexcept {
    // set busy to true
    busy = true;
}

void Link::set_free() noexcept {
    // set busy to false
    busy = false;
}

EventTime Link::serialization_delay(const ChunkSize chunk_size) const noexcept {
    assert(chunk_size > 0);

    // calculate serialization delay
    const auto delay = static_cast<Bandwidth>(chunk_size) / bandwidth_Bpns;

    // return serialization delay in EventTime type
    return static_cast<EventTime>(delay);
}

EventTime Link::communication_delay(const ChunkSize chunk_size) const noexcept {
    assert(chunk_size > 0);

    // calculate communication delay
    const auto delay = latency + (static_cast<Bandwidth>(chunk_size) / bandwidth_Bpns);

    // return communication delay in EventTime type
    return static_cast<EventTime>(delay);
}

void Link::schedule_chunk_transmission(std::unique_ptr<Chunk> chunk) noexcept {
    assert(chunk != nullptr);

    // link should be free
    assert(!busy);

    // set link busy
    set_busy();

    // get metadata
    const auto chunk_size = chunk->get_size();
    const auto current_time = Link::event_queue->get_current_time();

    // schedule chunk arrival event
    const auto communication_time = communication_delay(chunk_size);
    const auto chunk_arrival_time = current_time + communication_time;
    auto* const chunk_ptr = static_cast<void*>(chunk.release());
    Link::event_queue->schedule_event(chunk_arrival_time, Chunk::chunk_arrived_next_device, chunk_ptr);

    // schedule link free time
    const auto serialization_time = serialization_delay(chunk_size);
    const auto link_free_time = current_time + serialization_time;
    auto* const link_ptr = static_cast<void*>(this);
    Link::event_queue->schedule_event(link_free_time, link_become_free, link_ptr);
}
