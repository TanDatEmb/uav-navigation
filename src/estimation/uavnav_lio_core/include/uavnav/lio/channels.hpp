#pragma once

#include "uavnav/core/spsc_queue.hpp"
#include "uavnav/lio/limits.hpp"
#include "uavnav/lio/types.hpp"

// The two bounded queues between the LIO frontend (ingest thread) and backend (estimator thread),
// SYSTEM_DESIGN §3.5, D30/O14. The frontend only pushes `requests` and pops `results`; the backend does the
// opposite (one producer and one consumer per queue). A full push is refused, never blocks, and the
// pushing side emits a LioQueueOverflow event with its reason.
//
// Ownership and size: each SpscQueue holds its slots inline, so LioChannels is about
// kBackendRequestCapacity * sizeof(BackendRequest) + kBackendResultCapacity * sizeof(BackendResult) bytes
// (tens of KiB). Create it on the HEAP (std::make_unique<LioChannels>()), never on a thread stack, and owned
// by the object that owns both blocks. Destroy it last: join the backend thread, then destroy the backend,
// then the frontend, then the channels.
namespace uavnav::lio {

using RequestQueue = concurrency::SpscQueue<BackendRequest, limits::kBackendRequestCapacity>;
using ResultQueue = concurrency::SpscQueue<BackendResult, limits::kBackendResultCapacity>;

struct LioChannels {
  RequestQueue requests;
  ResultQueue results;
};

}  // namespace uavnav::lio
