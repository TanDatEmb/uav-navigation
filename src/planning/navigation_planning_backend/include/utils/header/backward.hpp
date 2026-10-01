#pragma once

// The upstream planner includes backward-cpp for optional crash reporting.
// Product builds deliberately keep this header a no-op so the realtime target
// does not pull a second logging/runtime dependency into the process.
