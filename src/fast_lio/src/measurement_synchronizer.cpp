// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#include "fast_lio/measurement_synchronizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "fast_lio/utils.hpp"

namespace fast_lio {

MeasurementSynchronizer::MeasurementSynchronizer(SynchronizerConfig config)
    : config_(std::move(config)) {}

void MeasurementSynchronizer::pushImu(const IMUData& sample) {
    std::lock_guard<std::mutex> lock(mutex_);

    ++diagnostics_.imu_received;

    // Validate finite
    if (!sample.acc.allFinite() || !sample.gyro.allFinite() ||
        !std::isfinite(sample.time)) {
        return;
    }

    // Large regression — clock reset (check BEFORE duplicate, since a large
    // backwards jump looks like a duplicate to the epsilon check)
    if (sample.time < last_imu_time_ - 1.0 && last_imu_time_ > -1e17) {
        // Significant backwards jump: reset synchronizer
        imu_buffer_.clear();
        lidar_buffer_.clear();
        lidar_pushed_ = false;
        retained_boundary_imu_.reset();
        last_scan_end_time_ = -1e18;
        ++diagnostics_.imu_out_of_order;
    }

    // Timestamp check — duplicate or small regression (after reset check)
    if (sample.time <= last_imu_time_ + config_.timestamp_epsilon_s &&
        last_imu_time_ > -1e17) {
        // Duplicate or small regression — drop sample, don't clear buffer
        ++diagnostics_.imu_out_of_order;
        return;
    }

    imu_buffer_.push_back(sample);
    last_imu_time_ = sample.time;

    trimImuBuffer();
}

void MeasurementSynchronizer::pushLidar(const NormalizedLidarScan& scan) {
    std::lock_guard<std::mutex> lock(mutex_);

    ++diagnostics_.lidar_received;

    // Validate scan contract
    if (scan.cloud->empty()) {
        return;
    }

    // Large regression — check before duplicate
    if (scan.scan_start_time_s < last_lidar_start_time_ - 1.0 &&
        last_lidar_start_time_ > -1e17) {
        imu_buffer_.clear();
        lidar_buffer_.clear();
        lidar_pushed_ = false;
        retained_boundary_imu_.reset();
        last_scan_end_time_ = -1e18;
        ++diagnostics_.lidar_out_of_order;
    }

    // Duplicate or small regression (after reset check)
    if (scan.scan_start_time_s <= last_lidar_start_time_ + config_.timestamp_epsilon_s &&
        last_lidar_start_time_ > -1e17) {
        ++diagnostics_.lidar_out_of_order;
        return;
    }

    lidar_buffer_.push_back(scan);
    last_lidar_start_time_ = scan.scan_start_time_s;

    trimLidarBuffer();
}

void MeasurementSynchronizer::trimImuBuffer() {
    if (imu_buffer_.size() > config_.max_imu_samples) {
        const std::size_t excess = imu_buffer_.size() - config_.max_imu_samples;
        // Erase from front, but preserve boundary if we have a pending scan
        imu_buffer_.erase(imu_buffer_.begin(),
                          imu_buffer_.begin() + static_cast<std::ptrdiff_t>(excess));
    }
}

void MeasurementSynchronizer::trimLidarBuffer() {
    if (lidar_buffer_.size() > config_.max_lidar_scans) {
        const std::size_t excess = lidar_buffer_.size() - config_.max_lidar_scans;
        lidar_buffer_.erase(lidar_buffer_.begin(),
                            lidar_buffer_.begin() + static_cast<std::ptrdiff_t>(excess));
        // If we dropped the cached front, reset cache
        if (excess > 0) {
            lidar_pushed_ = false;
        }
    }
}

SyncResult MeasurementSynchronizer::tryPop() {
    std::lock_guard<std::mutex> lock(mutex_);
    SyncResult result;

    diagnostics_.imu_buffer_size = imu_buffer_.size();
    diagnostics_.lidar_buffer_size = lidar_buffer_.size();

    if (lidar_buffer_.empty()) {
        result.status = SyncStatus::kWaitingForLidar;
        return result;
    }

    // Cache front scan info on first access
    if (!lidar_pushed_) {
        const NormalizedLidarScan& scan = lidar_buffer_.front();
        cached_start_time_ = scan.scan_start_time_s;
        cached_end_time_ = scan.scan_end_time_s;
        cached_has_per_point_time_ = scan.has_per_point_time;
        lidar_pushed_ = true;
    }

    // Check IMU has reached scan_end
    if (imu_buffer_.empty() || last_imu_time_ < cached_end_time_) {
        result.status = SyncStatus::kWaitingForImu;
        return result;
    }

    // --- Find IMU samples for this scan ---

    // Find first IMU index at or after scan_start
    auto it_start = imu_buffer_.begin();
    while (it_start != imu_buffer_.end() && it_start->time < cached_start_time_) {
        ++it_start;
    }

    // Check coverage at scan_start
    if (config_.require_imu_before_scan_start && cached_start_time_ < cached_end_time_) {
        // For REAL scans (duration > 0), need IMU before scan_start.
        // Check both buffer and retained boundary from previous package.
        const bool has_buffer_before = (it_start != imu_buffer_.begin());
        const bool has_retained = (retained_boundary_imu_.has_value() &&
                                    retained_boundary_imu_->time <= cached_start_time_);
        if (!has_buffer_before && !has_retained) {
            // No IMU before scan_start
            lidar_buffer_.pop_front();
            lidar_pushed_ = false;
            ++diagnostics_.dropped_no_start_coverage;
            result.status = SyncStatus::kDropNoStartCoverage;
            return result;
        }
    }

    // Bracket before scan_start: prefer retained boundary from previous package
    // (it is the sample at or after the previous scan_end, which is closest to scan_start).
    // Fall back to the IMU sample just before scan_start in the buffer.
    std::optional<IMUData> imu_before;
    if (retained_boundary_imu_.has_value() &&
        retained_boundary_imu_->time <= cached_start_time_) {
        imu_before = retained_boundary_imu_;
    } else if (it_start != imu_buffer_.begin()) {
        imu_before = *(it_start - 1);
    }

    // Find first IMU at or after scan_end
    auto it_end = it_start;
    while (it_end != imu_buffer_.end() && it_end->time < cached_end_time_) {
        ++it_end;
    }

    // Bracket after scan_end
    std::optional<IMUData> imu_after;
    if (it_end != imu_buffer_.end()) {
        imu_after = *it_end;
    }

    // For REAL scans, check we have bracket after scan_end
    if (config_.require_imu_after_scan_end && cached_start_time_ < cached_end_time_ && !imu_after) {
        result.status = SyncStatus::kWaitingForImu;
        return result;
    }

    // --- Collect IMU samples within [scan_start, scan_end] ---
    std::deque<IMUData> package_imus;
    double max_gap = 0.0;
    double prev_time = imu_before.has_value() ? imu_before->time : cached_start_time_;

    for (auto it = it_start; it != it_end; ++it) {
        const double dt = it->time - prev_time;
        if (dt > max_gap) {
            max_gap = dt;
        }
        package_imus.push_back(*it);
        prev_time = it->time;
    }

    // Special handling for SIM snapshot: scan_start == scan_end.
    // The range [start, end) is empty, so the loop above collects nothing.
    // For a snapshot we still need all IMU samples between this scan and the
    // previous one so the IMU processor can integrate the full inter-scan
    // interval. Collect every IMU sample in (last_scan_end_time_, scan_end].
    if (cached_start_time_ == cached_end_time_ && last_scan_end_time_ > -1e17) {
        // Find first IMU strictly after the previous scan end
        auto it_prev = imu_buffer_.begin();
        while (it_prev != imu_buffer_.end() && it_prev->time <= last_scan_end_time_) {
            ++it_prev;
        }

        for (auto it = it_prev; it != it_end; ++it) {
            const double dt = it->time - prev_time;
            if (dt > max_gap) {
                max_gap = dt;
            }
            package_imus.push_back(*it);
            prev_time = it->time;
        }

        // If the snapshot is the very first scan (no previous end known) or the
        // bracket at scan_end is the only available sample, fall back to it.
        if (package_imus.empty() && it_end != imu_buffer_.end()) {
            const double dt = it_end->time - prev_time;
            if (dt > max_gap) {
                max_gap = dt;
            }
            package_imus.push_back(*it_end);
            prev_time = it_end->time;
        }
    }

    // Update last_scan_end_time_ for the next package.
    last_scan_end_time_ = cached_end_time_;

    // Check gap after last IMU to scan_end
    if (imu_after.has_value()) {
        const double dt_end = imu_after->time - prev_time;
        if (dt_end > max_gap) {
            max_gap = dt_end;
        }
    }

    // Gap validation (only for REAL scans with duration)
    if (cached_start_time_ < cached_end_time_ && max_gap > config_.max_imu_gap_s) {
        // Drop scan — IMU gap too large for reliable deskew
        lidar_buffer_.pop_front();
        lidar_pushed_ = false;
        ++diagnostics_.dropped_imu_gap;
        diagnostics_.last_max_imu_gap_s = max_gap;
        result.status = SyncStatus::kDropImuGap;
        return result;
    }

    // --- Build package ---
    SyncPackage& pkg = result.package;
    const NormalizedLidarScan& scan = lidar_buffer_.front();
    pkg.cloud = scan.cloud;
    pkg.cloud_start_time = cached_start_time_;
    pkg.cloud_end_time = cached_end_time_;
    pkg.has_per_point_time = cached_has_per_point_time_;
    pkg.imus = std::move(package_imus);
    pkg.imu_before_scan = imu_before;
    pkg.imu_after_scan = imu_after;

    // Retain boundary sample for next package:
    // Use the last IMU sample at or before scan_end (from package.imus.back() or imu_after).
    // This ensures the next package has a bracket at or before its scan_start.
    if (!pkg.imus.empty()) {
        retained_boundary_imu_ = pkg.imus.back();
    } else if (imu_after.has_value()) {
        retained_boundary_imu_ = imu_after;
    }

    // --- Remove consumed IMU samples ---
    // Erase IMU samples strictly before scan_end. Keep imu_after (it_end) in buffer
    // so it is available for the next package's coverage check.
    if (it_end != imu_buffer_.begin()) {
        imu_buffer_.erase(imu_buffer_.begin(), it_end);
    }

    // Pop the consumed LiDAR scan
    lidar_buffer_.pop_front();
    lidar_pushed_ = false;

    // Update diagnostics
    ++diagnostics_.packages_emitted;
    diagnostics_.last_scan_duration_s = cached_end_time_ - cached_start_time_;
    diagnostics_.last_max_imu_gap_s = max_gap;

    result.status = SyncStatus::kReady;
    return result;
}

MeasurementSynchronizer::Diagnostics MeasurementSynchronizer::diagnostics() const {
    return diagnostics_;
}

}  // namespace fast_lio