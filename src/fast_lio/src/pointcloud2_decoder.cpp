// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#include "fast_lio/pointcloud2_decoder.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include <rclcpp/rclcpp.hpp>

namespace fast_lio {

namespace {

bool isNumericDatatype(std::uint8_t datatype) {
    switch (datatype) {
        case sensor_msgs::msg::PointField::FLOAT32:
        case sensor_msgs::msg::PointField::FLOAT64:
        case sensor_msgs::msg::PointField::UINT8:
        case sensor_msgs::msg::PointField::UINT16:
        case sensor_msgs::msg::PointField::UINT32:
        case sensor_msgs::msg::PointField::INT8:
        case sensor_msgs::msg::PointField::INT16:
        case sensor_msgs::msg::PointField::INT32:
            return true;
        default:
            return false;
    }
}

std::size_t datatypeSize(std::uint8_t datatype) {
    switch (datatype) {
        case sensor_msgs::msg::PointField::INT8:
        case sensor_msgs::msg::PointField::UINT8:
            return 1;
        case sensor_msgs::msg::PointField::INT16:
        case sensor_msgs::msg::PointField::UINT16:
            return 2;
        case sensor_msgs::msg::PointField::INT32:
        case sensor_msgs::msg::PointField::UINT32:
        case sensor_msgs::msg::PointField::FLOAT32:
            return 4;
        case sensor_msgs::msg::PointField::FLOAT64:
            return 8;
        default:
            return 0;
    }
}

bool isFieldValid(const sensor_msgs::msg::PointCloud2& msg,
                  const sensor_msgs::msg::PointField& field) {
    const std::size_t dt_size = datatypeSize(field.datatype);
    if (dt_size == 0) return false;
    if (field.offset + dt_size > msg.point_step) {
        return false;
    }
    return true;
}

}  // namespace

std::string DecodeResult::errorMessage() const {
    switch (error) {
        case DecodeError::kOk: return "OK";
        case DecodeError::kEmptyCloud: return "Empty point cloud";
        case DecodeError::kMissingXyzField: return "Missing required x/y/z field";
        case DecodeError::kMissingTimeField: return "Required per-point time field not found";
        case DecodeError::kInvalidTimeDatatype: return "Time field has unsupported datatype";
        case DecodeError::kInvalidTimestamp: return "Invalid timestamp (NaN/Inf/negative)";
        case DecodeError::kInvalidScanDuration: return "Scan duration exceeds configured maximum";
        case DecodeError::kAllPointsInvalid: return "All points rejected by filtering";
        case DecodeError::kSchemaMismatch: return "Schema does not match configured profile";
        default: return "Unknown error";
    }
}

PointCloud2Decoder::PointCloud2Decoder(PointCloudDecoderConfig config)
    : config_(std::move(config)) {}

bool PointCloud2Decoder::findField(const sensor_msgs::msg::PointCloud2& msg,
                                   const std::string& name,
                                   sensor_msgs::msg::PointField& out) const {
    for (const auto& field : msg.fields) {
        if (field.name == name) {
            out = field;
            return true;
        }
    }
    return false;
}

double PointCloud2Decoder::readFieldValue(const std::uint8_t* data_ptr,
                                           const sensor_msgs::msg::PointField& field) const {
    const std::uint8_t* p = data_ptr + field.offset;

    switch (field.datatype) {
        case sensor_msgs::msg::PointField::FLOAT32: {
            float val;
            std::memcpy(&val, p, sizeof(float));
            return static_cast<double>(val);
        }
        case sensor_msgs::msg::PointField::FLOAT64: {
            double val;
            std::memcpy(&val, p, sizeof(double));
            return val;
        }
        case sensor_msgs::msg::PointField::UINT8: {
            std::uint8_t val;
            std::memcpy(&val, p, sizeof(std::uint8_t));
            return static_cast<double>(val);
        }
        case sensor_msgs::msg::PointField::UINT16: {
            std::uint16_t val;
            std::memcpy(&val, p, sizeof(std::uint16_t));
            return static_cast<double>(val);
        }
        case sensor_msgs::msg::PointField::UINT32: {
            std::uint32_t val;
            std::memcpy(&val, p, sizeof(std::uint32_t));
            return static_cast<double>(val);
        }
        case sensor_msgs::msg::PointField::INT8: {
            std::int8_t val;
            std::memcpy(&val, p, sizeof(std::int8_t));
            return static_cast<double>(val);
        }
        case sensor_msgs::msg::PointField::INT16: {
            std::int16_t val;
            std::memcpy(&val, p, sizeof(std::int16_t));
            return static_cast<double>(val);
        }
        case sensor_msgs::msg::PointField::INT32: {
            std::int32_t val;
            std::memcpy(&val, p, sizeof(std::int32_t));
            return static_cast<double>(val);
        }
        default:
            return 0.0;
    }
}

void PointCloud2Decoder::computeScanTimeBounds(NormalizedLidarScan& scan,
                                                double header_time,
                                                double max_rel_time) const {
    if (config_.header_stamp_is_scan_start) {
        scan.scan_start_time_s = header_time;
        scan.scan_end_time_s = header_time + max_rel_time;
    } else {
        scan.scan_end_time_s = header_time;
        scan.scan_start_time_s = header_time - max_rel_time;
    }
}

void PointCloud2Decoder::logSchemaOnce(const sensor_msgs::msg::PointCloud2& msg) {
    if (schema_logged_) return;
    schema_logged_ = true;

    std::string fields_str;
    for (size_t i = 0; i < msg.fields.size(); ++i) {
        if (i > 0) fields_str += ",";
        fields_str += msg.fields[i].name;
    }

    RCLCPP_INFO(rclcpp::get_logger("fast_lio.decoder"),
                "LiDAR schema: fields=[%s] point_step=%u row_step=%u width=%u height=%u "
                "frame=%s time_field='%s' time_unit=%d per_point_time=%s",
                fields_str.c_str(), msg.point_step, msg.row_step, msg.width, msg.height,
                msg.header.frame_id.c_str(), config_.time_field.c_str(),
                static_cast<int>(config_.time_unit),
                config_.time_field.empty() ? "false" : "true");
}

DecodeResult PointCloud2Decoder::decode(const sensor_msgs::msg::PointCloud2& msg) {
    ++diagnostics_.received_scans;
    DecodeResult result;

    // Validate basic message structure
    if (msg.width == 0 || msg.height == 0 || msg.data.empty()) {
        result.error = DecodeError::kEmptyCloud;
        ++diagnostics_.rejected_scans;
        return result;
    }

    // Schema inspection: find x, y, z
    sensor_msgs::msg::PointField fx, fy, fz;
    const bool has_x = findField(msg, "x", fx);
    const bool has_y = findField(msg, "y", fy);
    const bool has_z = findField(msg, "z", fz);

    if (!has_x || !has_y || !has_z ||
        !isFieldValid(msg, fx) || !isFieldValid(msg, fy) || !isFieldValid(msg, fz)) {
        result.error = DecodeError::kMissingXyzField;
        ++diagnostics_.rejected_scans;
        return result;
    }

    // Find optional intensity field
    sensor_msgs::msg::PointField f_intensity;
    const bool has_intensity = !config_.intensity_field.empty() &&
                                findField(msg, config_.intensity_field, f_intensity) &&
                                isFieldValid(msg, f_intensity);

    // Find per-point time field
    sensor_msgs::msg::PointField f_time;
    bool has_time = false;
    if (!config_.time_field.empty()) {
        has_time = findField(msg, config_.time_field, f_time) && isFieldValid(msg, f_time);
        if (config_.require_per_point_time && !has_time) {
            result.error = DecodeError::kMissingTimeField;
            ++diagnostics_.rejected_scans;
            return result;
        }
        if (has_time && !isNumericDatatype(f_time.datatype)) {
            result.error = DecodeError::kInvalidTimeDatatype;
            ++diagnostics_.rejected_scans;
            return result;
        }
    }

    // Find optional tag field
    sensor_msgs::msg::PointField f_tag;
    const bool has_tag = !config_.tag_field.empty() &&
                          findField(msg, config_.tag_field, f_tag) &&
                          isFieldValid(msg, f_tag);

    // Find optional line field
    [[maybe_unused]] sensor_msgs::msg::PointField f_line;
    [[maybe_unused]] const bool has_line = !config_.line_field.empty() &&
                           findField(msg, config_.line_field, f_line) &&
                           isFieldValid(msg, f_line);

    // Frame validation (warn only)
    if (!config_.lidar_frame.empty() && msg.header.frame_id != config_.lidar_frame) {
        RCLCPP_WARN(rclcpp::get_logger("fast_lio.decoder"),
                     "LiDAR frame mismatch: expected '%s', got '%s'",
                     config_.lidar_frame.c_str(), msg.header.frame_id.c_str());
    }

    // Header timestamp
    const double header_time =
        static_cast<double>(msg.header.stamp.sec) +
        static_cast<double>(msg.header.stamp.nanosec) * 1e-9;

    if (!std::isfinite(header_time) || header_time <= 0.0) {
        result.error = DecodeError::kInvalidTimestamp;
        ++diagnostics_.rejected_scans;
        return result;
    }

    logSchemaOnce(msg);

    // Decode points
    NormalizedLidarScan& scan = result.scan;
    scan.lidar_frame = msg.header.frame_id;
    scan.has_per_point_time = has_time;
    scan.cloud->clear();
    scan.cloud->is_dense = false;
    scan.cloud->width = 0;
    scan.cloud->height = 1;

    const std::size_t total_points =
        static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height);
    scan.cloud->points.reserve(total_points / std::max(1, config_.point_stride));

    double min_rel_time = std::numeric_limits<double>::max();
    double max_rel_time = std::numeric_limits<double>::lowest();
    std::uint64_t valid_count = 0;

    for (std::size_t idx = 0; idx < total_points; ++idx) {
        const std::uint8_t* point_data = msg.data.data() + idx * msg.point_step;

        const double x = readFieldValue(point_data, fx);
        const double y = readFieldValue(point_data, fy);
        const double z = readFieldValue(point_data, fz);

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            ++diagnostics_.nonfinite_points;
            continue;
        }

        const double range_sq = x * x + y * y + z * z;
        const double range = std::sqrt(range_sq);

        if (range < config_.min_range_m) {
            ++diagnostics_.near_range_rejected;
            continue;
        }
        if (range > config_.max_range_m) {
            ++diagnostics_.far_range_rejected;
            continue;
        }

        // Tag filtering (Livox)
        if (config_.filter_livox_tags && has_tag) {
            const std::uint8_t tag =
                static_cast<std::uint8_t>(readFieldValue(point_data, f_tag));
            if ((tag & 0x30) != 0x10 && (tag & 0x30) != 0x00) {
                ++diagnostics_.invalid_tag_rejected;
                continue;
            }
        }

        // Stride
        if (config_.point_stride > 1 && (valid_count % config_.point_stride) != 0) {
            continue;
        }

        float intensity = 0.0f;
        if (has_intensity) {
            intensity = static_cast<float>(readFieldValue(point_data, f_intensity));
        }

        float rel_time_s = 0.0f;
        if (has_time) {
            const double raw_time = readFieldValue(point_data, f_time);
            rel_time_s = static_cast<float>(timeToSeconds(raw_time, config_.time_unit));
            if (!std::isfinite(rel_time_s) || rel_time_s < -1e-6) {
                ++diagnostics_.nonfinite_points;
                continue;
            }
            if (rel_time_s < min_rel_time) min_rel_time = rel_time_s;
            if (rel_time_s > max_rel_time) max_rel_time = rel_time_s;
        }

        PointType pt;
        pt.x = static_cast<float>(x);
        pt.y = static_cast<float>(y);
        pt.z = static_cast<float>(z);
        pt.intensity = intensity;
        pt.normal_x = 0.0f;
        pt.normal_y = 0.0f;
        pt.normal_z = 0.0f;
        pt.curvature = rel_time_s;
        scan.cloud->points.push_back(pt);
        ++valid_count;
    }

    diagnostics_.input_points += total_points;
    diagnostics_.valid_points += valid_count;

    if (scan.cloud->points.empty()) {
        result.error = DecodeError::kAllPointsInvalid;
        ++diagnostics_.rejected_scans;
        return result;
    }

    scan.cloud->width = static_cast<std::uint32_t>(scan.cloud->points.size());
    scan.cloud->height = 1;
    scan.cloud->is_dense = true;

    // Compute scan time bounds
    if (has_time) {
        // Normalize: subtract min_rel_time so all relative times start at 0
        if (min_rel_time > 0.0 && std::isfinite(min_rel_time)) {
            for (auto& pt : scan.cloud->points) {
                pt.curvature -= static_cast<float>(min_rel_time);
            }
            max_rel_time -= min_rel_time;
            min_rel_time = 0.0;
        }

        const double scan_duration = max_rel_time - min_rel_time;
        if (scan_duration > config_.max_scan_duration_s) {
            result.error = DecodeError::kInvalidScanDuration;
            ++diagnostics_.rejected_scans;
            RCLCPP_WARN(rclcpp::get_logger("fast_lio.decoder"),
                        "Scan duration %.4f s exceeds max %.4f s — rejecting",
                        scan_duration, config_.max_scan_duration_s);
            return result;
        }

        // Sort by relative time (ascending)
        std::sort(scan.cloud->points.begin(), scan.cloud->points.end(),
                  [](const PointType& a, const PointType& b) {
                      return a.curvature < b.curvature;
                  });

        computeScanTimeBounds(scan, header_time, max_rel_time);

        diagnostics_.last_point_time_min_s = min_rel_time;
        diagnostics_.last_point_time_max_s = max_rel_time;
        diagnostics_.last_scan_duration_s = scan_duration;
    } else {
        scan.has_per_point_time = false;
        scan.scan_start_time_s = header_time;
        scan.scan_end_time_s = header_time;

        diagnostics_.last_point_time_min_s = 0.0;
        diagnostics_.last_point_time_max_s = 0.0;
        diagnostics_.last_scan_duration_s = 0.0;
    }

    ++diagnostics_.accepted_scans;
    return result;
}

}  // namespace fast_lio
