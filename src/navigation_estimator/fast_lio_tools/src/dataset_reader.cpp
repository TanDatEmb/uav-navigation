#include "fast_lio_tools/dataset_reader.hpp"

#include <charconv>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace uav::nav::lio {
namespace {

std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

std::int64_t integer(const std::string& value) {
  std::int64_t result{};
  const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), result);
  if (error != std::errc{} || end != value.data() + value.size()) {
    throw std::runtime_error("invalid integer in offline dataset");
  }
  return result;
}

double real(const std::string& value) {
  std::size_t consumed{};
  const double result = std::stod(value, &consumed);
  if (consumed != value.size()) {
    throw std::runtime_error("invalid real number in offline dataset");
  }
  return result;
}

}  // namespace

DatasetReader::DatasetReader(std::filesystem::path directory) : directory_(std::move(directory)) {}

void DatasetReader::replay(const ImuCallback& on_imu, const LidarCallback& on_lidar) const {
  std::ifstream events(directory_ / "events.csv");
  if (!events) {
    throw std::runtime_error("dataset must contain events.csv");
  }
  std::string line;
  std::getline(events, line);  // exact schema header
  if (line != "type,time_ns,x,y,z,a,b,c,relative_time_ns") {
    throw std::runtime_error("events.csv schema is not supported");
  }
  while (std::getline(events, line)) {
    if (line.empty()) {
      continue;
    }
    const auto fields = split(line);
    if (fields.size() != 9U) {
      throw std::runtime_error("events.csv row has wrong field count");
    }
    const Timestamp stamp{integer(fields[1]), ClockDomain::kSensorTime};
    if (fields[0] == "imu") {
      on_imu(ImuSample{stamp, Eigen::Vector3d{real(fields[2]), real(fields[3]), real(fields[4])},
                       Eigen::Vector3d{real(fields[5]), real(fields[6]), real(fields[7])}});
    } else if (fields[0] == "lidar") {
      LidarScan scan{stamp, stamp, {}, integer(fields[8]) != 0};
      on_lidar(scan);
    } else {
      throw std::runtime_error("unknown event type");
    }
  }
}

}  // namespace uav::nav::lio
