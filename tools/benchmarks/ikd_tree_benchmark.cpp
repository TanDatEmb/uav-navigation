#include <ikd_Tree.h>

#include <chrono>
#include <algorithm>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <stdexcept>
#include <vector>

#include <unistd.h>

namespace {

using Clock = std::chrono::steady_clock;
using PointVector = KD_TREE<ikdTree_PointType>::PointVector;

std::vector<int> parseSizes(const std::string& text) {
  std::vector<int> sizes;
  std::stringstream stream(text);
  std::string field;
  while (std::getline(stream, field, ',')) {
    const int size = std::stoi(field);
    if (size <= 0) {
      throw std::invalid_argument("sizes must be positive");
    }
    sizes.push_back(size);
  }
  if (sizes.size() < 3U) {
    throw std::invalid_argument("at least three sizes are needed for a scaling report");
  }
  return sizes;
}

PointVector points(int count, std::mt19937& random) {
  std::uniform_real_distribution<float> distribution{-100.0F, 100.0F};
  PointVector output;
  output.reserve(static_cast<std::size_t>(count));
  for (int index = 0; index < count; ++index) {
    output.emplace_back(distribution(random), distribution(random), distribution(random));
  }
  return output;
}

template <typename Callable>
double microseconds(Callable&& callable) {
  const auto start = Clock::now();
  callable();
  return std::chrono::duration<double, std::micro>(Clock::now() - start).count();
}

template <typename Callable>
void silenceUpstreamStdout(Callable&& callable) {
  // ikd-Tree writes lifecycle messages directly through printf. Keep the
  // machine-readable benchmark stream clean without modifying vendored source.
  const int saved_stdout = dup(STDOUT_FILENO);
  const int null_stdout = open("/dev/null", O_WRONLY);
  if (saved_stdout < 0 || null_stdout < 0) {
    throw std::runtime_error("cannot redirect upstream benchmark output");
  }
  std::fflush(stdout);
  dup2(null_stdout, STDOUT_FILENO);
  close(null_stdout);
  try {
    callable();
  } catch (...) {
    std::fflush(stdout);
    dup2(saved_stdout, STDOUT_FILENO);
    close(saved_stdout);
    throw;
  }
  std::fflush(stdout);
  dup2(saved_stdout, STDOUT_FILENO);
  close(saved_stdout);
}

}  // namespace

int main(int argc, char** argv) {
  std::string sizes_text{"10000,20000,40000"};
  int query_count = 5000;
  for (int index = 1; index < argc; ++index) {
    const std::string argument{argv[index]};
    if (argument == "--sizes" && index + 1 < argc) {
      sizes_text = argv[++index];
    } else if (argument == "--queries" && index + 1 < argc) {
      query_count = std::stoi(argv[++index]);
    } else {
      std::cerr << "usage: ikd_tree_benchmark [--sizes n,n,n] [--queries n]\n";
      return 64;
    }
  }
  if (query_count <= 0) {
    std::cerr << "queries must be positive\n";
    return 64;
  }

  try {
    const std::vector<int> sizes = parseSizes(sizes_text);
    std::mt19937 random{0x4d315f4bU};
    std::cout << "{\n  \"schema_version\": 1,\n  \"backend\": \"upstream_ikd_tree\",\n"
              << "  \"seed\": 1295073099,\n  \"query_count\": " << query_count
              << ",\n  \"measurements\": [\n";
    for (std::size_t index = 0; index < sizes.size(); ++index) {
      PointVector initial = points(sizes[index], random);
      double build_us{};
      double query_us{};
      double insert_us{};
      double delete_us{};
      int valid_points_after_delete{};
      // KD_TREE owns a large queue and an asynchronous rebuild thread. Keep it
      // on the heap and suppress upstream lifecycle printf output for its full
      // lifetime so JSON remains machine-readable.
      silenceUpstreamStdout([&] {
        auto tree = std::make_unique<KD_TREE<ikdTree_PointType>>();
        build_us = microseconds([&] { tree->Build(initial); });
        PointVector query_points = points(query_count, random);
        query_us = microseconds([&] {
          for (const auto& query : query_points) {
            PointVector neighbors;
            std::vector<float> distances;
            tree->Nearest_Search(query, 5, neighbors, distances, 10.0);
          }
        });
        PointVector additions = points(std::max(1, sizes[index] / 20), random);
        insert_us = microseconds([&] { tree->Add_Points(additions, true); });
        BoxPointType box{{-10.0F, -10.0F, -10.0F}, {10.0F, 10.0F, 10.0F}};
        std::vector<BoxPointType> boxes{box};
        delete_us = microseconds([&] { tree->Delete_Point_Boxes(boxes); });
        valid_points_after_delete = tree->validnum();
        tree.reset();
      });
      std::cout << "    {\"point_count\": " << sizes[index] << ", \"build_us\": "
                << std::fixed << std::setprecision(3) << build_us << ", \"query_us\": "
                << query_us << ", \"insert_us\": " << insert_us << ", \"delete_us\": "
                << delete_us << ", \"valid_points_after_delete\": " << valid_points_after_delete << "}";
      std::cout << (index + 1U == sizes.size() ? "\n" : ",\n");
    }
    std::cout << "  ]\n}\n";
  } catch (const std::exception& error) {
    std::cerr << "benchmark failed: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
