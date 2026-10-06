#pragma once

#include <cstdint>
#include <optional>

#include <Eigen/Core>

// Degeneracy evaluation and EV quality of one scan correction (SYSTEM_DESIGN §3.2, D20, D28).
// Pure function of the per-scan information matrix; no state, no clock, no ROS. config.hpp includes
// this header (LioConfig holds a DegeneracyConfig), so this header must not include config.hpp.
namespace uavnav::lio {

/// Minimum acceptable smallest eigenvalue of the translation / rotation information blocks. Loaded from
/// the keys degeneracy_translation_min_info / degeneracy_rotation_min_info (see config.hpp).
struct DegeneracyConfig {
  double translation_min_info;
  double rotation_min_info;
};

struct DegeneracyReport {
  double translation_min_eigenvalue;  // raw smallest eigenvalue of the translation block (0.0 if unusable input)
  double rotation_min_eigenvalue;     // raw smallest eigenvalue of the rotation block (0.0 if unusable input)
  bool degenerate;                    // either block below its threshold, or the input is unusable
  std::uint8_t quality;               // 0..100
};

/// Evaluates the 6x6 information matrix of fast_lio_core's IkfomCorrectionResult::information:
/// block (0:3, 0:3) is translation, block (3:6, 3:6) rotation. The cross-coupling blocks are ignored by
/// the eigen analysis. Never throws.
///
/// Each diagonal block is symmetrised, (A + Aᵀ)/2, and its smallest eigenvalue taken with
/// Eigen::SelfAdjointEigenSolver<Matrix3d>.
///
///   degenerate = translation_min_eigenvalue < translation_min_info
///             OR rotation_min_eigenvalue   < rotation_min_info
///             OR the input is nullopt, or any of its 36 entries or the eigenvalues is non-finite
///             OR a threshold in `config` is not finite and > 0 (an unusable configuration is never "healthy").
///   quality    = clamp(round(50 * min(λt / translation_min_info, λr / rotation_min_info)), 0, 100),
///                with negative eigenvalues (numerical noise) counted as 0 for the quality only.
///                So 50 exactly at the thresholds, 100 at twice the thresholds or more, 0 with no information.
///                Quality 50 does not imply "not degenerate": a ratio of 0.995 rounds to 50 while `degenerate`
///                is true. Consumers gate on `degenerate`, never on quality alone.
///
/// nullopt means "no usable rows" (a degenerate/unknown case, not "skip"): degenerate = true, quality = 0,
/// both reported eigenvalues 0.0. Non-finite input gives the same result. For finite input the eigenvalues
/// are reported raw, so a slightly negative value shows as is.
DegeneracyReport evaluate_degeneracy(const std::optional<Eigen::Matrix<double, 6, 6>>& information,
                                     const DegeneracyConfig& config) noexcept;

}  // namespace uavnav::lio
