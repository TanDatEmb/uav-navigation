#pragma once

namespace navigation_planning {

struct PlanningTimingContract final {
  static constexpr double kPlannerPeriodS = 0.20;
  static constexpr double kSolveDeadlineS = 0.18;
  static constexpr double kStitchDurationS = 0.40;
  static constexpr double kCommitGuardS = 0.02;
  static constexpr double kMinimumMainReserveS =
      kSolveDeadlineS + kStitchDurationS + kPlannerPeriodS + kCommitGuardS;
  static constexpr double kUrgentBaselineThresholdS = 1.0;
  static constexpr double kCommandPeriodS = 0.02;
  static constexpr double kSnapshotPeriodS = 0.10;
  static constexpr double kLocalWindowM = 20.0;
  static constexpr double kStoppedRecoveryTimeoutS = 5.0;
  static constexpr double kStationarySpeedMps = 0.15;
};

static_assert(PlanningTimingContract::kMinimumMainReserveS == 0.80);

}  // namespace navigation_planning
