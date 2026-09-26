#include <gtest/gtest.h>

#include "px4_navigation_external_mode/px4_authority_handover.hpp"

namespace px4_navigation_external_mode::handover {

TEST(Px4AuthorityHandover, CallbackSuccessWaitsForVehicleStatus) {
  const AttemptToken token{4U, 2U};
  EXPECT_EQ(assessCallback(token, token, true, true, false, CallbackResult::kSuccess),
            CallbackDisposition::kAwaitVehicleStatus);
}

TEST(Px4AuthorityHandover, DeactivatedCallbackDoesNotImplyHold) {
  const AttemptToken token{4U, 2U};
  EXPECT_EQ(assessCallback(token, token, true, true, false, CallbackResult::kDeactivated),
            CallbackDisposition::kAwaitVehicleStatus);
}

TEST(Px4AuthorityHandover, FailureCallbackWaitsForRetryAndDoesNotConfirmHold) {
  const AttemptToken token{4U, 2U};
  EXPECT_EQ(assessCallback(token, token, true, true, false, CallbackResult::kFailure),
            CallbackDisposition::kAwaitVehicleStatus);
}

TEST(Px4AuthorityHandover, StatusConfirmationWinsOverLateCallback) {
  const AttemptToken token{4U, 2U};
  EXPECT_EQ(assessCallback(token, token, false, false, true, CallbackResult::kSuccess),
            CallbackDisposition::kAlreadyConfirmed);
  EXPECT_FALSE(retryDue(10'000, 0, false, true));
}

TEST(Px4AuthorityHandover, OldAttemptCannotMutateRetryOrNewActivation) {
  const AttemptToken old_attempt{4U, 1U};
  EXPECT_EQ(assessCallback(old_attempt, AttemptToken{4U, 2U}, true, true, false,
                           CallbackResult::kFailure),
            CallbackDisposition::kSuperseded);
  EXPECT_EQ(assessCallback(old_attempt, AttemptToken{5U, 1U}, true, true, false,
                           CallbackResult::kFailure),
            CallbackDisposition::kSuperseded);
}

TEST(Px4AuthorityHandover, RequestTimeoutRetriesAndStopsWhenConfirmed) {
  EXPECT_FALSE(retryDue(249'999'999, 250'000'000, true, false));
  EXPECT_TRUE(retryDue(250'000'000, 250'000'000, true, false));
  EXPECT_FALSE(retryDue(300'000'000, 250'000'000, true, true));
}

}  // namespace px4_navigation_external_mode::handover
