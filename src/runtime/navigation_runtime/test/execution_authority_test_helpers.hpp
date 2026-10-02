#pragma once

#include <memory>
#include <utility>

#include <navigation_execution/execution_authority.hpp>

namespace navigation_runtime::test {

inline navigation_execution::CommitDecision commitProductForTest(
    navigation_execution::ExecutionAuthority& authority,
    const navigation_execution::CommitToken& token,
    std::shared_ptr<const navigation_contracts::msg::NavigationGoal> goal,
    std::shared_ptr<const navigation_planning::CandidateBundle> candidate) {
  const auto predecessor = authority.snapshot();
  return authority.tryCommitIfCurrent(
      token, predecessor, std::move(goal), std::move(candidate), [] { return true; });
}

}  // namespace navigation_runtime::test
