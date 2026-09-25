# Unsafe fresh-world no-resume proof

Deterministic owner test: `TestExecutionAuthority.UnsafeFreshWorldRecertificationCannotResumeSuspendedExecution`, in `src/execution/navigation_execution/test/test_execution_authority.cpp`. The test starts with a suspended current execution and supplies a fresh immutable world whose exact validation rejects the retained bundle. The owner clears/fails closed that exact execution; a later fresh-world update cannot resurrect it.

Classification: COMPONENT, deterministic. Result: PASS in the final 75-case `test_execution_authority` binary. This proves the safety decision without a contrived Gazebo obstacle arrangement. It is not an unsafe-world SITL/HITL claim.
