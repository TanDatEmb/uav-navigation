# Coverage gaps — WP-A4-R1

`covered_by_test = NONE` remains a static evidence gap. Ordering is by effect safety impact; source coverage is not runtime/PX4 qualification.

## P0: FAIL_CLOSED / REQUEST_PX4_HOLD / EMERGENCY_BRAKE_*

- 18 rules: `RT-200, RT-201, RT-204, RT-220, RT-222, RT-223, RT-224, RT-225, RT-229, RT-231, RT-233, RT-236, RT-240, RT-242, RT-259, RT-266, RT-267, RT-271`
- Evidence: `NONE` for the complete NavigationRuntimeNode event path; predicate-only rows are not integration or flight evidence.

## P1: containment / cancellation / latch

- 44 rules: `RT-197, RT-198, RT-199, RT-202, RT-203, RT-205, RT-206, RT-207, RT-208, RT-209, RT-211, RT-213, RT-214, RT-215, RT-216, RT-226, RT-228, RT-230, RT-232, RT-234, RT-235, RT-238, RT-239, RT-241, RT-244, RT-245, RT-246, RT-247, RT-248, RT-249, RT-250, RT-251, RT-252, RT-253, RT-254, RT-255, RT-256, RT-257, RT-258, RT-260, RT-262, RT-263, RT-264, RT-270`
- Evidence: `NONE` for the complete NavigationRuntimeNode event path; predicate-only rows are not integration or flight evidence.

## P1: authority / recertification

- 3 rules: `RT-217, RT-221, RT-274`
- Evidence: `NONE` for the complete NavigationRuntimeNode event path; predicate-only rows are not integration or flight evidence.

## P2: publication / mission progress

- 4 rules: `RT-237, RT-265, RT-268, RT-269`
- Evidence: `NONE` for the complete NavigationRuntimeNode event path; predicate-only rows are not integration or flight evidence.

## P3: no-op / diagnostics / predicate-only

- 259 rules: `RT-001, RT-002, RT-003, RT-004, RT-005, RT-006, RT-007, RT-008, RT-009, RT-010, RT-011, RT-012, RT-013, RT-014, RT-015, RT-016, RT-017, RT-018, RT-019, RT-020, RT-021, RT-022, RT-023, RT-024, RT-025, RT-026, RT-027, RT-028, RT-029, RT-030, RT-031, RT-032, RT-033, RT-034, RT-035, RT-036, RT-037, RT-038, RT-039, RT-040, RT-041, RT-042, RT-043, RT-044, RT-045, RT-046, RT-047, RT-048, RT-049, RT-050, RT-051, RT-052, RT-053, RT-054, RT-055, RT-056, RT-057, RT-058, RT-059, RT-060, RT-061, RT-062, RT-063, RT-064, RT-065, RT-066, RT-067, RT-068, RT-069, RT-070, RT-071, RT-072, RT-073, RT-074, RT-075, RT-076, RT-077, RT-078, RT-079, RT-080, RT-081, RT-082, RT-083, RT-084, RT-085, RT-086, RT-087, RT-088, RT-089, RT-090, RT-091, RT-092, RT-093, RT-094, RT-095, RT-096, RT-097, RT-098, RT-099, RT-100, RT-101, RT-102, RT-103, RT-104, RT-105, RT-106, RT-107, RT-108, RT-109, RT-110, RT-111, RT-112, RT-113, RT-114, RT-115, RT-116, RT-117, RT-118, RT-119, RT-120, RT-121, RT-122, RT-123, RT-124, RT-125, RT-126, RT-127, RT-128, RT-129, RT-130, RT-131, RT-132, RT-133, RT-134, RT-135, RT-136, RT-137, RT-138, RT-139, RT-140, RT-141, RT-142, RT-143, RT-144, RT-145, RT-146, RT-147, RT-148, RT-149, RT-150, RT-151, RT-152, RT-153, RT-154, RT-155, RT-156, RT-157, RT-158, RT-159, RT-160, RT-161, RT-162, RT-163, RT-164, RT-165, RT-166, RT-167, RT-168, RT-169, RT-170, RT-171, RT-172, RT-173, RT-174, RT-175, RT-176, RT-177, RT-178, RT-179, RT-180, RT-181, RT-182, RT-183, RT-184, RT-185, RT-186, RT-187, RT-188, RT-189, RT-190, RT-191, RT-192, RT-193, RT-194, RT-195, RT-196, RT-210, RT-212, RT-218, RT-219, RT-227, RT-243, RT-261, RT-272, RT-273, RT-275, RT-276, RT-277, RT-278, RT-279, RT-280, RT-281, RT-282, RT-283, RT-284, RT-285, RT-286, RT-287, RT-288, RT-289, RT-290, RT-291, RT-292, RT-293, RT-294, RT-295, RT-296, RT-297, RT-298, RT-299, RT-300, RT-301, RT-302, RT-303, RT-304, RT-305, RT-306, RT-307, RT-308, RT-309, RT-310, RT-311, RT-312, RT-313, RT-314, RT-315, RT-316, RT-317, RT-318, RT-319, RT-320, RT-321, RT-322, RT-323, RT-324, RT-325, RT-326, RT-327, RT-328`
- Evidence: `NONE` for the complete NavigationRuntimeNode event path; predicate-only rows are not integration or flight evidence.

## Adversarial review

- Checker counts source `return` and side-effect call sites from baseline `7e0b850`; source drift must fail coverage.
- Fault-injection rules are diagnostic-only and separate from normal planner-result rules.
- SITL, PX4, rosbag replay, latency distributions, and qualification are `NOT_MEASURED`.
