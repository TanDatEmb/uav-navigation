SHELL := /bin/bash
# ROS Jazzy's rclpy/ament modules are installed for the system interpreter.
PYTHON ?= /usr/bin/python3
# Packages built and tested by `make build` / `make test`; override with PKGS="...".
PKGS ?= uavnav_core uavnav_interfaces fast_lio_core uavnav_lio_core uavnav_px4_bridge
# Extra `colcon build` arguments, e.g. COLCON_EXTRA="--cmake-clean-cache" after a package moves.
COLCON_EXTRA ?=

ROS_ENV = source /opt/ros/jazzy/setup.bash;

.PHONY: help setup build test sanitize gate clean

help:
	@echo "uav-navigation (rebuild/v2) commands"
	@echo "  make setup   check host, init submodules, install system dependencies"
	@echo "  make build   colcon build of PKGS and their dependencies (PKGS='$(PKGS)')"
	@echo "  make test    delete stale test results of PKGS, colcon test of PKGS, then colcon test-result"
	@echo "  make sanitize  ThreadSanitizer build + test of $(SANITIZE_PKGS), then rebuild normally"
	@echo "  make gate    tools/uavnav/gate.sh all (static, python, ros)"
	@echo "  make clean   print what would be removed (log/); deletes nothing"
	@echo "Machine limit: builds use -j3 and 2 colcon workers (15 GB RAM)."

setup:
	@tools/setup.sh

build:
	@$(ROS_ENV) nice -n 10 env MAKEFLAGS=-j3 colcon build --packages-up-to $(PKGS) --parallel-workers 2 $(COLCON_EXTRA) --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

test:
	@$(PYTHON) tools/uavnav/clean_test_results.py --build-base build $(PKGS) && { $(ROS_ENV) if test -f install/setup.bash; then source install/setup.bash; fi; colcon test --packages-select $(PKGS) --parallel-workers 2 && rc=0 && for p in $(PKGS); do colcon test-result --verbose --test-result-base build/$$p || rc=1; done; exit $$rc; }

# ThreadSanitizer run (D28) of SANITIZE_PKGS: uavnav_core (SpscQueue, event recorder) and uavnav_lio_core
# (LioFrontend, LioBackend, LioRunner: the two-thread LIO). Builds them in the shared build/ with
# -DUAVNAV_SANITIZE=thread (each package instruments only its own library and tests; fast_lio_core must already
# be installed and is not instrumented), runs their tests, fails closed when this run left no test log of a package, greps
# the logs for TSan warnings, and ALWAYS rebuilds them without the sanitizer afterwards (even when the run
# failed, and on SIGINT/SIGTERM through the trap, exit 130) so build/ and install/ return to the normal state.
# Fails when a build or test failed or a warning was found. Tests run under `setarch -R` (ASLR off): with the
# host's high mmap entropy, TSan from gcc 13 aborts with "unexpected memory mapping" otherwise. One colcon
# worker, -j2; do not run other heavy jobs meanwhile (15 GB RAM).
SANITIZE_PKGS := uavnav_core uavnav_lio_core
SANITIZE_BUILD = nice -n 10 env MAKEFLAGS=-j2 colcon build --packages-select $(SANITIZE_PKGS) --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

sanitize:
	@$(ROS_ENV) rc=0; \
	restore() { echo "make sanitize: restoring the normal (unsanitized) build"; $(SANITIZE_BUILD) -DUAVNAV_SANITIZE=; }; \
	trap 'trap - INT TERM; restore; exit 130' INT TERM; \
	$(SANITIZE_BUILD) -DUAVNAV_SANITIZE=thread || rc=1; \
	if test $$rc -eq 0; then \
	  if test -f install/setup.bash; then source install/setup.bash; fi; \
	  before=$$(readlink log/latest_test 2>/dev/null); \
	  nice -n 10 setarch "$$(uname -m)" -R colcon test --packages-select $(SANITIZE_PKGS) --parallel-workers 1 || rc=1; \
	  if test "$$(readlink log/latest_test 2>/dev/null)" = "$$before"; then \
	    echo "make sanitize: colcon test wrote no new log/latest_test (fail closed)"; rc=1; \
	  fi; \
	  for p in $(SANITIZE_PKGS); do \
	    colcon test-result --verbose --test-result-base build/$$p || rc=1; \
	    if ! test -d log/latest_test/$$p; then \
	      echo "make sanitize: no test log in log/latest_test/$$p/ (fail closed)"; rc=1; \
	    elif grep -rl "WARNING: ThreadSanitizer" log/latest_test/$$p/; then \
	      echo "make sanitize: ThreadSanitizer warning found in $$p (files listed above)"; rc=1; \
	    fi; \
	  done; \
	fi; \
	restore || rc=1; \
	exit $$rc

gate:
	@PYTHON="$(PYTHON)" tools/uavnav/gate.sh all

# build/ and install/ are a shared incremental build (CLAUDE.local.md); never removed here.
clean:
	@echo "make clean deletes nothing. Remove these yourself if you want a clean log:"
	@echo "  log/"
	@echo "build/ and install/ are a shared incremental build and are kept."
