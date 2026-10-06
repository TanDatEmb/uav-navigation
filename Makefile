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
	@echo "  make test    colcon test of PKGS, then colcon test-result"
	@echo "  make sanitize  ThreadSanitizer build + test of uavnav_core, then rebuild normally"
	@echo "  make gate    tools/uavnav/gate.sh all (static, python, ros)"
	@echo "  make clean   print what would be removed (log/); deletes nothing"
	@echo "Machine limit: builds use -j3 and 2 colcon workers (15 GB RAM)."

setup:
	@tools/setup.sh

build:
	@$(ROS_ENV) nice -n 10 env MAKEFLAGS=-j3 colcon build --packages-up-to $(PKGS) --parallel-workers 2 $(COLCON_EXTRA) --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

test:
	@$(ROS_ENV) if test -f install/setup.bash; then source install/setup.bash; fi; colcon test --packages-select $(PKGS) --parallel-workers 2 && rc=0 && for p in $(PKGS); do colcon test-result --verbose --test-result-base build/$$p || rc=1; done; exit $$rc

# ThreadSanitizer run of uavnav_core (D28). Builds the shared build/ with -fsanitize=thread, runs the
# tests, greps the logs for TSan warnings, and ALWAYS rebuilds without the sanitizer afterwards (even
# when the run failed) so build/ and install/ return to the normal state. Fails when a build or test
# failed or a warning was found. Tests run under `setarch -R` (ASLR off): with the host's high
# mmap entropy, TSan from gcc 13 aborts with "unexpected memory mapping" otherwise. Do not run other heavy jobs meanwhile (15 GB RAM).
sanitize:
	@$(ROS_ENV) rc=0; \
	nice -n 10 env MAKEFLAGS=-j2 colcon build --packages-select uavnav_core --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DUAVNAV_SANITIZE=thread || rc=1; \
	if test $$rc -eq 0; then \
	  if test -f install/setup.bash; then source install/setup.bash; fi; \
	  nice -n 10 setarch "$$(uname -m)" -R colcon test --packages-select uavnav_core --parallel-workers 1 || rc=1; \
	  colcon test-result --verbose --test-result-base build/uavnav_core || rc=1; \
	  if grep -rl "WARNING: ThreadSanitizer" log/latest_test/uavnav_core/ 2>/dev/null; then \
	    echo "make sanitize: ThreadSanitizer warning found (files listed above)"; rc=1; \
	  fi; \
	fi; \
	echo "make sanitize: restoring the normal (unsanitized) build"; \
	nice -n 10 env MAKEFLAGS=-j2 colcon build --packages-select uavnav_core --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DUAVNAV_SANITIZE= || rc=1; \
	exit $$rc

gate:
	@PYTHON="$(PYTHON)" tools/uavnav/gate.sh all

# build/ and install/ are a shared incremental build (CLAUDE.local.md); never removed here.
clean:
	@echo "make clean deletes nothing. Remove these yourself if you want a clean log:"
	@echo "  log/"
	@echo "build/ and install/ are a shared incremental build and are kept."
