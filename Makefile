SHELL := /bin/bash
# ROS Jazzy's rclpy/ament modules are installed for the system interpreter.
PYTHON ?= /usr/bin/python3
# Packages built and tested by `make build` / `make test`; override with PKGS="...".
PKGS ?= uavnav_core uavnav_interfaces

ROS_ENV = source /opt/ros/jazzy/setup.bash;

.PHONY: help setup build test gate clean

help:
	@echo "uav-navigation (rebuild/v2) commands"
	@echo "  make setup   check host, init submodules, install system dependencies"
	@echo "  make build   colcon build of PKGS and their dependencies (PKGS='$(PKGS)')"
	@echo "  make test    colcon test of PKGS, then colcon test-result"
	@echo "  make gate    tools/uavnav/gate.sh all (static, python, ros)"
	@echo "  make clean   print what would be removed (log/); deletes nothing"
	@echo "Machine limit: builds use -j3 and 2 colcon workers (15 GB RAM)."

setup:
	@tools/setup.sh

build:
	@$(ROS_ENV) nice -n 10 env MAKEFLAGS=-j3 colcon build --packages-up-to $(PKGS) --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

test:
	@$(ROS_ENV) if test -f install/setup.bash; then source install/setup.bash; fi; colcon test --packages-select $(PKGS) --parallel-workers 2 && rc=0 && for p in $(PKGS); do colcon test-result --verbose --test-result-base build/$$p || rc=1; done; exit $$rc

gate:
	@PYTHON="$(PYTHON)" tools/uavnav/gate.sh all

# build/ and install/ are a shared incremental build (CLAUDE.local.md); never removed here.
clean:
	@echo "make clean deletes nothing. Remove these yourself if you want a clean log:"
	@echo "  log/"
	@echo "build/ and install/ are a shared incremental build and are kept."
