# Logging and Monitoring Review: Legacy vs New Repository

## 1. Logging Mechanisms in Legacy Repository

The legacy repository (`/home/letandat/Dev/Mapping_and_Navigation_for_PX4_UAV/`) implements comprehensive logging mechanisms:

### ROS 2 Node Logging
- **RCLCPP Logging**: Extensive use of `RCLCPP_INFO`, `RCLCPP_WARN`, `RCLCPP_ERROR`, and `RCLCPP_DEBUG` throughout the codebase
- **Throttled Logging**: Use of `RCLCPP_WARN_THROTTLE` and `RCLCPP_INFO_THROTTLE` to prevent log spam
- **Structured Messages**: Context-rich log messages that include mode transitions, obstacle distances, path planning status, and performance metrics

### Custom Trajectory Logger
- **File**: `px4_navigation/src/trajectory_logger.cpp`
- **Functionality**: 
  - CSV-based logging of planned paths and executed drone states
  - Timestamped log files with automatic directory creation
  - Dual logging streams: planned waypoints and executed states
  - Periodic flushing to balance performance and data safety
  - Structured schema with headers for external tool compatibility

### Visualization and Analysis Tools
- **recorder.py**: Python-based flight data recorder that captures:
  - PX4 position and status data
  - Voxel map snapshots with sliding window accumulation
  - A* planned paths
  - Mission waypoints
  - Frame-by-frame drone state at 2Hz
- **visualize.py**: Post-flight analysis tool that generates HTML replays with:
  - 3D visualization of drone trajectory
  - Voxel cloud rendering
  - Mission waypoint visualization
  - Performance metrics computation

## 2. Logging Mechanisms in New Repository

The new repository (`/home/letandat/Dev/uav-navigation/`) currently has minimal logging infrastructure:

### Current State
- **Library-Only Structure**: Components exist as libraries without executable nodes
- **No ROS 2 Node Logging**: No `RCLCPP_*` calls found in the source code
- **Unit Test Logging**: Google Test framework used for testing but no runtime logging
- **No Custom Loggers**: No equivalent to the trajectory logger from the legacy system

### Available Components
- **Path Planning**: A* planner with debug information in return structures
- **Grid Management**: LocalPlanGrid with occupancy information
- **Virtual Scanning**: VirtualScan component for obstacle detection

## 3. Health/Timing/Diagnostic Topics in Legacy Repository

The legacy system provides comprehensive observability through multiple channels:

### Diagnostic Topics
- **/nav_path**: Published A* planned paths for visualization
- **/plan_grid_inflated**: Inflated obstacle grid visualization
- **Vehicle Status**: PX4 flight mode transitions and status
- **Odometry**: FAST-LIO2 pose information

### Performance Metrics
- **Control Loop Timing**: Tick duration monitoring with warnings for overruns
- **Planning Performance**: Iteration counts, timeout detection
- **Cross-Track Error**: Distance from drone to planned path and mission corridor
- **Obstacle Detection**: Minimum relevant obstacle distances
- **Velocity Tracking**: Smoothed velocity computation and monitoring

### Health Indicators
- **Mode Transitions**: Clear logging of IDLE → MISSION → ANTICIPATION → AVOIDANCE → RETURNING
- **Error Conditions**: Explicit handling of stale position data, odometry jumps, and communication failures
- **Resource Monitoring**: Memory usage implicit through voxel grid management

## 4. What is Missing in the New Repository for Observability

The new repository lacks critical observability features needed for production deployment:

### Startup and Initialization Logging
- No node lifecycle logging
- Missing configuration parameter validation
- Absence of initialization success/failure reporting

### Periodic Status Reporting
- No heartbeat or periodic status messages
- Missing performance metrics (tick times, memory usage)
- No resource utilization monitoring

### Error Path Diagnostics
- No structured error reporting
- Missing failure mode analysis
- Absence of recovery attempt logging

### Simulation vs Real-world Switch Diagnostics
- No environment detection logging
- Missing simulation-specific behavior indicators
- No hardware-in-the-loop transition monitoring

### Component Health Monitoring
- No health check mechanisms for individual components
- Missing watchdog functionality
- Absence of component status aggregation

## 5. Recommendations for Logging/Monitoring Policy

### Log Levels
1. **DEBUG**: Detailed algorithm internals, intermediate calculations
2. **INFO**: Normal operational messages, mode transitions, successful operations
3. **WARN**: Recoverable issues, suboptimal conditions, automatic corrections
4. **ERROR**: Unrecoverable errors, safety-critical failures, system malfunctions

### Rate-Limited Logs
- Implement throttled logging for high-frequency messages
- Use context-aware throttling (e.g., log once per mode)
- Provide summary logs for repetitive conditions

### Health Publishers
- **Heartbeat Topic**: Regular status updates with component health
- **Performance Metrics**: Planning time, control loop duration, memory usage
- **Diagnostic Aggregates**: Combined status of all subsystems

### Flight-Phase Logging
- **Takeoff Phase**: Specialized logging for initialization sequence
- **Navigation Phase**: Regular path planning and obstacle avoidance logs
- **Landing Phase**: Landing approach and completion status
- **Emergency Phase**: Detailed logging during recovery procedures

### RViz Plugins Integration
- **Path Visualization**: Planned and executed trajectories
- **Obstacle Map**: Real-time voxel grid visualization
- **Status Overlay**: Text-based status information panels

### Bag Recording Strategy
- **Essential Topics**: Position, velocity, planned paths, voxel maps
- **Diagnostic Topics**: Mode transitions, performance metrics, error conditions
- **Configuration**: Selective recording based on flight phase

## 6. Concrete Code Changes for Migration

### Add Executable Nodes
1. **Navigation Node**:
   ```cpp
   // Add main function and rclcpp::spin() loop
   // Implement node lifecycle with proper initialization logging
   // Add parameter validation and error reporting
   ```

2. **Mapping Node**:
   ```cpp
   // Create standalone mapping node with grid publishing
   // Add voxel grid visualization publisher
   // Implement health monitoring for mapping pipeline
   ```

### Implement RCLCPP Logging
1. **Add Standard Logging**:
   ```cpp
   #include <rclcpp/rclcpp.hpp>
   
   // In class constructor:
   logger_ = rclcpp::get_logger("navigation_node");
   
   // Throughout code:
   RCLCPP_INFO(logger_, "Message with %s parameters", param.c_str());
   RCLCPP_WARN(logger_, "Warning condition detected");
   RCLCPP_ERROR(logger_, "Error occurred: %s", error_msg.c_str());
   ```

2. **Add Throttled Logging**:
   ```cpp
   RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000, "Periodic warning message");
   ```

### Create Trajectory Logger Equivalent
1. **Header File** (`trajectory_logger.hpp`):
   ```cpp
   #pragma once
   #include <fstream>
   #include <string>
   #include <vector>
   #include <Eigen/Dense>
   
   class TrajectoryLogger {
   public:
       TrajectoryLogger(const std::string& log_dir);
       ~TrajectoryLogger();
       
       void logPlannedPath(const std::vector<Eigen::Vector3d>& path, double timestamp);
       void logState(double timestamp, double x, double y, double z,
                    double vx, double vy, double yaw, const std::string& mode,
                    double obstacle_dist, double cte, double tick_ms);
   
   private:
       std::ofstream planned_file_;
       std::ofstream executed_file_;
       int plan_id_{0};
       int flush_counter_{0};
   };
   ```

2. **Implementation** (`trajectory_logger.cpp`):
   ```cpp
   // Similar to legacy implementation with proper error handling
   ```

### Add Diagnostic Publishers
1. **Path Publisher**:
   ```cpp
   // Add nav_msgs::msg::Path publisher for planned trajectories
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
   ```

2. **Grid Publisher**:
   ```cpp
   // Add sensor_msgs::msg::PointCloud2 publisher for voxel visualization
   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr grid_pub_;
   ```

### Implement Performance Monitoring
1. **Timing Infrastructure**:
   ```cpp
   #include <chrono>
   
   auto start = std::chrono::steady_clock::now();
   // ... operations ...
   auto end = std::chrono::steady_clock::now();
   auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
   
   RCLCPP_DEBUG(logger_, "Operation took %ld ms", duration.count());
   ```

2. **Health Status Publisher**:
   ```cpp
   // Add diagnostic_msgs publisher for component health
   rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
   ```

### Add Configuration for Logging
1. **YAML Parameters**:
   ```yaml
   navigation_node:
     ros__parameters:
       log_level: "INFO"
       log_directory: "/tmp/navigation_logs"
       enable_trajectory_logging: true
       enable_performance_monitoring: true
   ```

2. **Parameter Validation**:
   ```cpp
   // In node constructor:
   declare_parameter("log_level", "INFO");
   declare_parameter("log_directory", "/tmp/navigation_logs");
   declare_parameter("enable_trajectory_logging", true);
   
   std::string log_level = get_parameter("log_level").as_string();
   // Set logger level based on parameter
   ```

### Integration with Existing Components
1. **AStarPlanner Enhancements**:
   ```cpp
   // Add logger to AStarPlanner constructor
   // Add debug logging for planning iterations
   // Include timing information in PlanResult
   ```

2. **LocalPlanGrid Visualization**:
   ```cpp
   // Add method to publish grid as PointCloud2
   // Include occupancy status in visualization
   ```

These changes would bring the new repository's logging and monitoring capabilities up to the standard of the legacy system while maintaining the improved architecture and modularity.