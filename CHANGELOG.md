# Changelog

All notable changes to this project will be documented in this file.

## [1.0.0] - 2025-10-19

### Added
- PID controller for single target navigation
- Multi-waypoint navigator with file-based waypoint loading
- Keyboard teleoperation controller
- Full robot URDF with sensors (LiDAR, IMU, Depth Camera)
- Gazebo simulation environment
- Robot localization with EKF
- YAML configuration files for PID parameters
- Comprehensive README documentation

### Fixed
- Angle normalization bug in rotation checks
- Removed blocking `time.sleep()` calls from callbacks
- Added non-blocking state transition timers
- Improved wheel friction parameters in URDF
- Used xacro properties for diff drive plugin values

### Changed
- Launch files now support `use_sim_time` parameter
- Launch files use YAML config files for parameters
- Improved IMU link definition (removed unnecessary visuals)
- Updated package metadata and license

## [0.0.1] - Initial Release

### Added
- Basic robot structure
- Initial navigation implementation

