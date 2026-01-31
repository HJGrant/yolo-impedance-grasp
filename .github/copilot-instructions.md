# AI Coding Instructions for UR10 Final Project

This workspace is a ROS Noetic project for controlling a UR10 robot using visual servoing. The project uses the `catkin` build system (specifically `catkin tools`).

## Project Architecture

- **Core Framework**: The project is built around the `tum_ics_ur_robot_lli` (Low Level Interface) framework. Controllers inherit from `tum_ics_ur_robot_lli::RobotControllers::ControlEffort`.
- **Math Library**: `control_core` (namespaced as `cc`) is used extensively for mathematical types and rigid body transformations (`cc::HomogeneousTransformation`, `cc::Jacobian`, `cc::LinearPosition`).
- **Robot Model**: `ur_model` provides kinematic and dynamic properties (`model_.T_tool_0`, `model_.J_tool_0`).
- **Vision**: Visual perception nodes publish target points (e.g., to `/target_point`), which the controller consumes.

## Code Conventions

### C++ & ROS
- **Standard**: C++17 (approximate, follow existing CMake settings).
- **Namespaces**:
  - `tum_ics_ur_robot_lli`: Main framework namespace.
  - `cc`: Alias for `control_core`.
  - `ur`: Alias for robot model classes.
- **Types**:
  - Use `Vector6d`, `Matrix6d` (from `control_core` via Eigen) for robot states.
  - Use `cc::JointPosition`, `cc::JointVelocity` for specific state vectors.
  - Use `cc::HomogeneousTransformation` for poses. Do not manually manipulate 4x4 matrices if `cc` helper methods exist.
- **Controller Lifecycle**:
  - `init()`: Load parameters (`ros::param`), initialize variables. Return `bool`.
  - `start()`: Called when controller is activated. Reset flags here.
  - `update(const RobotTime &time, const JointState &current)`: Main control loop running at 500Hz. Returns `Vector6d` (torques).
  - `stop()`: Called when controller stops.

### Specific Patterns
- **Parameter Loading**: Load gains and settings in `init()` using `~private_namespace`.
- **Thread Safety**: Use `std::mutex` (e.g., `target_mutex_`) when accessing data shared between ROS callbacks (e.g., `targetCallback`) and the real-time `update` loop.
- **Transforms**: Use `tf2_ros` for transforming target points into the robot's base frame (typically `ursa_base_link` or `model_.get0Frame()`) before using them in the control loop.
- **Debugging**: Publish `tum_ics_ur_robot_msgs::ControlData` in `update()` for plotting. Use `visualization_msgs::Marker` for Rviz visualization.

## Development Workflow

1.  **Build**:
    ```bash
    catkin build
    ```
    *Note: Do not use `catkin_make` unless you are sure the workspace is configured for it. `catkin build` is preferred.*

2.  **Source**:
    ```bash
    source devel/setup.bash
    ```

3.  **Launch**:
    ```bash
    roslaunch tum_ics_ur10_bringup bringUR10.launch
    ```

## Key Files
- **Controller Implementation**: [src/tum_ics_ur10_controller_tutorial/src/visual_servo_controller.cpp](src/tum_ics_ur10_controller_tutorial/src/visual_servo_controller.cpp) (Reference for control logic).
- **Robot Model**: [src/ur_model/src/model_ur.cpp](src/ur_model/src/model_ur.cpp) (Kinematics/Dynamics).
- **Types**: `src/utilities/control_core/include/control_core/types.h` (Math types).

## Common Tasks

- **Adding a new controller**: Create a class inheriting from `ControlEffort`, register it in `plugin.xml` (or equivalent mechanism if used), and add to `CMakeLists.txt`.
- **Modifying Control Law**: Edit `update()`. Ensure outputs are torque (`Vector6d`).
- **Visual Servoing**: Transform camera points to `base_link` inside the callback or `update` loop before computing error.
