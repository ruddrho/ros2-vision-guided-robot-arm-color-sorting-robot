# Changelog

## 5.0.1

- Increased the trajectory-result timeout margin from 10 s to 20 s to prevent successful controller goals from being incorrectly classified as timeouts.
- Updated README repository references and installation paths.
- Corrected the two-table workcell diagram to match the verified Gazebo layout.

## 5.0.0

- Retained the stable educational Xacro geometry after evaluating an
  experimental STL-shell redesign; the unused mesh assets remain available as
  reproducible design resources.
- Reduced both worktables to equal `0.44 m x 0.44 m` tops to improve clearance
  around the centrally mounted arm.
- Added an elbow-up manipulation branch and a low destination release posture
  to keep the arm housing clear of the tabletop.
- Added five color-specific source and transfer motion profiles plus visible
  placement-wrist rotation.
- Added safe initialization and return to an unchanged home configuration after
  every completed command.
- Added a project-specific Gazebo GUI configuration with a front-oriented
  workcell camera while preserving the standard Gazebo interface.
- Preserved the five-color OpenCV pipeline, independent arm and gripper
  controllers, DetachableJoint channels, topics, and sequential command flow.

## 4.1.0

- Restricted OpenCV contour search to the source-table camera region so the
  blue arm and destination markers cannot be selected as cubes.
- Increased the gripper contact-pad width and corrected the grasp height.
- Added one Gazebo DetachableJoint channel per color for reliable simulated
  grasp retention after reaching the calibrated grasp pose and immediately
  before finger closure, with detachment at the matching destination before
  the gripper opens.
- Preserved command-driven, arbitrary-order sorting and repeated-color rejection.
- Documented grasp stabilization explicitly instead of representing it as
  hardware-equivalent force control.

## 4.0.0

- Rebuilt the workcell around two identical tables with the robot between them.
- Added five calibrated source slots and five separate color-coded destination slots.
- Added color-specific placement targeting so command order controls which cube moves next.
- Added session tracking that rejects a color already placed in its destination.
- Added an automatically launched annotated top-view camera window.
- Restyled the original arm with sky-blue industrial shells, rounded joints, dark actuator modules, and metallic gripper details without changing collision geometry.
- Increased the overhead camera coverage for the complete two-table cell.
- Kept physical-grasp success unclaimed until a ROS 2 Jazzy and Gazebo Harmonic runtime trial is completed.

## 3.1.0

- Split the combined eight-joint controller into independent arm and gripper
  trajectory controllers.
- Added the `color_command` C++ utility for validated, discovery-aware terminal
  commands.
- Tightened finger goal tolerance from `0.022 m` to `0.003 m`.
- Replaced collision-penetrating pickup and placement configurations with
  table-clear grasp configurations.
- Restored the Gazebo position-interface proportional gain to `1.0`.
- Corrected cube feedback to use the Gazebo model pose rather than the leading
  zero-valued link pose.
- Added post-lift grasp-retention verification and controlled recovery.
- Allowed the coordinator to return to the command-waiting state after a trial.
- Updated the architecture figure, README, and technical report to distinguish
  verified baselines from the v3.1 runtime state.

## 3.0.0

- Added five-color C++ OpenCV detection and command-conditioned selection.
- Added the overhead Gazebo RGB camera and annotated debug-image topic.
- Redesigned the educational manipulator with industrial visual details.
- Added the five-cube workcell and color-specific pose feedback.
