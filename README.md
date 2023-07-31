# rosbot_xl_manipulation_ros

ROS packages for ROSbot XL with a manipulator.
## ROS packages

### `rosbot_xl_manipulation`

Metapackeage that contains dependencies to other repositories. It is also used to define whether simulation dependencies should be used. 

### `rosbot_xl_manipulation_bringup`

Package that contains launch, which starts all base functionalities: ros2 controllers for wheels and manipulator, MoveIt with servo control, `robot_localization` and `laser_filters` nodes (`rosbot_xl_bringup` configs are used).

### `rosbot_xl_manipulation_controller`

ROS2 hardware controller for ROSbot XL with manipulator. Starts controller for wheels, manipulator and gripper.

### `rosbot_xl_manipulation_description`

URDF model used for both simulation and as a source of transforms on physical robot. It includes robot model from `rosbot_xl_description`, RPlidar S1 and OpenManipulatorX. 

As there aren't any dedicated collision meshes available for OpenManipulatorX, visual ones are used (just as in the original OpenManipulatorX repository). It is possible to disable collisions using `manipulator_collision_enabled` argument passed to the URDF - we opted to disable collisions in Gazebo, as they resulted in large drop in simulation performance. Collisions are enabled for MoveIt, so collisions checking still works.

### `rosbot_xl_manipulation_gazebo`

Launch files for Ignition Gazebo working with ROS2 control.

### `rosbot_xl_manipulation_moveit`

Configs for launching MoveIt2 with OpenManipulatorX. Also includes servo config and node that translates joy commands to servo commands.

## ROS API

Available in [ROS_API.md](./ROS_API.md)