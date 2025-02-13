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

## Usage on hardware

To run the software on real ROSbot XL, also communication with Digital Board will be necessary.
First update your firmware to make sure that you use the latest version, then run the `micro-ROS` agent.
For detailed instructions refer to the [rosbot_xl_firmware repository](https://github.com/husarion/rosbot_xl_firmware).

## Source build

### Prerequisites

Install `colcon`, `vsc` and `rosdep`:

```bash
sudo apt-get update
sudo apt-get install -y ros-dev-tools python3-pip
```

Create workspace folder and clone `rosbot_xl_ros` repository:

```bash
mkdir -p ros2_ws/src
cd ros2_ws
git clone https://github.com/husarion/rosbot_xl_manipulation_ros.git src/rosbot_xl_manipulation_ros
```

### Build and run on hardware

Building:

```bash
export HUSARION_ROS_BUILD=hardware

source /opt/ros/$ROS_DISTRO/setup.bash

vcs import src < src/rosbot_xl_manipulation_ros/rosbot_xl_manipulation/rosbot_xl_manipulation.repos
vcs import src < src/rosbot_ros/rosbot/rosbot_hardware.repos
vcs import src < src/open_manipulator_x/open_manipulator_x.repos

rm -r src/rosbot_xl_ros/rosbot_xl_gazebo
rm -r src/rosbot_xl_manipulation_ros/rosbot_xl_manipulation_gazebo
cp -r src/ros2_controllers/diff_drive_controller src
cp -r src/ros2_controllers/imu_sensor_broadcaster src
rm -rf src/ros2_controllers

rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> [!IMPORTANT]
> Before starting the software on the robot please create udev rules:
>
> `echo 'ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", SYMLINK+="ttyMANIPULATOR"' | sudo tee /etc/udev/rules.d/10-local.rules`

Running:

```bash
source install/setup.bash
ros2 launch rosbot_xl_manipulation_bringup bringup.launch.py
```

### Build and run Gazebo simulation

Building:

```bash
export HUSARION_ROS_BUILD=simulation

source /opt/ros/$ROS_DISTRO/setup.bash

vcs import src < src/rosbot_xl_manipulation_ros/rosbot_xl_manipulation/rosbot_xl_manipulation.repos
vcs import src < src/rosbot_ros/rosbot/rosbot_simulation.repos
vcs import src < src/open_manipulator_x/open_manipulator_x.repos

rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Running:

```bash
source install/setup.bash
ros2 launch rosbot_xl_manipulation_gazebo simulation.launch.py
```

## Demos

For further usage examples check out our other repositories:

* [`rosbot-xl-docker`](https://github.com/husarion/rosbot-xl-docker) - Dockerfiles for building hardware and simulation images
* [`rosbot-xl-manipulation`](https://github.com/husarion/rosbot-xl-manipulation) - integration of ROSbot XL with OpenManipulatorX
