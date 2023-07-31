Use `bringup.launch.py` from `rosbot_xl_manipulation_bringup` to start all base functionalities for ROSbot XL. 
In `rosbot_xl_manipulation_ros` original ROSbot XL interface is extended with OpenMANIPULATOR-X.
Below you can find the manipulator API, for ROSbot XL ROS API, please refer to [the one in the `rosbot_xl_ros` repository](https://github.com/husarion/rosbot_xl_ros/blob/master/ROS_API.md).

- `gripper_controller` - `GripperActionController` ROS 2 controller (for details refer to the [official documentation](https://control.ros.org/master/doc/ros2_controllers/gripper_controllers/doc/userdoc.html))

  **Action Servers**
  - `/gripper_controller/gripper_cmd` (*control_msgs/action/GripperCommand*)


- `manipulator_controller` - `JointTrajectoryController` ROS 2 controller (for details refer to the [official documentation](https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html))

  **Subscribes**
  - `/manipulator_controller/joint_trajectory` (*trajectory_msgs/msg/JointTrajectory*)

  **Publishes**
  - `/manipulator_controller/state` (*control_msgs/msg/JointTrajectoryControllerState*)

  **Action Servers**:
  - `/manipulator_controller/follow_joint_trajectory` (*control_msgs/action/FollowJointTrajectory*)


- `move_group` - key node provided by MoveIt 2 ([documentation](https://moveit.picknik.ai/humble/doc/concepts/move_group.html))

  **Subscribes**
  - `/trajectory_execution_event` (*std_msgs/msg/String*)

  **Publishes**
  - `/display_contacts` (*visualization_msgs/msg/MarkerArray*)
  - `/display_planned_path` (*moveit_msgs/msg/DisplayTrajectory*)
  - `/motion_plan_request` (*moveit_msgs/msg/MotionPlanRequest*)
  - `/robot_description_semantic` (*std_msgs/msg/String*)

  **Service Servers**
  - `/apply_planning_scene` (*moveit_msgs/srv/ApplyPlanningScene*)
  - `/check_state_validity` (*moveit_msgs/srv/GetStateValidity*)
  - `/clear_octomap` (*std_srvs/srv/Empty*)
  - `/compute_cartesian_path` (*moveit_msgs/srv/GetCartesianPath*)
  - `/compute_fk` (*moveit_msgs/srv/GetPositionFK*)
  - `/compute_ik` (*moveit_msgs/srv/GetPositionIK*)
  - `/get_planner_params` (*moveit_msgs/srv/GetPlannerParams*)
  - `/load_map` (*moveit_msgs/srv/LoadMap*)
  - `/plan_kinematic_path` (*moveit_msgs/srv/GetMotionPlan*)
  - `/query_planner_interface` (*moveit_msgs/srv/QueryPlannerInterfaces*)
  - `/save_map` (*moveit_msgs/srv/SaveMap*)
  - `/set_planner_params` (*moveit_msgs/srv/SetPlannerParams*)

  **Action Servers**
  - `/execute_trajectory` (*moveit_msgs/action/ExecuteTrajectory*)
  - `/move_action` (*moveit_msgs/action/MoveGroup*)


- `moveit_simple_controller_manager` - provides interface to lower level controllers ([documentation](https://moveit.picknik.ai/humble/doc/examples/controller_configuration/controller_configuration_tutorial.html#example-controller-manager))

  **Action Clients**
  - `/gripper_controller/gripper_cmd` (*control_msgs/action/GripperCommand*)
  - `/manipulator_controller/follow_joint_trajectory` (*control_msgs/action/FollowJointTrajectory*)


- `joy_servo_node` - translates messages from gamepad to servo commands - moving the manipulator in cartesian or joint space. Also provides additional functions - moving manipulator to the Home position and opening/closing the gripper.

  **Subscribes**
  - `/joy` (*sensor_msgs/msg/Joy*)

  **Publishes**
  - `/servo_node/delta_joint_cmds` (*control_msgs/msg/JointJog*)
  - `/servo_node/delta_twist_cmds` (*geometry_msgs/msg/TwistStamped*)


- `servo_node` - based on cartesian or joint commands calculates joint trajectory ([documentation](https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html))

  **Subscribes**
  - `/servo_node/collision_velocity_scale` (*std_msgs/msg/Float64*)
  - `/servo_node/delta_joint_cmds` (*control_msgs/msg/JointJog*)
  - `/servo_node/delta_twist_cmds` (*geometry_msgs/msg/TwistStamped*)

  **Publishes**
  - `/manipulator_controller/joint_trajectory` (*trajectory_msgs/msg/JointTrajectory*)
  - `/servo_node/collision_velocity_scale` (*std_msgs/msg/Float64*)
  - `/servo_node/condition` (*std_msgs/msg/Float64*)
  - `/servo_node/status` (*std_msgs/msg/Int8*)

  **Service Servers**
  - `/servo_node/change_control_dimensions` (*moveit_msgs/srv/ChangeControlDimensions*)
  - `/servo_node/change_drift_dimensions` (*moveit_msgs/srv/ChangeDriftDimensions*)
  - `/servo_node/pause_servo` (*std_srvs/srv/Trigger*)
  - `/servo_node/reset_servo_status` (*std_srvs/srv/Empty*)
  - `/servo_node/start_servo` (*std_srvs/srv/Trigger*)
  - `/servo_node/stop_servo` (*std_srvs/srv/Trigger*)
  - `/servo_node/unpause_servo` (*std_srvs/srv/Trigger*)
