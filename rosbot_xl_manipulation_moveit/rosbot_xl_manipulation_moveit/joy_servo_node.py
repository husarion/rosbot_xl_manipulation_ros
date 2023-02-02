#!/usr/bin/env python3

from abc import ABC, abstractmethod

from threading import Thread

from math import fabs

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.node import Node

from control_msgs.msg import JointJog
from geometry_msgs.msg import Twist, TwistStamped, Vector3
from sensor_msgs.msg import Joy

from pymoveit2 import MoveIt2Gripper, MoveIt2


class JoyControl(ABC):
    @abstractmethod
    def is_pressed(self, msg: Joy) -> bool:
        raise NotImplementedError

    @abstractmethod
    def get_value(self, msg: Joy) -> float:
        raise NotImplementedError


# Defines axis control - continuous values from max negatice to max positive
class AxisControl(JoyControl):
    def __init__(self, axis_id: int, axis_deadzone: float, scaling=1.0, inverted=False):
        self.axis_id = axis_id
        self.axis_deadzone = axis_deadzone
        self.scaling = -scaling if inverted else scaling

    def is_pressed(self, msg: Joy) -> bool:
        return fabs(msg.axes[self.axis_id]) > self.axis_deadzone

    def get_value(self, msg: Joy) -> float:
        return msg.axes[self.axis_id] * self.scaling


# Defines control that can have negative, positive or zero value
class DoubleButtonControl(JoyControl):
    def __init__(self, positive_button_id: int, negative_button_id: int, scaling=1.0):
        self.positive_button_id = positive_button_id
        self.negative_button_id = negative_button_id
        self.scaling = scaling

    def is_pressed(self, msg: Joy) -> bool:
        return (
            msg.buttons[self.positive_button_id] or msg.buttons[self.negative_button_id]
        )

    def get_value(self, msg: Joy) -> float:
        return (
            msg.buttons[self.positive_button_id] - msg.buttons[self.negative_button_id]
        ) * self.scaling


# Defines control that can have two values: positive or zero
class SingleButtonControl(JoyControl):
    def __init__(self, button_id: int, scaling=1.0):
        self.button_id = button_id
        self.scaling = scaling

    def is_pressed(self, msg: Joy) -> bool:
        return msg.buttons[self.button_id]

    def get_value(self, msg: Joy) -> float:
        return msg.buttons[self.button_id] * self.scaling


class TeleopJoy(Node):
    def __init__(self):
        super().__init__("joy_servo_node")

        self.cartesian_cmd_names = [
            "linear_x",
            "linear_y",
            "linear_z",
            "angular_x",
            "angular_y",
            "angular_z",
        ]
        self.parse_parameters()

        self.joint_cmds_pub = self.create_publisher(
            JointJog, "servo_node/delta_joint_cmds", 10
        )
        self.twist_cmds_pub = self.create_publisher(
            TwistStamped, "servo_node/delta_twist_cmds", 10
        )

        self.callback_group_joy = MutuallyExclusiveCallbackGroup()
        self.joy_sub = self.create_subscription(
            Joy,
            "joy",
            self.joy_cb,
            qos_profile=10,
            callback_group=self.callback_group_joy,
        )

        # Create callback group that allows execution of callbacks in parallel without restrictions
        self.callback_group_gripper = ReentrantCallbackGroup()
        self.callback_group_manipulator = ReentrantCallbackGroup()

        # For controlling grippers position
        self.moveit2_gripper = MoveIt2Gripper(
            node=self,
            gripper_joint_names=self.gripper_joint_names,
            open_gripper_joint_positions=[self.opened_gripper_position],
            closed_gripper_joint_positions=[self.closed_gripper_position],
            gripper_group_name="gripper",
            callback_group=self.callback_group_gripper,
            execute_via_moveit=True,
        )

        # For moving arm to home position
        self.moveit2_manipulator = MoveIt2(
            node=self,
            joint_names=self.joint_names,
            base_link_name="base_link",
            end_effector_name=self.end_effector_name,
            group_name="manipulator",
            callback_group=self.callback_group_manipulator,
            execute_via_moveit=True,
        )

    def joy_cb(self, msg: Joy) -> None:
        if not self.dead_man_switch.is_pressed(msg):
            self.stop_manipulator()
            return

        if self.home_manipulator.is_pressed(msg):
            self.move_to_home()
        elif self.check_if_pressed(msg, self.manipulator_cartesian_controls):
            cmds = self.calculate_command(
                msg=msg,
                cmd_names=self.cartesian_cmd_names,
                controls_map=self.manipulator_cartesian_controls,
            )
            self.send_cartesian_command(cmds)
        elif self.check_if_pressed(msg, self.manipulator_joints_controls):
            cmds = self.calculate_command(
                msg=msg,
                cmd_names=self.joint_names,
                controls_map=self.manipulator_joints_controls,
            )
            self.send_joint_command(cmds)
        else:
            self.stop_manipulator()

        self.control_gripper(msg)

    def stop_manipulator(self) -> None:
        self.send_cartesian_command([0.0 for _ in self.cartesian_cmd_names])
        self.send_joint_command([0.0 for _ in self.joint_names])

    def check_if_pressed(self, msg: Joy, controls_map: dict) -> bool:
        for control_name in controls_map:
            if controls_map[control_name].is_pressed(msg):
                return True
        return False

    def calculate_command(self, msg: Joy, cmd_names: list, controls_map: dict) -> list:
        cmds = [0.0 for _ in cmd_names]
        for control_name in controls_map:
            if controls_map[control_name].is_pressed(msg):
                idx = cmd_names.index(control_name)
                cmds[idx] = controls_map[control_name].get_value(msg)
        return cmds

    def send_cartesian_command(self, cmd: list) -> None:
        cartesian_cmd_msg = TwistStamped()
        cartesian_cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cartesian_cmd_msg.header.frame_id = self.cartesian_control_reference_frame
        cartesian_cmd_msg.twist = Twist(
            linear=Vector3(x=cmd[0], y=cmd[1], z=cmd[2]),
            angular=Vector3(x=cmd[3], y=cmd[4], z=cmd[5]),
        )
        self.twist_cmds_pub.publish(cartesian_cmd_msg)

    def send_joint_command(self, cmd: list) -> None:
        joint_cmd_msg = JointJog()
        joint_cmd_msg.header.stamp = self.get_clock().now().to_msg()
        joint_cmd_msg.duration = 0.0  # it isn't used
        joint_cmd_msg.joint_names = self.joint_names
        joint_cmd_msg.velocities = cmd
        self.joint_cmds_pub.publish(joint_cmd_msg)

    def move_to_home(self) -> None:
        self.moveit2_manipulator.move_to_configuration(
            self.home_joint_configuration,
            self.joint_names,
        )
        self.moveit2_manipulator.wait_until_executed()

    def control_gripper(self, msg: Joy) -> None:
        if self.gripper_close.is_pressed(msg) and not self.moveit2_gripper.is_closed:
            self.moveit2_gripper.close()
            self.moveit2_gripper.wait_until_executed()
        elif self.gripper_open.is_pressed(msg) and not self.moveit2_gripper.is_open:
            self.moveit2_gripper.open()
            self.moveit2_gripper.wait_until_executed()

    def parse_parameters(self) -> None:
        self.declare_parameters(
            namespace="",
            parameters=[
                ("axis_deadzone", 0.05),
                ("joint_control_velocity", 0.5),
                ("cartesian_control_velocity_linear", 0.1),
                ("cartesian_control_velocity_angular", 0.4),
                ("gripper_control.open.position", 0.009),
                ("gripper_control.close.position", -0.009),
                ("end_effector_name", "end_effector_link"),
                ("gripper_joint_names", ["gripper_left_joint"]),
                ("cartesian_control_reference_frame", "link2"),
                ("cartesian_control_names", Parameter.Type.STRING_ARRAY),
                ("joint_names", Parameter.Type.STRING_ARRAY),
                ("home_joint_configuration", Parameter.Type.DOUBLE_ARRAY),
            ],
        )

        self.axis_deadzone = self.get_parameter("axis_deadzone").value
        self.joint_control_velocity = self.get_parameter("joint_control_velocity").value
        self.cartesian_control_velocity_linear = self.get_parameter(
            "cartesian_control_velocity_linear"
        ).value
        self.cartesian_control_velocity_angular = self.get_parameter(
            "cartesian_control_velocity_angular"
        ).value
        self.opened_gripper_position = self.get_parameter(
            "gripper_control.open.position"
        ).value
        self.closed_gripper_position = self.get_parameter(
            "gripper_control.close.position"
        ).value
        self.end_effector_name = self.get_parameter("end_effector_name").value
        self.gripper_joint_names = self.get_parameter("gripper_joint_names").value
        self.cartesian_control_reference_frame = self.get_parameter(
            "cartesian_control_reference_frame"
        ).value

        try:
            self.joint_names = self.get_parameter("joint_names").value
            self.cartesian_control_names = self.get_parameter(
                "cartesian_control_names"
            ).value
            self.home_joint_configuration = self.get_parameter(
                "home_joint_configuration"
            ).value
        except rclpy.exceptions.ParameterUninitializedException as e:
            error_msg = f"Required parameter not defined: {str(e)}"
            self.get_logger().error(error_msg)
            raise Exception(error_msg)

        self.dead_man_switch = self.parse_joy_control(param_namespace="dead_man_switch")
        self.home_manipulator = self.parse_joy_control(
            param_namespace="home_manipulator"
        )

        self.manipulator_joints_controls = {}
        for joint_name in self.joint_names:
            param_namespace = "joints_control." + joint_name
            self.manipulator_joints_controls[joint_name] = self.parse_joy_control(
                param_namespace=param_namespace, scaling=self.joint_control_velocity
            )

        self.manipulator_cartesian_controls = {}
        for cartesian_control_name in self.cartesian_control_names:
            if not cartesian_control_name in self.cartesian_cmd_names:
                self.get_logger().error(
                    f"Unknown cartesian control type {cartesian_control_name},"
                    f" currently supported names: {self.cartesian_control_names}"
                )
                continue

            param_namespace = "cartesian_control." + cartesian_control_name

            scaling = 1.0
            if "linear" in cartesian_control_name:
                scaling = self.cartesian_control_velocity_linear
            elif "angular" in cartesian_control_name:
                scaling = self.cartesian_control_velocity_angular

            self.manipulator_cartesian_controls[
                cartesian_control_name
            ] = self.parse_joy_control(param_namespace=param_namespace, scaling=scaling)

        self.gripper_open = self.parse_joy_control(
            param_namespace="gripper_control.open"
        )
        self.gripper_close = self.parse_joy_control(
            param_namespace="gripper_control.close"
        )

    def parse_joy_control(self, param_namespace: str, scaling=1.0) -> JoyControl:
        # Checks control type and creates appropriate implementation of JoyControl abstraction

        self.declare_parameter(param_namespace + ".control_type", "")
        control_type = self.get_parameter(param_namespace + ".control_type").value

        if control_type == "double_button":
            self.declare_parameter(param_namespace + ".positive_button_id", Parameter.Type.INTEGER)
            self.declare_parameter(param_namespace + ".negative_button_id", Parameter.Type.INTEGER)

            button_cw_id = self.get_parameter(
                param_namespace + ".positive_button_id"
            ).value
            button_ccw_id = self.get_parameter(
                param_namespace + ".negative_button_id"
            ).value
            controller = DoubleButtonControl(
                positive_button_id=button_cw_id,
                negative_button_id=button_ccw_id,
                scaling=scaling,
            )
        elif control_type == "axis":
            self.declare_parameter(param_namespace + ".axis_id", Parameter.Type.INTEGER)
            self.declare_parameter(param_namespace + ".inverted", False)
            axis_id = self.get_parameter(param_namespace + ".axis_id").value
            inverted = self.get_parameter(param_namespace + ".inverted").value
            controller = AxisControl(axis_id, self.axis_deadzone, scaling=scaling, inverted=inverted)
        elif control_type == "single_button":
            self.declare_parameter(param_namespace + ".button_id", Parameter.Type.INTEGER)
            button_id = self.get_parameter(param_namespace + ".button_id").value
            controller = SingleButtonControl(button_id=button_id, scaling=scaling)
        else:
            self.get_logger().error(
                f"Unknown control type {control_type} in {param_namespace}, currently"
                " supported types: single_button, double_button, axis. Please make"
                " sure that it is defined."
            )
            raise Exception("Unknown control type")

        return controller


def main():
    rclpy.init()
    servo_joy_node = TeleopJoy()
    servo_joy_node.get_logger().info("Starting")

    # Two threads are necessary to operate with pymoveit2 - otherwise waiting for execution won't work properly
    executor = MultiThreadedExecutor(2)
    executor.add_node(servo_joy_node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    executor_thread.join()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
