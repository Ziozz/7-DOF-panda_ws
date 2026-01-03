#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import math
from pymoveit2 import MoveIt2, GripperInterface
from pymoveit2.robots import panda


class PandaPickPlace(Node):
    def __init__(self):
        super().__init__("panda_pick_place")
        self.start_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, math.radians(-125.0)]
        self.home_joints  = [0.0, 0.0, 0.0, math.radians(-90.0), 0.0, math.radians(92.0), math.radians(50.0)]
        self.drop_joints  = [math.radians(-155.0), math.radians(30.0), math.radians(-20.0),
                             math.radians(-124.0), math.radians(44.0), math.radians(163.0), math.radians(7.0)]

        # ==============================
        # 1. 初始化 MoveIt2（机械臂）
        # ==============================
        self.arm = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name='panda_hand',
            group_name='panda_arm',
        )

        # ==============================
        # 2. 初始化夹爪
        # ==============================
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=panda.gripper_joint_names(),
            open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=panda.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name='panda_hand',
            gripper_command_action_name="gripper_action_controller/gripper_cmd",
        )

        self.get_logger().info("等待 MoveIt 初始化...")
        time.sleep(2.0)

    # ==============================
    # 基础动作封装
    # ==============================
    def go_home(self):
        self.get_logger().info("回到 Home 位姿")
        self.arm.move_to_configuration(self.home_joints)
        self.arm.wait_until_executed()

    def open_gripper(self):
        self.get_logger().info("打开夹爪")
        self.gripper.open()
        self.gripper.wait_until_executed()

    def close_gripper(self):
        self.get_logger().info("闭合夹爪")
        self.gripper.close()
        self.gripper.wait_until_executed()

    # ==============================
    # Pick & Place 主流程
    # ==============================
    def pick_and_place(self):
        # ---- 固定物体位姿（世界坐标系）----
        object_x = 0.45
        object_y = 0.0
        object_z = 0.75
        pick_position=[object_x, object_y, object_z]
        pick_quat_xyzw=[0.0, 1.0, 0.0, 0.0]

        # 1. Move to home joint configuration
        self.arm.move_to_configuration(self.home_joints)
        self.arm.wait_until_executed()

        # ---- 2. 移动到物体上方（Pre-grasp）----
        self.get_logger().info("移动到 Pre-grasp 位姿")
        self.arm.move_to_pose(
            position=[object_x, object_y, object_z + 0.15],
            quat_xyzw=[0.0, 1.0, 0.0, 0.0],  # 末端朝下
        )
        self.arm.wait_until_executed()

        # 3. Move above target (Cartesian)
        self.arm.move_to_pose(position=pick_position, quat_xyzw=pick_quat_xyzw)
        self.arm.wait_until_executed()

        # 4. open gripper
        self.open_gripper()

        # 5. Move down to approach object
        approach_position = [pick_position[0], pick_position[1], pick_position[2] - 0.31]
        self.arm.move_to_pose(position=approach_position, quat_xyzw=pick_quat_xyzw)
        self.arm.wait_until_executed()

        # 6. Close gripper
        self.close_gripper()

        # 7. Move to home joint configuration
        self.arm.move_to_configuration(self.home_joints)
        self.arm.wait_until_executed()

        # 8. Move to drop joint configuration
        self.get_logger().info("移动到丢弃位置")
        self.arm.move_to_configuration(self.drop_joints)
        self.arm.wait_until_executed()

        # ---- 9. 放下 ----
        self.open_gripper()

        # 10. Close gripper
        self.close_gripper()

        # ---- 11. 回 Home ----
        self.go_home()
        self.get_logger().info("Pick-and-place sequence complete.")


def main():
    rclpy.init()
    node = PandaPickPlace()

    node.open_gripper()
    node.go_home()
    node.pick_and_place()

    rclpy.shutdown()


if __name__ == "__main__":
    main()

