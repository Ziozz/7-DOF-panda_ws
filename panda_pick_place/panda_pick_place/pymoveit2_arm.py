#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arm_pymoveit2.py
使用 pymoveit2 的鲁棒移动脚本（首选 MoveIt，失败则 fallback 到控制器）
功能：
 - 等待 /joint_states 稳定
 - 优先调用 pymoveit2.MoveIt2.move_to_configuration() 进行规划+执行
 - 若 MoveIt 未发轨迹或规划失败，则用 FollowJointTrajectory 回退到控制器执行
 - 回退成功后同步 MoveIt 内部状态，确保 RViz 实体位置和轨迹一致
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from pymoveit2 import MoveIt2
from pymoveit2.robots import panda
import time, math, sys

class PyMoveIt2Arm(Node):
    def __init__(self, controller_name='panda_arm_controller'):
        super().__init__('arm_pymoveit2')

        # 最新 joint_states
        self.latest_js = None
        self.js_times = []
        self.JS_STABLE_COUNT = 3     # 连续收到几条更新视为稳定
        self.JS_STABLE_WINDOW = 0.25 # 连续更新的最大间隔（秒）

        # 订阅 /joint_states
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        # pymoveit2 的高层接口（MoveIt2）
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name="panda_arm",
        )

        # 控制器 action client（回退用）
        self.action_client = ActionClient(self, FollowJointTrajectory,
                                          f'/{controller_name}/follow_joint_trajectory')

    # joint_states 回调：记录时间戳用于判断“稳定”
    def _js_cb(self, msg: JointState):
        self.latest_js = msg
        self.js_times.append(time.time())
        if len(self.js_times) > 50:
            self.js_times = self.js_times[-50:]

    # 等待 joint_states 连续稳定（避免 race）
    def wait_for_js_stable(self, timeout=12.0):
        self.get_logger().info("等待 /joint_states 连续稳定...")
        start = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_js is None:
                if time.time()-start > timeout:
                    return False
                continue
            if len(self.js_times) >= self.JS_STABLE_COUNT:
                tail = self.js_times[-self.JS_STABLE_COUNT:]
                max_gap = max([t2 - t1 for t1, t2 in zip(tail[:-1], tail[1:])]) if len(tail) > 1 else 0.0
                if max_gap <= self.JS_STABLE_WINDOW:
                    return True
            if time.time()-start > timeout:
                return False
        return False

    # 获取按 panda.joint_names 排序的当前关节列表
    def get_current_joints(self):
        if self.latest_js is None:
            return None
        m = dict(zip(self.latest_js.name, self.latest_js.position))
        return [float(m.get(n, 0.0)) for n in panda.joint_names()]

    # 欧氏距离（关节空间）
    @staticmethod
    def _dist(a, b):
        return math.sqrt(sum((ai - bi)**2 for ai, bi in zip(a,b)))

    # 检测是否开始运动（用于判断 MoveIt 是否发出轨迹）
    def _detect_motion_started(self, before, timeout=2.5, thresh=1e-3):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = self.get_current_joints()
            if now is None:
                continue
            if self._dist(now, before) > thresh:
                return True
        return False

    # 等待达到目标（关节空间）
    def _wait_reach(self, target, timeout=12.0, tol=0.02):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = self.get_current_joints()
            if now is None:
                continue
            if self._dist(now, target) <= tol:
                return True
        return False

    # 尝试用 MoveIt 规划并执行
    def try_moveit(self, target):
        self.get_logger().info("尝试通过 MoveIt 规划并执行...")
        before = self.get_current_joints()
        if before is None:
            self.get_logger().error("未收到 joint_states，无法开始 MoveIt 规划")
            return False

        # 先把 MoveIt 内部状态与 /joint_states 对齐（尽量避免start-state race）
        try:
            self.moveit2.robot.set_joint_positions(before)
            self.moveit2.robot.update()
        except Exception:
            # 如果环境没有这些接口，也不致命，MoveIt 会从 /joint_states 读取
            pass

        # 请求规划并执行（pymoveit2 高层）
        try:
            self.moveit2.move_to_configuration(target)
        except Exception as e:
            self.get_logger().warn(f"MoveIt 请求抛出异常: {e}")
            return False

        # 检测是否开始移动
        if not self._detect_motion_started(before):
            self.get_logger().warn("MoveIt 没有启动实际运动（规划或执行失败）")
            return False

        # 等待到达
        if not self._wait_reach(target):
            self.get_logger().warn("MoveIt 执行未在限定时间内到达目标")
            return False

        self.get_logger().info("MoveIt 规划并执行成功")
        return True

    # 回退：发送 FollowJointTrajectory 到控制器
    def fallback_controller(self, target, duration=5.0):
        self.get_logger().info("使用 FollowJointTrajectory 回退到控制器执行...")
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("控制器 action server 不可用")
            return False

        cur = self.get_current_joints()
        if cur is None:
            self.get_logger().error("无当前关节信息，回退失败")
            return False

        traj = JointTrajectory()
        traj.joint_names = panda.joint_names()
        p0 = JointTrajectoryPoint()
        p0.positions = cur
        p0.time_from_start.sec = 0
        p1 = JointTrajectoryPoint()
        p1.positions = target
        secs = float(duration)
        p1.time_from_start.sec = int(math.floor(secs))
        p1.time_from_start.nanosec = int((secs - math.floor(secs)) * 1e9)
        traj.points = [p0, p1]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        # 发送并等待结果
        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        if not send_future.done():
            self.get_logger().error("发送轨迹到控制器超时")
            return False
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("控制器拒绝轨迹目标")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration+10.0)
        if not result_future.done():
            self.get_logger().error("等待控制器结果超时")
            return False
        res = result_future.result().result
        if hasattr(res, 'error_code') and res.error_code == 0:
            # 回退成功：尽量同步 MoveIt 内部状态（确保 RViz 显示实体）
            try:
                self.moveit2.robot.set_joint_positions(target)
                self.moveit2.robot.update()
            except Exception:
                pass
            self.get_logger().info("控制器执行成功，已同步 MoveIt 状态")
            return True
        else:
            self.get_logger().error(f"控制器执行失败: {getattr(res,'error_code', res)}")
            return False

def main(argv=None):
    rclpy.init(args=argv)
    node = PyMoveIt2Arm()

    if not node.wait_for_js_stable(timeout=12.0):
        node.get_logger().error("等待 /joint_states 超时，退出")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    cur = node.get_current_joints()
    node.get_logger().info(f"当前关节: {[round(x,4) for x in cur]}")

    # 目标（示例：home）
    home = [0.0,0.0,0.0,-1.5708,-0.006,1.6071,0.5603]

    if node.try_moveit(home):
        node.get_logger().info("MoveIt 成功执行")
    else:
        node.get_logger().warn("MoveIt 失败 -> 回退控制器执行")
        ok = node.fallback_controller(home, duration=5.0)
        if ok:
            node.get_logger().info("回退成功")
        else:
            node.get_logger().error("回退也失败，请检查控制器/日志")

    node.destroy_node()
    rclpy.shutdown()
    return 0

if __name__ == '__main__':
    sys.exit(main())
