from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose


@dataclass
class RobotPose:
    x: float
    y: float
    qx: float
    qy: float
    qz: float
    qw: float


class RobotClient:
    def __init__(self, node: Node, namespace: str):
        self.node = node
        self.namespace = namespace.strip("/")

        self._pose: Optional[RobotPose] = None
        self._goal_handle = None
        self._result_future = None
        self._cancel_requested = False
        self._last_status = "idle"

        amcl_topic = f"/{self.namespace}/amcl_pose"
        action_name = f"/{self.namespace}/navigate_to_pose"

        amcl_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.pose_sub = self.node.create_subscription(
            PoseWithCovarianceStamped,
            amcl_topic,
            self._pose_callback,
            amcl_qos,
        )

        self.nav_client = ActionClient(
            self.node,
            NavigateToPose,
            action_name,
        )

        self.node.get_logger().info(
            f"[{self.namespace}] RobotClient initialized. "
            f"pose_topic={amcl_topic}, action={action_name}"
        )

    def _pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._pose = RobotPose(
            x=p.x,
            y=p.y,
            qx=q.x,
            qy=q.y,
            qz=q.z,
            qw=q.w,
        )

    def has_pose(self) -> bool:
        return self._pose is not None

    def get_pose(self) -> Optional[RobotPose]:
        return self._pose

    def get_status(self) -> str:
        return self._last_status

    def send_goal(
        self,
        x: float,
        y: float,
        qx: float = 0.0,
        qy: float = 0.0,
        qz: float = 0.0,
        qw: float = 1.0,
    ) -> bool:
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().error(
                f"[{self.namespace}] NavigateToPose action server not available"
            )
            self._last_status = "server_unavailable"
            return False

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal_msg.pose = pose

        self.node.get_logger().info(
            f"[{self.namespace}] Sending goal: "
            f"x={x:.3f}, y={y:.3f}, qx={qx:.3f}, qy={qy:.3f}, qz={qz:.3f}, qw={qw:.3f}"
        )

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback,
        )
        send_goal_future.add_done_callback(self._goal_response_callback)
        self._cancel_requested = False
        self._last_status = "goal_sent"
        return True

    def cancel_active_goal(self, timeout_sec: float = 2.0) -> bool:
        if self._goal_handle is None:
            # Goal may still be in goal_sent state before acceptance callback.
            if self._last_status == "goal_sent":
                self._cancel_requested = True
                self._last_status = "canceling"
                self.node.get_logger().info(
                    f"[{self.namespace}] Cancel requested while goal response pending"
                )
                return True

            self.node.get_logger().warn(
                f"[{self.namespace}] No active goal handle to cancel"
            )
            return False

        try:
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)
            self._cancel_requested = True
            self._last_status = "canceling"
            return True
        except Exception as e:
            self.node.get_logger().error(
                f"[{self.namespace}] Failed to request goal cancel: {e}"
            )
            return False

    def _cancel_done_callback(self, future) -> None:
        try:
            response = future.result()
        except Exception as e:
            self.node.get_logger().error(
                f"[{self.namespace}] Cancel response failed: {e}"
            )
            return

        goals_canceling = getattr(response, "goals_canceling", [])
        if goals_canceling:
            self.node.get_logger().info(
                f"[{self.namespace}] Cancel accepted by action server"
            )
        else:
            self.node.get_logger().warn(
                f"[{self.namespace}] Cancel request returned empty goals_canceling"
            )

    def _goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None:
            self.node.get_logger().error(f"[{self.namespace}] Goal response is None")
            self._last_status = "response_error"
            return

        if not goal_handle.accepted:
            self.node.get_logger().warn(f"[{self.namespace}] Goal rejected")
            self._last_status = "rejected"
            return

        self.node.get_logger().info(f"[{self.namespace}] Goal accepted")
        self._goal_handle = goal_handle
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_callback)
        if self._cancel_requested:
            self.node.get_logger().info(
                f"[{self.namespace}] Goal accepted after cancel request, canceling now"
            )
            try:
                cancel_future = goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self._cancel_done_callback)
                self._last_status = "canceling"
            except Exception as e:
                self.node.get_logger().error(
                    f"[{self.namespace}] Failed to cancel just-accepted goal: {e}"
                )
                self._last_status = "executing"
        else:
            self._last_status = "executing"

    def _feedback_callback(self, feedback_msg) -> None:
        pass

    def _result_callback(self, future) -> None:
        result = future.result()
        if result is None:
            self.node.get_logger().error(f"[{self.namespace}] Result future returned None")
            self._last_status = "result_error"
            return

        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info(f"[{self.namespace}] Goal succeeded")
            self._last_status = "succeeded"
        elif status == GoalStatus.STATUS_CANCELED:
            self.node.get_logger().warn(f"[{self.namespace}] Goal canceled")
            self._last_status = "canceled"
        elif status == GoalStatus.STATUS_ABORTED:
            self.node.get_logger().warn(f"[{self.namespace}] Goal aborted")
            self._last_status = "aborted"
        else:
            self.node.get_logger().warn(
                f"[{self.namespace}] Goal finished with raw status={status}"
            )
            self._last_status = f"finished_{status}"

        self._goal_handle = None
        self._result_future = None
        self._cancel_requested = False
