"""Replay the tutorial reference trajectories in Gazebo."""

import time

import numpy as np
import pinocchio as pin
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from hpp_exec import configs_to_joint_trajectory, read_current_configuration
from init import JOINT_NAMES, ROBOT_IN_WORLD, q_init, q_start, robot
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from solution import (
    GRASP_TRANSITION,
    PICK_TRANSITIONS,
    RELEASE_TRANSITION,
    arm_configs,
    arm_segments,
    arm_times,
    configs,
    segments,
    times,
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_TOPIC = "/staubli1/joint_trajectory"
GRIPPER_TOPIC = "/staubli1/gripper_joint_trajectory"
JOINT_STATE_TOPIC = "/staubli1/joint_states"
BOX_NAME = "room315_payload_box"
WORLD_NAME = "room_315_only"


def duration_message(seconds):
    message = Duration()
    message.sec = int(seconds)
    message.nanosec = int((seconds - message.sec) * 1e9)
    return message


def sleep_with_spin(node, duration):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        rclpy.spin_once(
            node,
            timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())),
        )


def wait_for_subscriber(node, publisher, topic, timeout=5.0):
    deadline = time.monotonic() + timeout
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() == 0:
        raise RuntimeError(f"no subscriber on {topic}")


def call_service(node, client, request, label, timeout=5.0):
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(f"{label} service is unavailable")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise RuntimeError(f"{label} timed out")
    response = future.result()
    if response is None:
        raise RuntimeError(f"{label} returned no result")
    if not response.success:
        raise RuntimeError(f"{label} failed: {response}")
    return response


def box_world_pose(configuration):
    rank = robot.rankInConfiguration["box/root_joint"]
    quaternion = pin.Quaternion(np.asarray(configuration[rank + 3 : rank + 7]))
    box_in_robot = pin.SE3(
        quaternion.matrix(), np.asarray(configuration[rank : rank + 3])
    )
    placement = ROBOT_IN_WORLD * box_in_robot
    quaternion = pin.Quaternion(placement.rotation).coeffs()

    pose = Pose()
    pose.position.x = float(placement.translation[0])
    pose.position.y = float(placement.translation[1])
    pose.position.z = float(placement.translation[2])
    pose.orientation.x = float(quaternion[0])
    pose.orientation.y = float(quaternion[1])
    pose.orientation.z = float(quaternion[2])
    pose.orientation.w = float(quaternion[3])
    return pose


class GazeboExecutor:
    def __init__(self, node):
        self.node = node
        self.arm_publisher = node.create_publisher(JointTrajectory, ARM_TOPIC, 10)
        self.gripper_publisher = node.create_publisher(
            JointTrajectory, GRIPPER_TOPIC, 10
        )
        service_prefix = f"/world/{WORLD_NAME}"
        self.pose_client = node.create_client(
            SetEntityPose, f"{service_prefix}/set_pose"
        )

        wait_for_subscriber(node, self.arm_publisher, ARM_TOPIC)
        wait_for_subscriber(node, self.gripper_publisher, GRIPPER_TOPIC)

    def set_box_request(self, configuration):
        request = SetEntityPose.Request()
        request.entity.name = BOX_NAME
        request.entity.type = Entity.MODEL
        request.pose = box_world_pose(configuration)
        return request

    def reset_box(self):
        call_service(
            self.node,
            self.pose_client,
            self.set_box_request(q_init),
            f"reset {BOX_NAME}",
        )

    def command_gripper(self, positions, label):
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            "gripper_left_finger_joint",
            "gripper_right_finger_joint",
        ]
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = duration_message(0.2)
        trajectory.points.append(point)
        self.gripper_publisher.publish(trajectory)
        rclpy.spin_once(self.node, timeout_sec=0.05)
        print(f"gripper action: {label}")
        sleep_with_spin(self.node, 0.5)
        return True

    def open_gripper(self):
        return self.command_gripper([0.0025, 0.0025], "open")

    def close_gripper(self):
        return self.command_gripper([0.0, 0.0], "close")

    def execute_trajectory(self, trajectory_configs, trajectory_times):
        start_time = trajectory_times[0]
        local_times = [value - start_time for value in trajectory_times]
        trajectory = configs_to_joint_trajectory(
            trajectory_configs,
            local_times,
            JOINT_NAMES,
            joint_indices=list(range(6)),
        )
        self.arm_publisher.publish(trajectory)
        rclpy.spin_once(self.node, timeout_sec=0.05)

        pending_pose = None
        started = time.monotonic()
        duration = local_times[-1]
        while time.monotonic() - started < duration:
            elapsed = time.monotonic() - started
            index = max(
                0,
                min(
                    int(np.searchsorted(local_times, elapsed, side="right")) - 1,
                    len(trajectory_configs) - 1,
                ),
            )
            if pending_pose is None or pending_pose.done():
                pending_pose = self.pose_client.call_async(
                    self.set_box_request(trajectory_configs[index])
                )
            rclpy.spin_once(self.node, timeout_sec=0.02)

        call_service(
            self.node,
            self.pose_client,
            self.set_box_request(trajectory_configs[-1]),
            "set final box pose",
        )

    def execute_segments(
        self,
        execution_segments,
        trajectory_configs,
        trajectory_times,
        pre_actions=None,
    ):
        pre_actions = pre_actions or {}
        for index, segment in enumerate(execution_segments):
            for action in [
                *segment.pre_actions,
                *pre_actions.get(segment.transition_name, []),
            ]:
                if not action():
                    return False

            segment_configs = trajectory_configs[
                segment.start_index : segment.end_index
            ]
            segment_times = trajectory_times[segment.start_index : segment.end_index]
            if len(segment_configs) >= 2:
                print(
                    f"Gazebo segment {index}: {segment.transition_name} "
                    f"({segment.duration:.2f} s)"
                )
                self.execute_trajectory(segment_configs, segment_times)

            for action in segment.post_actions:
                if not action():
                    return False
        return True


def main():
    rclpy.init()
    node = Node("hpp_mfja_manipulation_gazebo")
    try:
        executor = GazeboExecutor(node)
        executor.reset_box()
        current = read_current_configuration(
            node,
            JOINT_NAMES,
            JOINT_STATE_TOPIC,
            strip_prefix=True,
            require_single_publisher=True,
        )
        if current is None:
            raise RuntimeError(f"no configuration received on {JOINT_STATE_TOPIC}")

        if np.max(np.abs(current - q_start[:6])) < 0.05:
            print("Executing the collision-checked arm motion to the work pose")
            executor.open_gripper()
            executor.execute_segments(arm_segments, arm_configs, arm_times)
        elif np.max(np.abs(current - q_init[:6])) >= 0.05:
            raise RuntimeError("Gazebo robot is neither at q_start nor q_init")

        actions = {
            PICK_TRANSITIONS[0]: [executor.open_gripper],
            GRASP_TRANSITION: [executor.close_gripper],
            RELEASE_TRANSITION: [executor.open_gripper],
        }
        if not executor.execute_segments(segments, configs, times, actions):
            raise RuntimeError("Gazebo execution failed")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
