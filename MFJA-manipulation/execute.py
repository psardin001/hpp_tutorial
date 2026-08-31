"""Execute the Room 315 pick and place on the Stäubli."""

import argparse
import time

import numpy as np
import rclpy
from hpp_exec import execute_segments, read_current_configuration
from init import JOINT_NAMES, q_init
from rclpy.node import Node
from solution import (
    GRASP_TRANSITION,
    RELEASE_TRANSITION,
    configs,
    segments,
    times,
)

ACTION_TOPIC = "/manipulator_controller/joint_trajectory_action"
JOINT_STATE_TOPIC = "/joint_states"


def sleep_with_spin(node, duration):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        rclpy.spin_once(
            node,
            timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())),
        )


class StaubliGripper:
    def __init__(self, node):
        from staubli_msgs.msg import IOModule, ServiceReturnCode
        from staubli_msgs.srv import WriteSingleIO

        self.node = node
        self.IOModule = IOModule
        self.ServiceReturnCode = ServiceReturnCode
        self.WriteSingleIO = WriteSingleIO
        self.client = node.create_client(WriteSingleIO, "/io_interface/write_single_io")
        if not self.client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Stäubli gripper IO service is unavailable")

    def command(self, state, label):
        request = self.WriteSingleIO.Request()
        request.module.id = self.IOModule.VALVE_OUT
        request.pin = 0
        request.state = state
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        response = future.result()
        if response is None or response.code.val != self.ServiceReturnCode.SUCCESS:
            print(f"Stäubli gripper {label} failed")
            return False
        print(f"Stäubli gripper pre-action: {label}")
        sleep_with_spin(self.node, 0.5)
        return True

    def open_gripper(self):
        return self.command(True, "open")

    def close_gripper(self):
        return self.command(False, "close")


def execute_on_robot():
    rclpy.init()
    node = Node("hpp_mfja_manipulation_staubli")
    try:
        current = read_current_configuration(
            node,
            JOINT_NAMES,
            JOINT_STATE_TOPIC,
            strip_prefix=True,
            require_single_publisher=True,
        )
        if current is None:
            raise RuntimeError(f"no configuration received on {JOINT_STATE_TOPIC}")
        error = float(np.max(np.abs(current - q_init[:6])))
        if error > 0.03:
            raise RuntimeError(
                f"arm configuration is {error:.3f} rad from q_init; "
                "position the robot at the validated start configuration"
            )

        gripper = StaubliGripper(node)
        actions = {
            GRASP_TRANSITION: [gripper.close_gripper],
            RELEASE_TRANSITION: [gripper.open_gripper],
        }
        success = execute_segments(
            segments,
            configs,
            times,
            JOINT_NAMES,
            joint_indices=list(range(6)),
            controller_topic=ACTION_TOPIC,
            pre_actions_by_transition=actions,
        )
        if not success:
            raise RuntimeError("Stäubli execution failed")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--confirm-real", action="store_true")
    args = parser.parse_args()
    if not args.confirm_real:
        raise RuntimeError(
            "pass --confirm-real after completing the instructor preflight"
        )
    execute_on_robot()


if __name__ == "__main__":
    main()
