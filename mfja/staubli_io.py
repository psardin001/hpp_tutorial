"""Stäubli gripper IO and measured joint-position checks."""

import time

import numpy as np

JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]


def set_gripper(node, config, mode):
    import rclpy
    from staubli_msgs.msg import IOModule, ServiceReturnCode
    from staubli_msgs.srv import WriteSingleIO

    client = node.create_client(WriteSingleIO, config["gripper_service"])
    try:
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Gripper service unavailable")
        request = WriteSingleIO.Request()
        request.module.id = IOModule.VALVE_OUT
        request.pin = config["gripper_pin"]
        request.state = mode == "open"
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        response = future.result()
        if response is None or response.code.val != ServiceReturnCode.SUCCESS:
            raise RuntimeError("Gripper command failed")
        time.sleep(config["gripper_settle_s"])
    finally:
        node.destroy_client(client)
    return True


def check_position(node, config, target):
    from hpp_exec import read_current_configuration

    current = read_current_configuration(
        node,
        JOINT_NAMES,
        topic=config["joint_state_topic"],
        timeout_sec=10.0,
        require_single_publisher=True,
    )
    if current is None or not np.isfinite(current).all():
        raise RuntimeError("No valid joint state received")
    error = float(np.max(np.abs(current - target)))
    if error > config["joint_tolerance_rad"]:
        raise RuntimeError(
            f"Arm differs from the expected configuration by {error:.4f} rad"
        )
