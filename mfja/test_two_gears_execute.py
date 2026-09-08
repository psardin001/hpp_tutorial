"""Offline checks: all ROS clients and execution calls are replaced by mocks."""

import copy
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import staubli_io
import two_gears_execute as runner


def sample_plan():
    return dict(
        config=dict(
            calibrated=True,
            execution=dict(joint_tolerance_rad=0.03, trajectory_action="/test/action"),
        ),
        configurations=[[0.0] * 6, [0.1] * 6, [0.2] * 6],
        times=[0.0, 1.0, 2.0],
        segments=[
            dict(
                start_index=i,
                end_index=i + 2,
                transition_name=f"transition {i}",
            )
            for i in range(2)
        ],
        gripper=["close", "open"],
    )


class PlanValidation(unittest.TestCase):
    def test_shared_endpoint(self):
        runner.validate_plan(sample_plan())

    def test_rejects_invalid_samples_and_boundaries(self):
        plan = sample_plan()
        for key, value in (
            ("times", [0, 0, 1]),
            ("times", [0, 1, float("nan")]),
            ("configurations", [[float("nan")] * 6] * 3),
            ("configurations", [[0.0] * 7] * 3),
            ("gripper", ["open"]),
            ("gripper", ["open", "invalid"]),
            ("segments", []),
        ):
            with self.subTest(key=key, value=value):
                invalid = copy.deepcopy(plan)
                invalid[key] = value
                with self.assertRaises(ValueError):
                    runner.validate_plan(invalid)
        for key, value in (("start_index", 2), ("end_index", 4)):
            invalid = copy.deepcopy(plan)
            invalid["segments"][1][key] = value
            with self.assertRaises(ValueError):
                runner.validate_plan(invalid)

    def test_uncalibrated_plan_never_imports_ros(self):
        plan = sample_plan()
        plan["config"]["calibrated"] = False
        with patch.dict(sys.modules, {"rclpy": None}):
            with self.assertRaisesRegex(RuntimeError, "Measure the fixture poses"):
                runner.execute(plan, 0)


class Execution(unittest.TestCase):
    def setUp(self):
        self.node = Mock()
        self.ros = Mock()
        self.executor = Mock(return_value=True)
        self.modules = patch.dict(
            sys.modules,
            {
                "rclpy": self.ros,
                "rclpy.node": SimpleNamespace(Node=Mock(return_value=self.node)),
                "hpp_exec": SimpleNamespace(
                    Segment=lambda **kw: SimpleNamespace(pre_actions=[], **kw),
                    execute_segments=self.executor,
                ),
            },
        )
        self.modules.start()
        self.addCleanup(self.modules.stop)

    def test_invalid_index_never_initializes_ros(self):
        for index in (-1, 2):
            with self.assertRaisesRegex(ValueError, "Segment index"):
                runner.execute(sample_plan(), index)
        self.ros.init.assert_not_called()
        self.executor.assert_not_called()

    def test_only_selected_segment_and_gripper(self):
        def send(segments, *args, **kwargs):
            self.assertEqual(len(segments), 1)
            self.assertEqual(segments[0].start_index, 1)
            self.assertEqual(segments[0].end_index, 3)
            return segments[0].pre_actions[0]()

        self.executor.side_effect = send
        with patch.object(runner, "check_position") as check:
            with patch.object(runner, "set_gripper", return_value=True) as gripper:
                runner.execute(sample_plan(), 1)
                gripper.assert_called_once_with(
                    self.node, sample_plan()["config"]["execution"], "open"
                )
        self.assertEqual(check.call_count, 2)
        np.testing.assert_array_equal(check.call_args_list[0].args[2], [0.1] * 6)
        np.testing.assert_array_equal(check.call_args_list[1].args[2], [0.2] * 6)
        self.node.destroy_node.assert_called_once()
        self.ros.shutdown.assert_called_once()

    def test_start_mismatch_blocks_io_and_motion(self):
        with patch.object(runner, "check_position", side_effect=RuntimeError("start")):
            with patch.object(runner, "set_gripper") as gripper:
                with self.assertRaisesRegex(RuntimeError, "start"):
                    runner.execute(sample_plan(), 0)
        gripper.assert_not_called()
        self.executor.assert_not_called()
        self.ros.shutdown.assert_called_once()

    def test_failed_motion_does_not_continue(self):
        self.executor.return_value = False
        with patch.object(runner, "check_position") as check:
            with self.assertRaisesRegex(RuntimeError, "Execution failed"):
                runner.execute(sample_plan(), 0)
        self.executor.assert_called_once()
        check.assert_called_once()

    def test_endpoint_mismatch_is_reported(self):
        with patch.object(
            runner, "check_position", side_effect=[None, RuntimeError("endpoint")]
        ):
            with self.assertRaisesRegex(RuntimeError, "endpoint"):
                runner.execute(sample_plan(), 0)
        self.executor.assert_called_once()
        self.ros.shutdown.assert_called_once()

    def test_position_tolerance(self):
        config = dict(joint_state_topic="/joint_states", joint_tolerance_rad=0.03)
        for position in (None, np.full(6, np.nan), np.full(6, 0.031)):
            with self.subTest(position=position):
                reader = Mock(return_value=position)
                with patch.dict(
                    sys.modules,
                    {"hpp_exec": SimpleNamespace(read_current_configuration=reader)},
                ):
                    with self.assertRaises(RuntimeError):
                        staubli_io.check_position(self.node, config, np.zeros(6))

    def test_gripper_polarity_and_service_failure(self):
        service = SimpleNamespace(
            Request=lambda: SimpleNamespace(module=SimpleNamespace())
        )
        config = dict(gripper_service="/test/io", gripper_pin=0, gripper_settle_s=0)
        client = self.node.create_client.return_value
        client.wait_for_service.return_value = True
        future = client.call_async.return_value
        with patch.dict(
            sys.modules,
            {
                "staubli_msgs.msg": SimpleNamespace(
                    IOModule=SimpleNamespace(VALVE_OUT=2),
                    ServiceReturnCode=SimpleNamespace(SUCCESS=0),
                ),
                "staubli_msgs.srv": SimpleNamespace(WriteSingleIO=service),
            },
        ):
            for mode in ("open", "close"):
                future.result.return_value = SimpleNamespace(
                    code=SimpleNamespace(val=0)
                )
                self.assertTrue(staubli_io.set_gripper(self.node, config, mode))
                request = client.call_async.call_args.args[0]
                self.assertEqual(request.state, mode == "open")
                self.assertEqual((request.module.id, request.pin), (2, 0))
            future.result.return_value = None
            with self.assertRaisesRegex(RuntimeError, "Gripper command failed"):
                staubli_io.set_gripper(self.node, config, "open")
        self.assertEqual(self.node.destroy_client.call_count, 3)


if __name__ == "__main__":
    unittest.main()
