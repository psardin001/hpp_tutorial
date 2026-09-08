"""Offline checks: all ROS clients and execution calls are replaced by mocks."""

import copy
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import two_gears_execute as runner


def sample_plan():
    return dict(
        config=dict(
            calibrated=True,
            execution=dict(joint_tolerance_rad=0.03, trajectory_action="/test/action"),
        ),
        configurations=[[0.0] * 6, [0.1] * 6, [0.2] * 6],
        joint_indices=list(range(6)),
        times=[0.0, 1.0, 2.0],
        segments=[
            dict(
                start_index=i,
                end_index=i + 2,
                transition_name=f"transition {i}",
                containing_state="test state",
                gripper=mode,
            )
            for i, mode in enumerate(("close", "open"))
        ],
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
            ("joint_indices", [0] * 6),
            ("joint_indices", list(range(1, 7))),
            ("segments", []),
        ):
            with self.subTest(key=key, value=value):
                invalid = copy.deepcopy(plan)
                invalid[key] = value
                with self.assertRaises(ValueError):
                    runner.validate_plan(invalid)
        for key, value in (("start_index", 2), ("gripper", "invalid")):
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


class PathValidation(unittest.TestCase):
    def setUp(self):
        self.path = Mock()
        self.leaves = [Mock(), Mock(), Mock()]
        for leaf, duration in zip(self.leaves, (1.0, 2.0, 1.0)):
            leaf.length.return_value = duration
        flat = Mock()
        flat.numberPaths.return_value = len(self.leaves)
        flat.pathAtRank.side_effect = self.leaves
        modules = patch.dict(
            sys.modules,
            {"pyhpp.core.path": SimpleNamespace(Vector=Mock(return_value=flat))},
        )
        modules.start()
        self.addCleanup(modules.stop)
        self.graph = Mock()
        self.edges = [Mock(), Mock(), Mock()]
        self.graph.transitionAtParam.side_effect = self.edges
        for edge in self.edges:
            edge.pathValidation.return_value.validate.return_value = (True, None, None)

    def test_checks_each_timed_subpath_with_its_transition(self):
        runner.validate_path(self.path, self.graph)
        self.assertEqual(
            [call.args for call in self.graph.transitionAtParam.call_args_list],
            [(self.path, 0.5), (self.path, 2.0), (self.path, 3.5)],
        )
        for edge, leaf in zip(self.edges, self.leaves):
            edge.pathValidation.return_value.validate.assert_called_once_with(
                leaf, False
            )

    def test_rejects_invalid_middle_subpath(self):
        self.edges[1].name.return_value = "placement"
        self.edges[1].pathValidation.return_value.validate.return_value = (
            False,
            None,
            "collision",
        )
        with self.assertRaisesRegex(RuntimeError, "subpath 1 .*placement.*collision"):
            runner.validate_path(self.path, self.graph)
        self.edges[2].pathValidation.assert_not_called()

    def test_native_validation_exception_propagates(self):
        self.edges[0].pathValidation.return_value.validate.side_effect = ValueError(
            "wrong argument size"
        )
        with self.assertRaisesRegex(ValueError, "wrong argument size"):
            runner.validate_path(self.path, self.graph)

    def test_planning_validates_before_sampling(self):
        sample = Mock()
        with patch.dict(
            sys.modules,
            {
                "hpp_exec": SimpleNamespace(segments_from_graph=sample),
                "staubli_scene": SimpleNamespace(read_room=lambda _: ("room", [])),
                "two_gears": SimpleNamespace(solve=lambda _: self.path),
            },
        ):
            with patch.object(
                runner, "build_scene", return_value=(None, self.graph, None)
            ):
                with patch.object(
                    runner, "validate_path", side_effect=RuntimeError("invalid path")
                ) as validate:
                    with self.assertRaisesRegex(RuntimeError, "invalid path"):
                        runner.make_plan({}, [0.0] * 6, "unused")
        validate.assert_called_once_with(self.path, self.graph)
        sample.assert_not_called()


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
                    Segment=lambda start, end, **kw: SimpleNamespace(
                        start_index=start, end_index=end, pre_actions=[], **kw
                    ),
                    execute_segments=self.executor,
                ),
            },
        )
        self.modules.start()
        self.addCleanup(self.modules.stop)

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
                        runner.check_position(self.node, config, np.zeros(6))

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
                self.assertTrue(runner.set_gripper(self.node, config, mode))
                request = client.call_async.call_args.args[0]
                self.assertEqual(request.state, mode == "open")
                self.assertEqual((request.module.id, request.pin), (2, 0))
            future.result.return_value = None
            with self.assertRaisesRegex(RuntimeError, "Gripper command failed"):
                runner.set_gripper(self.node, config, "open")
        self.assertEqual(self.node.destroy_client.call_count, 3)


if __name__ == "__main__":
    unittest.main()
