"""Planning validation and export of hpp-exec segments."""

import json
import sys
import unittest
from dataclasses import asdict
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import Mock, patch

import numpy as np
import two_gears as planner
from hpp_exec import Segment


class PathValidation(unittest.TestCase):
    def setUp(self):
        self.path = Mock()
        self.leaves = [Mock(), Mock(), Mock()]
        for leaf, duration in zip(self.leaves, (1.0, 2.0, 1.0)):
            leaf.length.return_value = duration
        flat = Mock()
        flat.numberPaths.return_value = len(self.leaves)
        flat.pathAtRank.side_effect = self.leaves
        vector = patch.object(planner, "Vector", return_value=flat)
        vector.start()
        self.addCleanup(vector.stop)
        self.graph = Mock()
        self.edges = [Mock(), Mock(), Mock()]
        self.graph.transitionAtParam.side_effect = self.edges
        for edge in self.edges:
            edge.pathValidation.return_value.validate.return_value = (True, None, None)

    def test_checks_each_timed_subpath_with_its_transition(self):
        planner.validate_path(self.path, self.graph)
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
            planner.validate_path(self.path, self.graph)
        self.edges[2].pathValidation.assert_not_called()

    def test_native_validation_exception_propagates(self):
        self.edges[0].pathValidation.return_value.validate.side_effect = ValueError(
            "wrong argument size"
        )
        with self.assertRaisesRegex(ValueError, "wrong argument size"):
            planner.validate_path(self.path, self.graph)

    def test_invalid_timed_path_blocks_export(self):
        self.edges[0].pathValidation.return_value.validate.return_value = (
            False,
            None,
            "collision",
        )
        problem = Mock()
        problem.constraintGraph.return_value = self.graph
        with TemporaryDirectory() as directory:
            target = Path(directory) / "plan.json"
            args = [
                "two_gears.py",
                str(target),
                "--mfja-root",
                directory,
                "--q-start",
                *["0"] * 6,
            ]
            with (
                patch.object(sys, "argv", args),
                patch.object(planner, "read_room", return_value=("room", [0.0] * 6)),
                patch.object(
                    planner, "build_problem", return_value=(None, self.graph, problem)
                ),
                patch.object(planner, "StatesPathFinder"),
                patch.object(planner, "GraphRandomShortcut"),
                patch.object(planner, "Toppra") as toppra,
                patch.object(planner, "export_plan") as export,
            ):
                toppra.return_value.optimize.return_value = self.path
                with self.assertRaisesRegex(RuntimeError, "Invalid timed subpath"):
                    planner.main()
            export.assert_not_called()
            self.assertFalse(target.exists())


class Export(unittest.TestCase):
    def test_segments_round_trip_and_arm_joint_order(self):
        configs = np.arange(24.0).reshape(3, 8)
        indices = [2, 0, 7, 1, 6, 3]
        robot = Mock()
        robot.rankInConfiguration = dict(
            zip(("staubli/" + name for name in planner.JOINT_NAMES), indices)
        )
        segments = [
            Segment(0, 2, transition_name="grasp", end_time=1.0),
            Segment(1, 3, transition_name="release", start_time=1.0, end_time=2.0),
        ]
        graph = Mock()
        graph.getContainingNode.side_effect = [
            "staubli/tool0_gripper grasps gear_42_1/stud",
            "free",
        ]
        with TemporaryDirectory() as directory:
            target = Path(directory) / "plan.json"
            with (
                patch.object(
                    planner,
                    "segments_from_graph",
                    return_value=(configs, [0.0, 1.0, 2.0], segments),
                ),
                patch.object(planner, "print_segments"),
            ):
                exported = planner.export_plan(
                    target,
                    robot,
                    graph,
                    Mock(),
                    {"calibrated": False},
                    configs[0, indices],
                )
            saved = json.loads(target.read_text())
        self.assertEqual([Segment(**item) for item in saved["segments"]], segments)
        self.assertEqual([asdict(s) for s in exported], saved["segments"])
        np.testing.assert_array_equal(saved["configurations"], configs[:, indices])
        self.assertEqual(saved["gripper"], ["close", "open"])
        self.assertFalse(saved["config"]["calibrated"])

    def test_changed_start_preserves_existing_file(self):
        robot = Mock()
        robot.rankInConfiguration = dict(
            zip(("staubli/" + name for name in planner.JOINT_NAMES), range(6))
        )
        with TemporaryDirectory() as directory:
            target = Path(directory) / "plan.json"
            target.write_text("existing plan")
            with patch.object(
                planner,
                "segments_from_graph",
                return_value=([np.ones(6), np.ones(6)], [0.0, 1.0], [Segment(0, 2)]),
            ):
                with self.assertRaisesRegex(RuntimeError, "requested arm start"):
                    planner.export_plan(target, robot, Mock(), Mock(), {}, np.zeros(6))
            self.assertEqual(target.read_text(), "existing plan")


if __name__ == "__main__":
    unittest.main()
