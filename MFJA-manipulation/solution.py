"""Reference solution for the Room 315 Stäubli exercise."""

import numpy as np
from hpp_exec import print_segments, segments_by_transition, segments_from_graph
from init import (
    BOX_HANDLE,
    GRIPPER,
    display,  # noqa: F401
    graph,
    problem,
    q_goal,
    q_init,
    q_start,
    robot,
)
from pyhpp.core import SimpleTimeParameterization
from pyhpp.core import path as hpp_path
from pyhpp.manipulation import EnforceTransitionSemantic, TransitionPlanner

PICK_TRANSITIONS = [
    f"{GRIPPER} > {BOX_HANDLE} | f_{step}" for step in ("01", "12", "23", "34")
]
TRANSFER_TRANSITION = "Loop | 0-0"
RELEASE_TRANSITIONS = [
    f"{GRIPPER} < {BOX_HANDLE} | 0-0_{step}" for step in ("43", "32", "21", "10")
]
GRASP_TRANSITION = PICK_TRANSITIONS[2]
RELEASE_TRANSITION = RELEASE_TRANSITIONS[2]


def generate_target_chains(q_free, attempts=80, max_chains=8, preferred=None):
    """Generate valid pregrasp-to-grasp waypoint chains."""
    shooter = problem.configurationShooter()
    box_rank = robot.rankInConfiguration["box/root_joint"]
    candidates = []

    for attempt in range(attempts):
        seed = np.asarray(shooter.shoot()).flatten()
        seed[box_rank : box_rank + 7] = q_free[box_rank : box_rank + 7]
        if preferred is not None and attempt % 3 == 0:
            seed[:6] = preferred[:6]
        elif attempt % 3 == 1:
            seed[:6] = q_free[:6]

        source = q_free
        chain = []
        for index, transition_name in enumerate(PICK_TRANSITIONS):
            transition = graph.getTransition(transition_name)
            initializer = seed if index == 0 else source
            success, target, _ = graph.generateTargetConfig(
                transition, source, initializer
            )
            if not success:
                break

            target = np.asarray(target).flatten()
            valid, _ = transition.pathValidation().validateConfiguration(target)
            if not valid:
                break
            chain.append(target)
            source = target

        if len(chain) != len(PICK_TRANSITIONS):
            continue
        if any(
            np.max(np.abs(chain[-1] - candidate[-1])) < 1e-5 for candidate in candidates
        ):
            continue
        candidates.append(chain)
        if len(candidates) == max_chains:
            break

    if not candidates:
        raise RuntimeError("failed to generate a valid grasp waypoint chain")
    return candidates


def plan_transition(planner, transition_name, q_start, q_end):
    transition = graph.getTransition(transition_name)
    valid, report = transition.pathValidation().validateConfiguration(q_end)
    if not valid:
        raise RuntimeError(f"invalid target for {transition_name}: {report}")

    planner.setTransition(transition)
    success, path, report = planner.directPath(q_start, q_end, True)
    if success:
        return path

    goals = np.zeros((1, robot.configSize()), order="F")
    goals[0, :] = q_end
    try:
        return planner.planPath(q_start, goals, True)
    except Exception as error:
        raise RuntimeError(f"failed to plan {transition_name}: {report}") from error


def concatenate(paths):
    result = hpp_path.Vector(robot.configSize(), robot.numberDof())
    for path in paths:
        result.appendPath(path)
    return result


def plan_arm_motion():
    planner = TransitionPlanner(problem)
    return concatenate([plan_transition(planner, "Loop | f", q_start, q_init)])


def plan_pick_and_place():
    source_chains = generate_target_chains(q_init)
    destination_chains = generate_target_chains(q_goal, preferred=source_chains[0][-1])
    planner = TransitionPlanner(problem)
    planner.maxIterations(2000)
    planner.timeOut(20)

    last_error = None
    for source_chain in source_chains:
        for destination_chain in destination_chains:
            try:
                paths = []
                current = q_init
                for transition_name, target in zip(PICK_TRANSITIONS, source_chain):
                    paths.append(
                        plan_transition(planner, transition_name, current, target)
                    )
                    current = target

                paths.append(
                    plan_transition(
                        planner,
                        TRANSFER_TRANSITION,
                        current,
                        destination_chain[-1],
                    )
                )
                current = destination_chain[-1]

                release_targets = [*reversed(destination_chain[:-1]), q_goal]
                for transition_name, target in zip(
                    RELEASE_TRANSITIONS, release_targets
                ):
                    paths.append(
                        plan_transition(planner, transition_name, current, target)
                    )
                    current = target
                return concatenate(paths)
            except RuntimeError as error:
                last_error = error

    raise RuntimeError(f"failed to plan a target pair: {last_error}") from last_error


def time_parameterize(path):
    semantic = EnforceTransitionSemantic(problem)
    path = semantic.optimize(path)

    optimizer = SimpleTimeParameterization(problem)
    optimizer.order = 2
    optimizer.safety = 0.5
    optimizer.maxAcceleration = 0.5
    return optimizer.optimize(path)


print("Planning the free arm motion...")
p_arm = plan_arm_motion()
p_arm_timed = time_parameterize(p_arm)
arm_configs, arm_times, arm_segments = segments_from_graph(
    p_arm_timed, graph, n_per_unit=20, min_samples=30
)

print("Planning the classic pick and place...")
p_pick_place = plan_pick_and_place()
p_timed = time_parameterize(p_pick_place)
configs, times, segments = segments_from_graph(
    p_timed, graph, n_per_unit=20, min_samples=80
)
segments_by_name = segments_by_transition(segments)
if len(segments_by_name[GRASP_TRANSITION]) != 1:
    raise RuntimeError("expected one grasp execution segment")
if len(segments_by_name[RELEASE_TRANSITION]) != 1:
    raise RuntimeError("expected one release execution segment")

print(f"arm trajectory duration: {p_arm_timed.length():.2f} s")
print(f"pick-and-place duration: {p_timed.length():.2f} s")
print_segments(segments)
