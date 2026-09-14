"""Plan one_gear from the measured arm pose and execute with TOPPRA velocities."""

import argparse
import ctypes
import json
import runpy
from pathlib import Path

import numpy as np
import yaml
from staubli_io import read_joints, robot_connection
from two_gears_execute import (
    execute_all,
    execute_next,  # noqa: F401
    export_plan,
    prompt_execution,
)


def plan_from_start(script, args):
    """Plan a gear exercise from the requested or measured arm position."""
    q_start = None if args.start is None else np.deg2rad(args.start)
    if q_start is not None and not np.isfinite(q_start).all():
        raise ValueError("--start doit contenir six angles finis")
    config = yaml.safe_load(
        Path(__file__).with_name("two_gears_execution.yaml").read_text()
    )
    if args.execute:
        with robot_connection() as node:
            q_start = read_joints(node, config["execution"])
    ctypes.CDLL(None).srand(args.seed)
    namespace = runpy.run_path(
        str(Path(__file__).with_name(script)),
        init_globals={} if q_start is None else {"q_start": q_start},
    )
    export_plan(
        args.plan,
        namespace["robot"],
        namespace["graph"],
        namespace["p2"],
        config,
        q_start,
    )
    print(f"Plan enregistré : {args.plan}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--execute", action="store_true", help="exécution réelle TOPPRA : suivant/tout"
    )
    parser.add_argument("--all", action="store_true", help="enchaîner tout le cycle")
    parser.add_argument(
        "--start", nargs=6, type=float, metavar="DEG", help="départ hors ligne"
    )
    parser.add_argument("--plan", type=Path, default=Path("one-gear.json"))
    parser.add_argument("--seed", type=int, default=97, help="graine de planification")
    args = parser.parse_args()
    if args.execute and args.start is not None:
        parser.error("--execute utilise la position mesurée ; retirer --start")
    if args.all and not args.execute:
        parser.error("--all nécessite --execute")
    plan_from_start("one_gear.py", args)
    if args.execute:
        plan = json.loads(args.plan.read_text())
        if args.all:
            execute_all(plan)
        else:
            prompt_execution(plan)


if __name__ == "__main__":
    try:
        main()
    except (ValueError, RuntimeError) as error:
        raise SystemExit(str(error)) from error
    except (KeyboardInterrupt, EOFError):
        raise SystemExit(
            "Exécution interrompue. Vérifier l'arrêt au pendant."
        ) from None
