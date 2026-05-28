# Copyright (C) 2024-2026 PSO Unit, Fondazione Bruno Kessler
# This file is part of TAMPEST.
#
# TAMPEST is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# TAMPEST is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
#

import sys
import argparse
from unified_planning.shortcuts import *
from tampest.motion.motion_planning_data import (
    SupportedTopologicalRefinement,
    SupportedPlanner,
)
from benchmarks import get_problem

env = get_environment()
env.factory.add_engine("tampest", "tampest.engine", "TampestEngine")
env.factory.add_engine("cpse", "cpse", "CPSE")
env.factory.add_meta_engine("tamp", "tampest.meta_engine", "TampMetaEngine")
env.factory.add_meta_engine("samp", "tampest.meta_engine_samp", "SampMetaEngine")
env.credits_stream = None

# Domains that have a scheduling (samp) variant
_SAMP_DOMAINS = {"tdoors", "kitting", "majsp", "jsp", "logistics", "navigation"}
# Domains that support 3D
_THREED_DOMAINS = {"maze", "rover"}
# Flags that are only consumed by samp engines / samp problem constructors
_SAMP_ONLY_FLAGS = [
    ("--use_fluents", "use_fluents"),
    ("--opt", "opt"),
]
# Params required by specific domains (flag_name, attr_name)
_REQUIRED_BY_DOMAIN = {
    "kitting":   [("--kit_size", "kit_size"), ("--n_kit", "n_kit")],
    "majsp":     [("--n_pallets", "n_pallets")],
    "jsp":       [("--n_pallets", "n_pallets")],
    "driverlog": [("--n_drivers", "n_drivers")],
    "floortile": [("--n_tiles", "n_tiles"), ("--n_colors", "n_colors")],
}


def _validate(args, tp, domain, dim):
    errors = []
    is_samp = "samp" in tp

    # engine–domain compatibility
    if is_samp and domain not in _SAMP_DOMAINS:
        errors.append(
            f"'{tp}' is a scheduling engine but '{domain}' has no scheduling variant "
            f"(samp-compatible domains: {', '.join(sorted(_SAMP_DOMAINS))})"
        )

    # 3D availability
    if dim == "3D" and domain not in _THREED_DOMAINS:
        errors.append(
            f"--dim 3D is only supported for {', '.join(sorted(_THREED_DOMAINS))} "
            f"(got --domain {domain})"
        )

    # required params per domain
    for flag, attr in _REQUIRED_BY_DOMAIN.get(domain, []):
        if getattr(args, attr) is None:
            errors.append(f"--domain {domain} requires {flag}")

    # samp-only flags passed to a non-samp engine
    if not is_samp:
        active = [
            flag for flag, attr in _SAMP_ONLY_FLAGS
            if getattr(args, attr)  # True for bools, non-None/non-zero for floats
        ]
        if active:
            errors.append(
                f"{', '.join(active)} {'is' if len(active) == 1 else 'are'} only "
                f"meaningful with a samp engine (--tp samp[...])"
            )

    if errors:
        for e in errors:
            print(f"error: {e}", file=sys.stderr)
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--domain", required=True, type=str)
    parser.add_argument("--dim", required=False, type=str)
    parser.add_argument("--d", required=False, type=int)
    parser.add_argument("--c", required=False, type=int)
    parser.add_argument("--r", required=False, type=int)
    parser.add_argument("--capacity", required=False, type=int)
    parser.add_argument("--kit_size", required=False, type=int)
    parser.add_argument("--n_kit", required=False, type=int)
    parser.add_argument("--n_pallets", required=False, type=int)
    parser.add_argument("--n_drivers", required=False, type=int)
    parser.add_argument("--n_tiles", required=False, type=int)
    parser.add_argument("--n_colors", required=False, type=int)
    parser.add_argument("--n_shelves", required=False, type=int, default=2)
    parser.add_argument("--n_components", required=False, type=int, default=1)
    parser.add_argument("--tp", required=False, type=str)
    parser.add_argument("--mp", required=False, type=str)
    parser.add_argument("--tr", required=False, type=str)
    parser.add_argument("--mp_time", required=False, type=float)
    parser.add_argument("--use_external_locations", action="store_true")
    parser.add_argument("--use_fluents", action="store_true")
    parser.add_argument("--opt", action="store_true")

    args, _ = parser.parse_known_args()

    domain = args.domain
    dim = args.dim if args.dim else "2D"
    d = args.d if args.d is not None else 1
    c = args.c if args.c is not None else 0
    n_robots = args.r if args.r else 1
    robot_capacity = args.capacity if args.capacity else None
    kit_size = args.kit_size if args.kit_size else None
    n_kit = args.n_kit if args.n_kit else None
    n_pallets = args.n_pallets if args.n_pallets else None
    n_drivers = args.n_drivers if args.n_drivers else None
    n_tiles = args.n_tiles if args.n_tiles else None
    n_colors = args.n_colors if args.n_colors else None
    n_shelves = args.n_shelves
    n_components = args.n_components
    use_external_locations = args.use_external_locations
    tp = "tampest"  # ['fast-downward' 'enhsp' 'tamer' 'tampest']
    if args.tp:
        tp = args.tp

    mp = SupportedPlanner.STRRTstar
    if args.mp == "RRT":
        mp = SupportedPlanner.RRT
    elif args.mp == "LazyRRT":
        mp = SupportedPlanner.LazyRRT
    elif args.mp == "RRTConnect":
        mp = SupportedPlanner.RRTConnect
    elif args.mp == "STRRTstar":
        mp = SupportedPlanner.STRRTstar

    tr = SupportedTopologicalRefinement.ALL  # ['none' 'unreach' 'obs' 'all']
    if args.tr is None or args.tr == "all":
        tr = SupportedTopologicalRefinement.ALL
    elif args.tr == "none":
        tr = SupportedTopologicalRefinement.NONE
    elif args.tr == "unreach":
        tr = SupportedTopologicalRefinement.UNREACH
    elif args.tr == "obs":
        tr = SupportedTopologicalRefinement.OBS
    else:
        raise NotImplementedError(f"Unknown --tr: {args.tr!r}")

    _validate(args, tp, domain, dim)

    max_radius_bound = False
    incremental = True
    step_horizon = 50
    interpolate = False
    simplified = False
    distance = 5.0 if mp == SupportedPlanner.STRRTstar else None

    mp_time = args.mp_time if args.mp_time else 5.0

    problem = get_problem(
        domain,
        dim,
        d,
        samp="samp" in tp,
        c=c,
        n_robots=n_robots,
        capacity=robot_capacity,
        kit_size=kit_size,
        n_kit=n_kit,
        n_pallets=n_pallets,
        n_drivers=n_drivers,
        n_tiles=n_tiles,
        n_colors=n_colors,
        n_shelves=n_shelves,
        n_components=n_components,
        use_external_locations=use_external_locations,
        use_fluents=args.use_fluents,
        use_resources=False,
        add_metric=args.opt,
    )

    base_mp_params = {
        "motion_planning_time": mp_time,
        "interpolate": interpolate,
        "simplified": simplified,
        "distance": distance,
        "motion_planner": mp,
        "topological_refinement": tr,
        "max_radius_bound": max_radius_bound,
    }

    if tp == "tampest":
        params = {
            **base_mp_params,
            "incremental": incremental,
            "step_horizon": step_horizon,
        }
        with OneshotPlanner(name="tampest", params=params) as planner:
            res = planner.solve(problem, output_stream=sys.stdout)
    elif "samp" in tp:
        params = {
            **base_mp_params,
            "use_fluents": args.use_fluents,
            "enable_reachable_generalization": True,
            "enable_movable_generalization": True,
        }
        with OneshotPlanner(name=tp, params=params) as planner:
            res = planner.solve(problem)
    else:
        with OneshotPlanner(name=f"tamp[{tp}]", params=base_mp_params) as planner:
            res = planner.solve(problem)

    if res.plan:
        print("SOLUTION FOUND!")
        print(res)
        # from tampest.motion.plotting import plot_plan
        # plot_plan(problem.all_objects, res)
    else:
        print("NO SOLUTION FOUND!")

    sys.stdout.flush()


if __name__ == "__main__":
    main()
