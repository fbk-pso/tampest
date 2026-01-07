# Copyright (C) 2024-2025 PSO Unit, Fondazione Bruno Kessler
# This file is part of TAMPEST.
#
# TAMPEST is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# TAMPEST is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
#

import argparse
from unified_planning.shortcuts import *
from tampest.motion.motion_planning_data import (
    SupportedTopologicalRefinement,
    SupportedPlanner,
)
from benchmarks import get_problem

env = get_environment()
env.factory.add_engine("tampest", "tampest.engine", "TampestEngine")
env.factory.add_meta_engine("tamp", "tampest.meta_engine", "TampMetaEngine")
env.credits_stream = None


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
    parser.add_argument("--tp", required=False, type=str)
    parser.add_argument("--mp", required=False, type=str)
    parser.add_argument("--tr", required=False, type=str)
    parser.add_argument("--mp_time", required=False, type=float)

    args, _ = parser.parse_known_args()

    domain = args.domain
    dim = args.dim if args.dim else "2D"
    d = args.d if args.d else 1
    c = args.c if args.c is not None else None
    n_robots = args.r if args.r else 1
    robot_capacity = args.capacity if args.capacity else None
    kit_size = args.kit_size if args.kit_size else None
    n_kit = args.n_kit if args.n_kit else None
    n_pallets = args.n_pallets if args.n_pallets else None
    n_drivers = args.n_drivers if args.n_drivers else None
    n_tiles = args.n_tiles if args.n_tiles else None
    n_colors = args.n_colors if args.n_colors else None

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

    tr = SupportedTopologicalRefinement.NONE  # ['none' 'unreach' 'obs' 'all']
    if args.tr == "none":
        tr = SupportedTopologicalRefinement.NONE
    elif args.tr == "unreach":
        tr = SupportedTopologicalRefinement.UNREACH
    elif args.tr == "obs":
        tr = SupportedTopologicalRefinement.OBS
    elif args.tr == "all":
        tr = SupportedTopologicalRefinement.ALL
    else:
        raise NotImplementedError

    max_radius_bound = False
    incremental = True
    step_horizon = 50
    interpolate = False
    simplified = False
    tolerance = 0.0
    distance = 5.0

    mp_time = 5.0
    if args.mp_time:
        mp_time = args.mp_time

    problem = get_problem(
        domain,
        dim,
        d,
        c=c,
        n_robots=n_robots,
        capacity=robot_capacity,
        kit_size=kit_size,
        n_kit=n_kit,
        n_pallets=n_pallets,
        n_drivers=n_drivers,
        n_tiles=n_tiles,
        n_colors=n_colors,
    )

    if tp == "tampest":
        with OneshotPlanner(
            name="tampest",
            params={
                "incremental": incremental,
                "step_horizon": step_horizon,
                "motion_planning_time": mp_time,
                "interpolate": interpolate,
                "simplified": simplified,
                "tolerance": tolerance,
                "distance": distance,
                "motion_planner": mp,
                "topological_refinement": tr,
                "max_radius_bound": max_radius_bound,
            },
        ) as planner:
            res = planner.solve(problem, output_stream=sys.stdout)
    else:
        with OneshotPlanner(
            name=f"tamp[{tp}]",
            params={
                "motion_planning_time": mp_time,
                "interpolate": interpolate,
                "simplified": simplified,
                "tolerance": tolerance,
                "distance": distance,
                "motion_planner": mp,
                "topological_refinement": tr,
            },
        ) as planner:
            res = planner.solve(problem)

    if res.plan:
        print("SOLUTION FOUND!")
        print(res)
    else:
        print("NO SOLUTION FOUND!")


if __name__ == "__main__":
    main()
