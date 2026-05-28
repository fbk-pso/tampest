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

import math
import os
import random
from unified_planning.model.motion import SchedulingMotionProblem
from unified_planning.shortcuts import *
import yaml
from benchmarks.navigation.generator import SetupGenerator
from benchmarks.scheduling_utils import (
    activity_precondition_constraint,
    compute_time_using_trapezoidal_velocity_profile,
)

random.seed(23)

SETUP_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "configs/2D")


class Navigation:

    def __init__(self, tmpdirname=None) -> None:
        if tmpdirname:
            self.dirname = tmpdirname
        else:
            self.dirname = SETUP_PATH

        self.robot_footprint = [(-1.0, 0.5), (1.0, 0.5), (1.0, -0.5), (-1.0, -0.5)]

    def SE2Control(self, q, u, qdot):

        def distance(p1, p2):
            return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

        robot_lenght = distance(self.robot_footprint[0], self.robot_footprint[1])
        theta = q[2]
        qdot[0] = u[0] * math.cos(theta)
        qdot[1] = u[0] * math.sin(theta)
        qdot[2] = u[0] * math.tan(u[1]) / robot_lenght

    def get_problem(
        self,
        n_robots: int,
        use_fluents: bool = False,
        add_metric: bool = True,
        force_sequential_schedule: bool = False,
    ) -> SchedulingMotionProblem:
        setup = SetupGenerator(n_robots, self.dirname)
        problem = SchedulingMotionProblem("Navigation")
        occ_map = OccupancyMap(setup.map_file[0], SE2(0, 0, 0))

        v_min = [1.0 for i in range(n_robots)]
        v_max = [20.0 for i in range(n_robots)]
        phi_min = [-1 for i in range(n_robots)]
        phi_max = [1 for i in range(n_robots)]

        with open(setup.locations_file) as file:
            locations_dict = yaml.safe_load(file.read())

        # robots, doors and shelves locations
        Robot = MovableType("robot")
        RobotConfig = ConfigurationType("robot_config", occ_map, ConfigurationKind.SE2)
        robots = [
            MovableObject(
                f"robot{r}",
                Robot,
                footprint=self.robot_footprint,
                motion_model=MotionModels.SE2,
                control_model=self.SE2Control,
                control_parameters={
                    "v_min": v_min[r],
                    "v_max": v_max[r],
                    "phi_min": phi_min[r],
                    "phi_max": phi_max[r],
                },
            )
            for r in range(n_robots)
        ]
        start_configs = [
            ConfigurationObject(
                f"s{r}", RobotConfig, SE2(*locations_dict["start_locs"][r])
            )
            for r in range(n_robots)
        ]

        goal_configs = [
            ConfigurationObject(
                f"g{r}", RobotConfig, SE2(*locations_dict["goal_locs"][r])
            )
            for r in range(n_robots)
        ]

        problem.add_objects(robots + start_configs + goal_configs)
        for r in range(n_robots):
            problem.set_initial_configuration(robots[r], start_configs[r])

        if use_fluents:
            robots_at = [
                problem.add_fluent(
                    f"robot{r}_at", RobotConfig, default_initial_value=start_configs[r]
                )
                for r in range(n_robots)
            ]

        for r in range(n_robots):

            activity = problem.add_motion_activity(
                f"move_r{r}_{start_configs[r].name}_{goal_configs[r].name}",
                1,
                optional=True,
            )

            dist = math.dist(
                (
                    start_configs[r].configuration.x,
                    start_configs[r].configuration.y,
                ),
                (goal_configs[r].configuration.x, goal_configs[r].configuration.y),
            )
            ub = math.ceil(dist / v_min[r])
            lb = int(
                compute_time_using_trapezoidal_velocity_profile(
                    distance=dist, acceleration=0.5, max_velocity=v_max[r]
                )
            )
            lb = max(1, lb)
            ub = max(lb, ub)

            activity.set_duration_bounds(lb, ub)

            if use_fluents:
                activity.add_condition(
                    TimePointInterval(Timing(0, activity.start)),
                    Equals(robots_at[r], start_configs[r]),
                )
                activity.add_effect(
                    Timing(-1, activity.end), robots_at[r], goal_configs[r]
                )

            activity.add_motion_effect(robots[r], goal_configs[r])
            activity.add_motion_constraint(
                ActivityWaypoints(robots[r], start_configs[r], [goal_configs[r]])
            )

            if not use_fluents:

                problem.add_constraint(
                    activity_precondition_constraint(
                        activity,
                        [],
                        [],
                        is_initially_applicable=True,
                    ),
                    scope=[activity.present],
                )

        if use_fluents:
            problem.add_condition(
                GlobalEndTiming(),
                And(Equals(robots_at[r], goal_configs[r]) for r in range(n_robots)),
            )

        if add_metric:
            problem.add_quality_metric(
                unified_planning.model.metrics.MinimizeMakespan()
            )
        if force_sequential_schedule:
            r = problem.add_resource(f"sequential_schedule", capacity=1)
            for activity in problem.activities:
                activity.uses(r, amount=1)
        return problem


def main():
    env = get_environment()
    env.factory.add_meta_engine("samp", "tampest.meta_engine_samp", "SampMetaEngine")
    env.factory.add_engine("aries-opt", "up_aries", "AriesOpt")
    env.credits_stream = None

    problem = Navigation().get_problem(n_robots=10, use_fluents=False)
    print(problem)
    print(problem.kind)

    # solve
    # with OneshotPlanner(name="samp[aries-opt]", params={"distance": 5.0}) as planner:
    with OneshotPlanner(
        name="samp[aries]", params={"distance": 5.0, "motion_planning_time": 20}
    ) as planner:
        # with OneshotPlanner(name="samp[cpse]", params={"distance": 5.0}) as planner:
        print("planner", planner.name)
        res = planner.solve(problem)
        print(res)


if __name__ == "__main__":
    main()
