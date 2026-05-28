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
import yaml
from unified_planning.shortcuts import *
from unified_planning.model.motion import SchedulingMotionProblem
import math
from benchmarks.tdoors.generator import SetupGenerator
from benchmarks.scheduling_utils import (
    activity_precondition_constraint,
    compute_time_using_trapezoidal_velocity_profile,
)

FILE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "configs/2D")


class TemporalDoors:

    def __init__(self, tmpdirname=None) -> None:
        if tmpdirname:
            self.dirname = tmpdirname
        else:
            self.dirname = FILE_PATH

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
        n_doors: int,
        n_robots: int,
        *,
        c0: Optional[int] = 0,
        c1: Optional[int] = 0,
        use_resources=False,
        use_fluents=False,
    ):  # co = # of configurations left - c1 = # of configurations right

        setup = SetupGenerator(
            n_doors, n_robots, self.robot_footprint, c0, c1
        )  # change random uniform
        problem = SchedulingMotionProblem("nav_with_sliding_door_and_time")

        occ_map = OccupancyMap(setup.map, SE2(0, 0, 0))
        Robot = MovableType("robot")
        Door = MovableType("door")
        RobotConfig = ConfigurationType("robot_config", occ_map, ConfigurationKind.SE2)
        DoorConfig = ConfigurationType("door_config", occ_map, ConfigurationKind.SE2)

        # EPSILON = 0.01

        # u[0] = v = velocity
        # u[1] = phi = steering angle

        with open(setup.locations) as file:
            locations_dict = yaml.safe_load(file.read())

        v_min = [1.0 for i in range(n_robots)]
        v_max = [20.0 for i in range(n_robots)]  # 20
        phi_min = [-1 for i in range(n_robots)]
        phi_max = [1 for i in range(n_robots)]

        robots = [
            MovableObject(
                "robot%s" % i,
                Robot,
                footprint=self.robot_footprint,
                motion_model=MotionModels.REEDSSHEPP,
                motion_parameters={"turning_radius": 4.0},
                control_model=self.SE2Control,
                control_parameters={
                    "v_min": v_min[i],
                    "v_max": v_max[i],
                    "phi_min": phi_min[i],
                    "phi_max": phi_max[i],
                },
            )
            for i in range(n_robots)
        ]

        start_configs = [
            ConfigurationObject(
                "start%s" % i, RobotConfig, SE2(*locations_dict["robots"][i]["start"])
            )
            for i in range(n_robots)
        ]
        goal_configs = [
            ConfigurationObject(
                "goal%s" % i, RobotConfig, SE2(*locations_dict["robots"][i]["goal"])
            )
            for i in range(n_robots)
        ]

        connection_configs = []
        if c0 != 0 or c1 != 0:
            connection_configs = [
                ConfigurationObject(
                    "t%s" % i, RobotConfig, SE2(*locations_dict["extra_configs"][i])
                )
                for i in range(c0 + c1)
            ]

        doors = [
            MovableObject(
                "d%s" % i,
                Door,
                footprint=[(-0.5, 3.0), (0.5, 3.0), (0.5, -3.0), (-0.5, -3.0)],
                motion_model=MotionModels.SE2,
            )
            for i in range(n_doors)
        ]

        open_locs = [
            ConfigurationObject(
                "o%s" % i, DoorConfig, SE2(*locations_dict["doors"][i]["open"])
            )
            for i in range(n_doors)
        ]
        close_locs = [
            ConfigurationObject(
                "c%s" % i, DoorConfig, SE2(*locations_dict["doors"][i]["close"])
            )
            for i in range(n_doors)
        ]
        button_locs = [
            ConfigurationObject(
                "b%s" % i, RobotConfig, SE2(*locations_dict["doors"][i]["button"])
            )
            for i in range(n_doors)
        ]

        if use_resources:
            robot_resources = [
                problem.add_resource(f"r{i}", capacity=1) for i in range(n_robots)
            ]
            door_resources = [
                problem.add_resource(f"d{d}", capacity=1) for d in range(n_doors)
            ]
        if use_fluents:
            doors_at = [
                problem.add_fluent(
                    f"door{d}_at", DoorConfig, default_initial_value=close_locs[d]
                )
                for d in range(n_doors)
            ]
            robots_at = [
                problem.add_fluent(
                    f"robot{r}_at", RobotConfig, default_initial_value=start_configs[r]
                )
                for r in range(n_robots)
            ]

        all_locations = {
            l.name: l
            for l in button_locs + start_configs + goal_configs + connection_configs
        }
        move_activities = [
            {
                "start_location": {l: [] for l in all_locations},
                "end_location": {l: [] for l in all_locations},
            }
            for r in range(n_robots)
        ]
        for r in range(n_robots):
            robot_move_activities = set()
            for location_from in all_locations.values():
                for location_to in all_locations.values():
                    if location_from == location_to:
                        continue

                    activity = problem.add_motion_activity(
                        f"move_r{r}_{location_from.name}_{location_to.name}",
                        1,
                        optional=True,
                    )

                    dist = math.dist(
                        (location_from.configuration.x, location_from.configuration.y),
                        (location_to.configuration.x, location_to.configuration.y),
                    )
                    ub = math.ceil(dist / v_min[r])
                    lb = int(
                        compute_time_using_trapezoidal_velocity_profile(
                            distance=dist, acceleration=1, max_velocity=v_max[r]
                        )
                    )
                    lb = max(1, lb)
                    ub = max(lb, ub)
                    activity.set_duration_bounds(lb, ub)

                    if use_resources:
                        activity.uses(robot_resources[r])
                    if use_fluents:
                        activity.add_condition(
                            TimePointInterval(Timing(0, activity.start)),
                            Equals(robots_at[r], location_from),
                        )
                        activity.add_effect(
                            Timing(0, activity.end), robots_at[r], location_to
                        )
                        static_obstacles = {
                            door: doors_at[d] for d, door in enumerate(doors)
                        }
                        dynamic_obstacles = {
                            robot: robots_at[ri]
                            for ri, robot in enumerate(robots)
                            if ri != r
                        }
                    else:
                        static_obstacles = doors
                        dynamic_obstacles = list(set(robots) - {robots[r]})

                    activity.add_motion_effect(robots[r], location_to)
                    activity.add_motion_constraint(
                        ActivityWaypoints(
                            robots[r],
                            location_from,
                            [location_to],
                            static_obstacles=static_obstacles,
                            dynamic_obstacles_at_start=dynamic_obstacles,
                        )
                    )

                    robot_move_activities.add(activity)
                    move_activities[r]["start_location"][location_from.name].append(
                        (activity, location_to.name)
                    )
                    move_activities[r]["end_location"][location_to.name].append(
                        (activity, location_from.name)
                    )

            if not use_fluents:
                for start_location in move_activities[r]["start_location"]:
                    for activity, _ in move_activities[r]["start_location"][
                        start_location
                    ]:
                        helper_activities = [
                            a
                            for a, _ in move_activities[r]["end_location"][
                                start_location
                            ]
                            if activity != a
                        ]
                        rival_activities = list(
                            robot_move_activities - set(helper_activities) - {activity}
                        )
                        problem.add_constraint(
                            activity_precondition_constraint(
                                activity,
                                helper_activities,
                                rival_activities,
                                is_initially_applicable=(
                                    start_configs[r].name == start_location
                                ),
                            ),
                            scope=[activity.present],
                        )

            for d in range(n_doors):
                activity = problem.add_motion_activity(
                    f"open_d{d}_r{r}",
                    1,
                    optional=True,
                )
                if use_fluents:
                    activity.add_effect(
                        Timing(0, activity.end), doors_at[d], open_locs[d]
                    )
                if use_resources:
                    activity.uses(door_resources[d])
                    activity.uses(robot_resources[r])
                activity.add_motion_effect(doors[d], open_locs[d])

                if use_fluents:
                    activity.add_condition(
                        TimePointInterval(Timing(0, activity.start)),
                        Equals(robots_at[r], button_locs[d]),
                    )
                else:
                    helper_activities = [
                        a
                        for a, _ in move_activities[r]["end_location"][
                            button_locs[d].name
                        ]
                    ]
                    rival_activities = list(
                        robot_move_activities - set(helper_activities)
                    )
                    problem.add_constraint(
                        activity_precondition_constraint(
                            activity,
                            helper_activities,
                            rival_activities,
                            is_initially_applicable=(
                                start_configs[r].name == button_locs[d].name
                            ),
                        ),
                        scope=[activity.present],
                    )

        problem.add_objects(robots)
        problem.add_objects(doors)
        problem.add_objects(start_configs)
        problem.add_objects(goal_configs)
        if connection_configs:
            problem.add_objects(connection_configs)
        problem.add_objects(button_locs)
        problem.add_objects(open_locs)
        problem.add_objects(close_locs)

        for d in range(n_doors):
            problem.set_initial_configuration(doors[d], close_locs[d])

        for r in range(n_robots):
            problem.set_initial_configuration(robots[r], start_configs[r])

        # goal
        for r in range(n_robots):
            problem.add_constraint(
                Or(
                    activity.present
                    for activity, _ in move_activities[r]["end_location"][
                        goal_configs[r].name
                    ]
                )
            )

        problem.add_quality_metric(unified_planning.model.metrics.MinimizeMakespan())
        return problem


def main():
    env = get_environment()
    env.factory.add_meta_engine("samp", "tampest.meta_engine_samp", "SampMetaEngine")
    env.factory.add_engine("aries-opt", "up_aries", "AriesOpt")
    env.credits_stream = None

    problem = TemporalDoors().get_problem(
        n_doors=2, n_robots=1, use_resources=False, use_fluents=True
    )
    print(problem)
    print(problem.kind)

    # solve
    # with OneshotPlanner(name="samp[aries]", params={"distance": 5.0}) as planner:
    with OneshotPlanner(name="samp[aries-opt]", params={"distance": 5.0}) as planner:
        print("planner", planner.name)
        res = planner.solve(problem)
        print(res)


if __name__ == "__main__":
    main()
