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

import itertools
import math
import os
import random
import unified_planning as up
from unified_planning.model.motion import SchedulingMotionProblem
from unified_planning.shortcuts import *
import yaml
from benchmarks.kitting.generator import SetupGenerator
from benchmarks.scheduling_utils import (
    activity_precondition_constraint,
    enforce_activity_non_overlap_constraints,
    compute_time_using_trapezoidal_velocity_profile,
)

random.seed(23)

SETUP_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "configs/2D")


class Kitting:

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
        n_doors: int,
        n_robots: int,
        kit_size: int,
        n_kits: int,
        use_resources=False,
        use_fluents=False,
    ) -> SchedulingMotionProblem:

        n_components = kit_size * n_kits

        setup = SetupGenerator(n_doors, n_components, n_robots, self.dirname)
        n_doors *= 2

        problem = SchedulingMotionProblem("Kitting")

        occ_map = OccupancyMap(setup.map, SE2(0, 0, 0))

        Robot = MovableType("robot")
        Door = MovableType("door")
        RobotConfig = ConfigurationType("robot_config", occ_map, ConfigurationKind.SE2)
        DoorConfig = ConfigurationType("door_config", occ_map, ConfigurationKind.SE2)

        # robots and doors - objects

        with open(setup.locations) as file:
            locations_dict = yaml.safe_load(file.read())

        v_min = [1.0 for i in range(n_robots)]
        v_max = [20.0 for i in range(n_robots)]
        phi_min = [-1 for i in range(n_robots)]
        phi_max = [1 for i in range(n_robots)]

        robots = [
            MovableObject(
                "robot%s" % i,
                Robot,
                footprint=self.robot_footprint,
                motion_model=MotionModels.SE2,
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
                "s%s" % i, RobotConfig, SE2(*locations_dict["robots"][i])
            )
            for i in range(n_robots)
        ]

        doors = [
            MovableObject(
                "door%s" % i,
                Door,
                footprint=[(-0.5, 2.1), (0.5, 2.1), (0.5, -2.1), (-0.5, -2.1)],
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

        problem.add_objects(
            robots + start_configs + doors + open_locs + close_locs + button_locs
        )

        # components and kit - objects

        Component = UserType("Component")
        Kit = UserType("Kit")

        kits = [Object("kit%s" % i, Kit) for i in range(n_kits)]
        problem.add_objects(kits)

        EMPTY = Object("EMPTY", Component)
        problem.add_object(EMPTY)

        components = [Object("component%s" % i, Component) for i in range(n_components)]
        components_configs = [
            ConfigurationObject(
                "l%s" % i, RobotConfig, SE2(*locations_dict["components"][i])
            )
            for i in range(n_components)
        ]
        random.shuffle(components_configs)

        problem.add_objects(components + components_configs)

        for d in range(n_doors):
            problem.set_initial_configuration(doors[d], close_locs[d])

        for r in range(n_robots):
            problem.set_initial_configuration(robots[r], start_configs[r])

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

        if use_resources:
            operator = problem.add_resource("operator", capacity=1)
            robot_resources = [
                problem.add_resource(f"r{i}", capacity=1) for i in range(n_robots)
            ]
            door_resources = [
                problem.add_resource(f"d{d}", capacity=1) for d in range(n_doors)
            ]

        locations = {
            l.name: l for l in start_configs + button_locs + components_configs
        }
        move_activities = [
            {
                "start_location": {l: [] for l in locations},
                "end_location": {l: [] for l in locations},
            }
            for r in range(n_robots)
        ]
        load_activities = [[] for r in range(n_robots)]
        robot_move_activities = [set() for r in range(n_robots)]
        for r in range(n_robots):
            for location_from in locations.values():
                for location_to in locations.values():
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

                    robot_move_activities[r].add(activity)
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
                            robot_move_activities[r]
                            - set(helper_activities)
                            - {activity}
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

        open_activities = [[] for r in range(n_robots)]
        for r in range(n_robots):
            for d in range(n_doors):
                activity = problem.add_motion_activity(
                    f"open_d{d}_r{r}", 1, optional=True
                )
                open_activities[r].append(activity)
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
                        robot_move_activities[r] - set(helper_activities)
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

            for k in range(n_kits):
                load_activities[r].append([])
                for c in range(kit_size):
                    activity = problem.add_activity(
                        f"load_r{r}_k{k}_c{c}", 5, optional=True
                    )
                    if use_resources:
                        activity.uses(robot_resources[r])
                    load_activities[r][k].append(activity)

                    if use_fluents:
                        activity.add_condition(
                            TimePointInterval(Timing(0, activity.start)),
                            Equals(robots_at[r], components_configs[k * kit_size + c]),
                        )
                    else:
                        helper_activities = [
                            a
                            for a, _ in move_activities[r]["end_location"][
                                components_configs[k * kit_size + c].name
                            ]
                        ]
                        rival_activities = list(
                            robot_move_activities[r] - set(helper_activities)
                        )
                        problem.add_constraint(
                            activity_precondition_constraint(
                                activity,
                                helper_activities,
                                rival_activities,
                                is_initially_applicable=(
                                    start_configs[r].name
                                    == components_configs[k * kit_size + c].name
                                ),
                            ),
                            scope=[activity.present],
                        )

                    if c > 0:
                        # previous component processed
                        activity.add_constraint(
                            GE(activity.start, load_activities[r][k][c - 1].end)
                        )

        robot_builds_kit = [[] for r in range(n_robots)]
        for r in range(n_robots):
            for k in range(n_kits):
                robot_builds_kit[r].append(
                    problem.add_variable(f"robot{r}_builds_k{k}", BoolType())
                )
                # ensure that the robot that builds the kit performs all the load activities for that kit
                for c in range(kit_size):
                    problem.add_constraint(
                        Iff(robot_builds_kit[r][k], load_activities[r][k][c].present)
                    )

        for k in range(n_kits):
            # at least one robot builds the kit
            problem.add_constraint(Or(robot_builds_kit[r][k] for r in range(n_robots)))

            # at most one robot builds the kit
            if n_robots > 1:
                for r in range(n_robots):
                    problem.add_constraint(
                        Implies(
                            robot_builds_kit[r][k],
                            And(
                                [
                                    Not(robot_builds_kit[r2][k])
                                    for r2 in range(n_robots)
                                    if r2 != r
                                ]
                            ),
                        )
                    )

        prepare_unload_activities = []
        for k in range(n_kits):
            activity = problem.add_activity(f"prepare_unload_k{k}", 30)
            if use_resources:
                activity.uses(operator)
            prepare_unload_activities.append(activity)
            for k2 in range(k):
                # no overlap
                problem.add_constraint(
                    Or(
                        GE(
                            prepare_unload_activities[k].start,
                            prepare_unload_activities[k2].end,
                        ),
                        LE(
                            prepare_unload_activities[k].end,
                            prepare_unload_activities[k2].start,
                        ),
                    )
                )

        unload_activities = [[] for r in range(n_robots)]
        for r in range(n_robots):
            for k in range(n_kits):
                activity = problem.add_activity(f"unload_r{r}_k{k}", 5, optional=True)
                unload_activities[r].append(activity)
                if use_resources:
                    activity.uses(robot_resources[r])
                activity.add_constraint(
                    And(
                        GE(
                            activity.start, Plus(prepare_unload_activities[k].start, 10)
                        ),
                        LE(activity.end, Plus(prepare_unload_activities[k].start, 20)),
                    )
                )
                activity.add_constraint(
                    GE(activity.start, load_activities[r][k][-1].end)
                )
                problem.add_constraint(
                    Implies(robot_builds_kit[r][k], activity.present)
                )

                if use_fluents:
                    activity.add_condition(
                        TimePointInterval(Timing(0, activity.start)),
                        Equals(robots_at[r], start_configs[r]),
                    )
                else:
                    helper_activities = [
                        a
                        for a, _ in move_activities[r]["end_location"][
                            start_configs[r].name
                        ]
                    ]
                    rival_activities = list(
                        robot_move_activities[r] - set(helper_activities)
                    )
                    problem.add_constraint(
                        activity_precondition_constraint(
                            activity,
                            helper_activities,
                            rival_activities,
                            is_initially_applicable=False,
                        ),
                        scope=[activity.present],
                    )

        if not use_resources:
            enforce_activity_non_overlap_constraints(prepare_unload_activities, problem)

            for r in range(n_robots):
                all_robot_activities = (
                    list(robot_move_activities[r])
                    + open_activities[r]
                    + [a[0] for a in zip(*load_activities[r])]
                    + unload_activities[r]
                )
                enforce_activity_non_overlap_constraints(all_robot_activities, problem)

        problem.add_quality_metric(unified_planning.model.metrics.MinimizeMakespan())
        return problem


def main():
    env = get_environment()
    env.factory.add_meta_engine("samp", "tampest.meta_engine_samp", "SampMetaEngine")
    env.factory.add_engine("aries-opt", "up_aries", "AriesOpt")
    env.credits_stream = None

    problem = Kitting().get_problem(
        n_doors=1,
        n_robots=1,
        kit_size=1,
        n_kits=1,
        use_resources=False,
        use_fluents=True,
    )
    print(problem)
    print(problem.kind)

    # solve
    with OneshotPlanner(name="samp[aries-opt]", params={"distance": 5.0}) as planner:
        # with OneshotPlanner(name="samp[aries]", params={"distance": 5.0}) as planner:
        # with OneshotPlanner(name="samp[cpse]", params={"distance": 5.0}) as planner:
        print("planner", planner.name)
        res = planner.solve(problem)
        print(res)


if __name__ == "__main__":
    main()
