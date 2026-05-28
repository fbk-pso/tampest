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
from benchmarks.logistics.generator import SetupGenerator
from benchmarks.scheduling_utils import (
    activity_precondition_constraint,
    enforce_activity_non_overlap_constraints,
    compute_time_using_trapezoidal_velocity_profile,
)

random.seed(23)

SETUP_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "configs/2D")


class Logistics:

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
        n_shelves: int,
        n_components: int,
        use_doors: bool = False,
        use_external_locations: bool = True,
        use_resources: bool = False,
        use_fluents: bool = False,
        force_activity_duration: bool = False,
        add_metric: bool = True,
        force_sequential_schedule: bool = False,
    ) -> SchedulingMotionProblem:
        setup = SetupGenerator(n_shelves, n_robots, self.dirname)
        problem = SchedulingMotionProblem("Logistics")
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
                # motion_model=MotionModels.SE2,
                motion_model=MotionModels.REEDSSHEPP,
                motion_parameters={"turning_radius": 1.0},
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
                f"s{r}", RobotConfig, SE2(*locations_dict["depot_locs"][r])
            )
            for r in range(n_robots)
        ]
        if use_doors:
            n_doors = len(locations_dict["doors_open_locs"])
            Door = MovableType("door")
            DoorConfig = ConfigurationType(
                "door_config", occ_map, ConfigurationKind.SE2
            )
            doors = [
                MovableObject(
                    f"d{d}",
                    Door,
                    footprint=[(-0.5, 5.0), (0.5, 5.0), (0.5, -5.0), (-0.5, -5.0)],
                    motion_model=MotionModels.SE2,
                )
                for d in range(n_doors)
            ]
            open_locs = [
                ConfigurationObject(
                    f"o{i}", DoorConfig, SE2(*locations_dict["doors_open_locs"][i])
                )
                for i in range(n_doors)
            ]
            close_locs = [
                ConfigurationObject(
                    f"c{i}", DoorConfig, SE2(*locations_dict["doors_close_locs"][i])
                )
                for i in range(n_doors)
            ]
            button_locs = [
                ConfigurationObject(
                    "b%s" % i, RobotConfig, SE2(*locations_dict["doors_button_locs"][i])
                )
                for i in range(n_doors)
            ]
            problem.add_objects(doors + open_locs + close_locs + button_locs)
            for d in range(n_doors):
                problem.set_initial_configuration(doors[d], close_locs[d])
        else:
            n_doors = 0
            doors = []
            button_locs = []

        # locations = random.choices(
        #     locations_dict["external_locs"] + locations_dict["internal_locs"],
        #     k=4 * n_shelves,
        # )
        locations = locations_dict["internal_locs"][:n_components]
        if use_external_locations:
            locations += locations_dict["external_locs"][:n_components]
        location_configs = [
            ConfigurationObject(f"l{i}", RobotConfig, SE2(*locations[i]))
            for i in range(len(locations))
        ]
        problem.add_objects(robots + start_configs + location_configs)
        for r in range(n_robots):
            problem.set_initial_configuration(robots[r], start_configs[r])

        # components
        n_component_types = n_components
        Component_types = [UserType(f"Component{i}") for i in range(n_component_types)]
        components = [
            Object(f"component{c}", Component_types[c]) for c in range(n_components)
        ]
        component_location_map = {}
        for i, component_type in enumerate(Component_types):
            # component_location_map[component_type] = random.choices(
            #     location_configs, k=random.randint(1, len(location_configs))
            # )
            component_location_map[component_type] = [location_configs[i]]
            if use_external_locations:
                component_location_map[component_type].append(
                    location_configs[n_components + i]
                )

        problem.add_objects(components)

        if use_fluents:
            robots_at = [
                problem.add_fluent(
                    f"robot{r}_at", RobotConfig, default_initial_value=start_configs[r]
                )
                for r in range(n_robots)
            ]
            robot_is_free = [
                problem.add_fluent(
                    f"robot{r}_is_free", BoolType(), default_initial_value=True
                )
                for r in range(n_robots)
            ]
            doors_at = [
                problem.add_fluent(
                    f"door{d}_at", DoorConfig, default_initial_value=close_locs[d]
                )
                for d in range(n_doors)
            ]

        if use_resources:
            robot_resources = [
                problem.add_resource(f"r{r}", capacity=1) for r in range(n_robots)
            ]
            door_resources = [
                problem.add_resource(f"d{d}", capacity=1) for d in range(n_doors)
            ]

        locations = {l.name: l for l in start_configs + location_configs + button_locs}
        move_activities = [
            {
                "start_location": {l: [] for l in locations},
                "end_location": {l: [] for l in locations},
            }
            for r in range(n_robots)
        ]
        robot_move_activities = [set() for r in range(n_robots)]
        for r in range(n_robots):
            connected_locations = [
                ([start_configs[r]], location_configs + button_locs),
                (location_configs + button_locs, [start_configs[r]]),
            ]
            if use_doors:
                for s in range(n_shelves // 2):
                    connected_locations.append(
                        (
                            [button_locs[s]],
                            location_configs[s * 8 : min(n_components, (s + 1) * 8)],
                        )
                    )
            connected_locations = [
                (location_from, location_to)
                for ll_from, ll_to in connected_locations
                for location_from in ll_from
                for location_to in ll_to
            ]

            for location_from, location_to in connected_locations:
                activity = problem.add_motion_activity(
                    f"move_r{r}_{location_from.name}_{location_to.name}",
                    1,
                    optional=True,
                )

                dist = math.dist(
                    (
                        location_from.configuration.x,
                        location_from.configuration.y,
                    ),
                    (location_to.configuration.x, location_to.configuration.y),
                )
                ub = math.ceil(dist / v_min[r])
                lb = int(
                    compute_time_using_trapezoidal_velocity_profile(
                        distance=dist, acceleration=0.5, max_velocity=v_max[r]
                    )
                )
                lb = max(1, lb)
                ub = max(lb, ub)
                if force_activity_duration:
                    lb = 1
                    ub = 100
                activity.set_duration_bounds(lb, ub)

                if use_resources:
                    activity.uses(robot_resources[r])
                if use_fluents:
                    activity.add_condition(
                        TimePointInterval(Timing(0, activity.start)),
                        Equals(robots_at[r], location_from),
                    )
                    activity.add_effect(
                        Timing(-1, activity.end), robots_at[r], location_to
                    )
                    static_obstacles = {
                        door: doors_at[d] for d, door in enumerate(doors)
                    }
                    dynamic_obstacles = {
                        robot: robots_at[ri]
                        for ri, robot in enumerate(robots)
                        if ri != r
                    }
                    if location_to == start_configs[r]:
                        activity.add_effect(
                            Timing(-1, activity.end), robot_is_free[r], True
                        )

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
                    f"open_d{d}_r{r}",
                    1,
                    optional=True,
                )
                if use_fluents:
                    activity.add_effect(
                        Timing(-1, activity.end), doors_at[d], open_locs[d]
                    )
                if use_resources:
                    activity.uses(door_resources[d])
                    activity.uses(robot_resources[r])
                activity.add_motion_effect(doors[d], open_locs[d])
                open_activities[r].append(activity)

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

        pick_activities = [[] for r in range(n_robots)]
        for r in range(n_robots):
            for c, component in enumerate(components):
                activity = problem.add_activity(
                    f"pick_r{r}_{component.name}", 1, optional=True
                )
                pick_activities[r].append(activity)

                if use_resources:
                    activity.uses(robot_resources[r])

                if use_fluents:
                    activity.add_condition(
                        TimePointInterval(Timing(0, activity.start)),
                        Or(
                            Equals(robots_at[r], location)
                            for location in component_location_map[component.type]
                        ),
                    )
                    activity.add_condition(
                        TimePointInterval(Timing(0, activity.start)), robot_is_free[r]
                    )
                    activity.add_effect(
                        Timing(-1, activity.end), robot_is_free[r], False
                    )
                else:
                    # ensure the robot is at a valid pick location before picking
                    helper_activities = []
                    for location in component_location_map[component.type]:
                        for a, _ in move_activities[r]["end_location"][location.name]:
                            helper_activities.append(a)
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

            if not use_fluents:
                for activity in pick_activities[r]:
                    # the robot is free
                    helper_activities = []
                    for location in start_configs:
                        for a, _ in move_activities[r]["start_location"][location.name]:
                            helper_activities.append(a)
                    rival_activities = set(pick_activities[r]) - {activity}
                    problem.add_constraint(
                        activity_precondition_constraint(
                            activity,
                            helper_activities,
                            rival_activities,
                            is_initially_applicable=False,
                        ),
                        scope=[activity.present],
                    )

        for c in range(n_components):
            # ensure that exactly one robot is assigned to pick a component
            problem.add_constraint(
                Or(pick_activities[r][c].present for r in range(n_robots))
            )
            if n_robots > 1:
                for r in range(n_robots):
                    pick_activities[r][c].add_constraint(
                        Not(
                            Or(
                                pick_activities[r2][c].present
                                for r2 in range(n_robots)
                                if r != r2
                            )
                        )
                    )

        if not use_resources:
            for r in range(n_robots):
                enforce_activity_non_overlap_constraints(
                    list(robot_move_activities[r])
                    + pick_activities[r]
                    + open_activities[r],
                    problem,
                )

        if use_fluents:
            problem.add_condition(
                GlobalEndTiming(),
                And(Equals(robots_at[r], start_configs[r]) for r in range(n_robots)),
            )
            # problem.add_condition(
            #     GlobalEndTiming(), And(robot_is_free[r] for r in range(n_robots))
            # )
        else:
            for r in range(n_robots):
                for c in range(n_components):
                    activity = pick_activities[r][c]
                    move_to_depot_activities = [
                        a
                        for location in start_configs
                        for a, _ in move_activities[r]["end_location"][location.name]
                    ]
                    activity.add_constraint(
                        Or(
                            And(a.present, LE(activity.end, a.start))
                            for a in move_to_depot_activities
                        )
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

    problem = Logistics().get_problem(
        n_robots=1, n_shelves=2, n_components=1, use_resources=False, use_fluents=False
    )
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
