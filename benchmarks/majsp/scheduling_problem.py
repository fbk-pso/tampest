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
import random
from unified_planning.shortcuts import *
from unified_planning.model.motion import SchedulingMotionProblem
import os

import yaml
from benchmarks.majsp.generator import SetupGenerator
from benchmarks.scheduling_utils import (
    activity_precondition_constraint,
    enforce_activity_non_overlap_constraints,
    compute_time_using_trapezoidal_velocity_profile,
)
from typing import Tuple

SETUP_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "configs/2D")

random.seed(23)


def generate_instance(
    n_jobs: int, n_machines: int
) -> Tuple[List[List[int]], Tuple[List[List[int]]]]:
    times = [[] for j in range(n_jobs)]
    machines = [[] for j in range(n_jobs)]

    for j in range(n_jobs):
        for m in range(n_machines):
            times[j].append(random.randint(1, 10))

        machines[j] = list(range(n_machines))
        random.shuffle(machines[j])

    return times, machines


class JSP:
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
        n_jobs: int,
        n_machines: int,
        use_resources=False,
        use_fluents=False,
        force_activity_duration=False,
        add_metric=True,
        force_sequential_schedule: bool = False,
    ) -> SchedulingMotionProblem:

        times, machines = generate_instance(n_jobs, n_machines)
        n_doors = n_machines

        setup = SetupGenerator(n_machines, n_jobs, n_robots, self.dirname)
        problem = SchedulingMotionProblem("JSP")
        occ_map = OccupancyMap(setup.map[0], SE2(0, 0, 0))

        # Setting up Types
        Robot = MovableType("robot")
        Door = MovableType("door")
        RobotConfig = ConfigurationType("robot_config", occ_map, ConfigurationKind.SE2)
        DoorConfig = ConfigurationType("door_config", occ_map, ConfigurationKind.SE2)

        # Setting up Objects

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
                motion_model=MotionModels.REEDSSHEPP,
                motion_parameters={"turning_radius": 1.0},
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

        machine_locs = [
            ConfigurationObject(
                "m%s" % i, RobotConfig, SE2(*locations_dict["treatments"][i])
            )
            for i in range(n_machines)
        ]
        robot_locs = [
            ConfigurationObject(
                "r%s" % i, RobotConfig, SE2(*locations_dict["robots"][i])
            )
            for i in range(n_robots)
        ]

        doors = [
            MovableObject(
                "door%s" % i,
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
            machine_resources = [
                problem.add_resource(f"m{m}", capacity=1) for m in range(n_machines)
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
                    f"robot{r}_at", RobotConfig, default_initial_value=robot_locs[r]
                )
                for r in range(n_robots)
            ]

        problem.add_objects(
            robots
            + machine_locs
            + robot_locs
            + doors
            + open_locs
            + close_locs
            + button_locs
        )

        for d in range(n_doors):
            problem.set_initial_configuration(doors[d], close_locs[d])

        for r in range(n_robots):
            problem.set_initial_configuration(robots[r], robot_locs[r])

        locations = {l.name: l for l in machine_locs + robot_locs + button_locs}
        move_activities = [
            {
                "start_location": {l: [] for l in locations},
                "end_location": {l: [] for l in locations},
            }
            for r in range(n_robots)
        ]
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
                                    robot_locs[r].name == start_location
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
                                robot_locs[r].name == button_locs[d].name
                            ),
                        ),
                        scope=[activity.present],
                    )

        jobs_activities = [[] for r in range(n_robots)]
        machine_activities = [[] for m in range(n_machines)]
        for r in range(n_robots):
            for j in range(n_jobs):
                jobs_activities[r].append([])

                for t in range(n_machines):
                    machine = machines[j][t]
                    activity = problem.add_activity(
                        f"r{r}_j{j}_t{t}_m{machine}",
                        duration=times[j][t],
                        optional=True,
                    )
                    if use_resources:
                        activity.uses(machine_resources[machine])
                        activity.uses(robot_resources[r])
                    jobs_activities[r][j].append(activity)
                    machine_activities[machine].append(activity)

                    if use_fluents:
                        activity.add_condition(
                            TimePointInterval(Timing(0, activity.start)),
                            Equals(robots_at[r], machine_locs[machine]),
                        )
                    else:
                        helper_activities = [
                            a
                            for a, _ in move_activities[r]["end_location"][
                                machine_locs[machine].name
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
                                    robot_locs[r].name == machine_locs[machine].name
                                ),
                            ),
                            scope=[activity.present],
                        )

        # precedence constraints
        for j in range(n_jobs):
            for t in range(1, n_machines):
                for r in range(n_robots):
                    jobs_activities[r][j][t].add_constraint(
                        And(
                            Implies(
                                jobs_activities[r2][j][t - 1].present,
                                GE(
                                    jobs_activities[r][j][t].start,
                                    jobs_activities[r2][j][t - 1].end,
                                ),
                            )
                            for r2 in range(n_robots)
                        )
                    )

        # exactly one task performed
        for j in range(n_jobs):
            for t in range(n_machines):
                # at least one
                problem.add_constraint(
                    Or(jobs_activities[r][j][t].present for r in range(n_robots))
                )
                if n_robots > 1:
                    # at most one
                    for r in range(n_robots):
                        problem.add_constraint(
                            Implies(
                                jobs_activities[r][j][t].present,
                                Not(
                                    Or(
                                        jobs_activities[r2][j][t].present
                                        for r2 in range(n_robots)
                                        if r2 != r
                                    )
                                ),
                            )
                        )

        if not use_resources:
            for m in range(n_machines):
                enforce_activity_non_overlap_constraints(machine_activities[m], problem)

            for r in range(n_robots):
                all_robot_activities = (
                    list(robot_move_activities[r])
                    + open_activities[r]
                    + [
                        a
                        for job_activities in jobs_activities[r]
                        for a in job_activities
                    ]
                )
                enforce_activity_non_overlap_constraints(all_robot_activities, problem)

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

    problem = JSP().get_problem(
        n_robots=1, n_jobs=1, n_machines=2, use_resources=False, use_fluents=True
    )
    print(problem)
    print(problem.kind)

    # solve
    # with OneshotPlanner(name="samp[cpse]", params={"distance": 5.0}) as planner:
    with OneshotPlanner(name="samp[aries-opt]", params={"distance": 5.0}) as planner:
        # with OneshotPlanner(name="samp[aries]", params={"distance": 5.0}) as planner:
        # with OneshotPlanner(problem_kind=problem.kind, params={"distance": 5.0}) as planner:
        print("planner", planner.name)
        res = planner.solve(problem)
        print(res)


if __name__ == "__main__":
    main()

    # for n_jobs in range(3, 6):
    #     for n_machines in range(3, 6):
    #         times, machines = generate_instance(n_jobs, n_machines)
    #         print(f"generate_instance(n_jobs={n_jobs}, n_machines={n_machines})")
    #         print("times")
    #         for l in times:
    #             print(l)
    #         print("machines")
    #         for l in machines:
    #             print(l)
