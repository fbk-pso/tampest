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

import itertools
import math
import os
import yaml
from unified_planning.shortcuts import *
import math
from benchmarks.tdoors.generator import SetupGenerator

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
    ):  # co = # of configurations left - c1 = # of configurations right

        setup = SetupGenerator(
            n_doors, n_robots, self.robot_footprint, c0, c1
        )  # change random uniform

        occ_map = OccupancyMap(setup.map, SE2(0, 0, 0))
        Robot = MovableType("robot")
        Door = MovableType("door")
        RobotConfig = ConfigurationType("robot_config", occ_map, ConfigurationKind.SE2)
        DoorConfig = ConfigurationType("door_config", occ_map, ConfigurationKind.SE2)

        robot_at = Fluent("robot_at", RobotConfig, robot=Robot)
        robot_moving = Fluent("moving", BoolType(), arg=Robot)
        door_at = Fluent("door_at", DoorConfig, door=Door)
        visited = Fluent("visited", BoolType(), robot=Robot, config=RobotConfig)
        open_config = Fluent("open_config", DoorConfig, door=Door)
        close_config = Fluent("close_config", DoorConfig, door=Door)
        button_config = Fluent("button_config", RobotConfig, door=Door)

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
        # robots = [MovableObject('robot%s' % i, Robot, footprint=self.robot_footprint, motion_model=MotionModels.SE2, control_model=self.SE2Control, control_parameters={"v_min": v_min[i], "v_max": v_max[i], "phi_min": phi_min[i], "phi_max": phi_max[i]}) for i in range(n_robots)]

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
            attached = Fluent(
                "attached", BoolType(), c_from=RobotConfig, c_to=RobotConfig
            )

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

        min_duration = Fluent(
            "min_duration", RealType(), r=Robot, c1=RobotConfig, c2=RobotConfig
        )
        max_duration = Fluent(
            "max_duration", RealType(), r=Robot, c1=RobotConfig, c2=RobotConfig
        )

        move_action = DurativeMotionAction(
            f"move", robot=Robot, c_from=RobotConfig, c_to=RobotConfig
        )
        robot_move_param = move_action.parameter("robot")
        c_from = move_action.parameter("c_from")
        c_to = move_action.parameter("c_to")
        move_action.set_duration_constraint(
            DurationInterval(
                min_duration(robot_move_param, c_from, c_to),
                max_duration(robot_move_param, c_from, c_to),
            )
        )
        move_action.add_condition(
            StartTiming(), Equals(robot_at(robot_move_param), c_from)
        )
        if connection_configs:
            move_action.add_condition(StartTiming(), attached(c_from, c_to))
        move_action.add_condition(StartTiming(), Not(robot_moving(robot_move_param)))
        move_action.add_effect(StartTiming(), robot_moving(robot_move_param), True)
        move_action.add_effect(EndTiming(), visited(robot_move_param, c_to), True)
        move_action.add_effect(EndTiming(), robot_moving(robot_move_param), False)
        move_action.add_effect(EndTiming(), robot_at(robot_move_param), c_to)
        move_action.add_motion_constraint(
            Waypoints(robot_move_param, c_from, [c_to], {d: door_at(d) for d in doors})
        )
        # move_action.add_motion_constraint(WaypointsConcurrent(robot_move_param, c_from, [c_to], {d : door_at(d) for d in doors}, {r: robot_at(r) for r in robots})))

        open_action = InstantaneousAction(
            f"open",
            robot=Robot,
            door=Door,
            c_r=RobotConfig,
            c_close=DoorConfig,
            c_open=DoorConfig,
        )
        robot_open_param = open_action.parameter("robot")
        door = open_action.parameter("door")
        c_r = open_action.parameter("c_r")
        c_close = open_action.parameter("c_close")
        c_open = open_action.parameter("c_open")
        open_action.add_precondition(Equals(robot_at(robot_open_param), c_r))
        open_action.add_precondition(Equals(button_config(door), c_r))
        open_action.add_precondition(Equals(door_at(door), c_close))
        open_action.add_precondition(Not(Equals(door_at(door), c_open)))
        open_action.add_precondition(Equals(close_config(door), c_close))
        open_action.add_precondition(Equals(open_config(door), c_open))
        open_action.add_effect(door_at(door), c_open)

        problem = Problem("nav_with_sliding_door_and_time")

        problem.add_objects(robots)
        problem.add_objects(doors)
        problem.add_objects(start_configs)
        problem.add_objects(goal_configs)
        if connection_configs:
            problem.add_objects(connection_configs)
        problem.add_objects(button_locs)
        problem.add_objects(open_locs)
        problem.add_objects(close_locs)

        problem.add_fluent(robot_at)
        problem.add_fluent(door_at)
        problem.add_fluent(open_config)
        problem.add_fluent(close_config)
        problem.add_fluent(button_config)
        problem.add_fluent(robot_moving, default_initial_value=False)
        problem.add_fluent(min_duration, default_initial_value=0.0)
        problem.add_fluent(max_duration, default_initial_value=0.0)
        problem.add_fluent(visited, default_initial_value=False)

        problem.add_actions((move_action, open_action))
        # problem.add_action(move_action)

        for i in range(n_doors):
            problem.set_initial_value(button_config(doors[i]), button_locs[i])
            problem.set_initial_value(open_config(doors[i]), open_locs[i])
            problem.set_initial_value(close_config(doors[i]), close_locs[i])

        for i in range(n_doors):
            initial_config = close_locs[i]
            # initial_config = open_locs[i]
            problem.set_initial_value(door_at(doors[i]), initial_config)

        all_configs = list(
            itertools.permutations(
                connection_configs + start_configs + goal_configs + button_locs, 2
            )
        )
        # print(all_configs)

        if connection_configs:
            problem.add_fluent(attached, default_initial_value=False)
            for c in all_configs:
                problem.set_initial_value(attached(c[0], c[1]), True)

        for i in range(n_robots):
            problem.set_initial_value(robot_at(robots[i]), start_configs[i])
            problem.add_goal(visited(robots[i], goal_configs[i]))

            for j in range(len(all_configs)):
                # euclidean distance between (x0, y0) and (x1, y1) * v_max or v_min
                dist = math.dist(
                    (
                        all_configs[j][0].configuration.x,
                        all_configs[j][0].configuration.y,
                    ),
                    (
                        all_configs[j][1].configuration.x,
                        all_configs[j][1].configuration.y,
                    ),
                )
                problem.set_initial_value(
                    max_duration(robots[i], all_configs[j][0], all_configs[j][1]),
                    dist / v_min[i],
                )
                problem.set_initial_value(
                    min_duration(robots[i], all_configs[j][0], all_configs[j][1]),
                    dist / v_max[i],
                )

        return problem


# problem = TemporalDoors().get_problem(1, 2)
#
# from unified_planning.shortcuts import OneshotPlanner
# from unified_planning.environment import get_environment
#
# get_environment().factory.add_engine("tempest", "tempest.engine", "TempestEngine")
#
# with OneshotPlanner(name="tamer") as planner:
#    res = planner.solve(problem)
#    print(res)
