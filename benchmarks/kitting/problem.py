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
from unified_planning.shortcuts import *
import yaml
from benchmarks.kitting.generator import SetupGenerator

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
        self, n_doors: int, n_robots: int, kit_size: int, n_kits: int
    ) -> up.model.Problem:

        n_components = kit_size * n_kits

        setup = SetupGenerator(n_doors, n_components, n_robots, self.dirname)

        problem = Problem("Kitting")

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
            for i in range(n_doors * 2)
        ]
        open_locs = [
            ConfigurationObject(
                "o%s" % i, DoorConfig, SE2(*locations_dict["doors"][i]["open"])
            )
            for i in range(n_doors * 2)
        ]
        close_locs = [
            ConfigurationObject(
                "c%s" % i, DoorConfig, SE2(*locations_dict["doors"][i]["close"])
            )
            for i in range(n_doors * 2)
        ]
        button_locs = [
            ConfigurationObject(
                "b%s" % i, RobotConfig, SE2(*locations_dict["doors"][i]["button"])
            )
            for i in range(n_doors * 2)
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

        problem.add_objects(components + components_configs)

        # fluents

        robot_at = Fluent("robot_at", RobotConfig, r=Robot)
        unload_at = Fluent("unload_at", RobotConfig, r=Robot)
        door_at = Fluent("door_at", DoorConfig, d=Door)
        open_config = Fluent("open_config", DoorConfig, d=Door)
        close_config = Fluent("close_config", DoorConfig, d=Door)
        button_config = Fluent("button_config", RobotConfig, d=Door)
        is_present = Fluent("is_present", BoolType(), c=Component, l=RobotConfig)
        components_on_kit = Fluent(
            "components_on_kit", Component, k=Kit, i=IntType(0, kit_size - 1)
        )
        robot_busy = Fluent("robot_busy", BoolType(), r=Robot)
        human_busy = Fluent("human_busy")
        ready_to_receive = Fluent(
            "ready_to_receive", BoolType(), i=IntType(0, n_kits - 1)
        )
        components_on_robot = Fluent(
            "components_on_robot", Component, i=IntType(0, kit_size - 1), r=Robot
        )
        completed = Fluent("completed", BoolType(), i=IntType(0, n_kits - 1), k=Kit)
        robot_cnt = Fluent("robot_cnt", IntType(0, kit_size), r=Robot)
        kit_cnt = Fluent("kit_cnt", IntType(0, n_kits))
        battery = Fluent("battery", IntType(0, 10), r=Robot)
        min_duration = Fluent(
            "min_duration", RealType(), r=Robot, l1=RobotConfig, l2=RobotConfig
        )
        max_duration = Fluent(
            "max_duration", RealType(), r=Robot, l1=RobotConfig, l2=RobotConfig
        )

        problem.add_fluents(
            [robot_at, unload_at, door_at, open_config, close_config, button_config]
        )
        problem.add_fluent(is_present, default_initial_value=False)
        problem.add_fluent(components_on_kit, default_initial_value=EMPTY)
        problem.add_fluent(robot_busy, default_initial_value=False)
        problem.add_fluent(human_busy, default_initial_value=False)
        problem.add_fluent(ready_to_receive, default_initial_value=False)
        problem.add_fluent(components_on_robot, default_initial_value=EMPTY)
        problem.add_fluent(completed, default_initial_value=False)
        problem.add_fluent(robot_cnt, default_initial_value=0)
        problem.add_fluent(kit_cnt, default_initial_value=0)
        problem.add_fluent(min_duration, default_initial_value=0)
        problem.add_fluent(max_duration, default_initial_value=1000)
        problem.add_fluent(battery, default_initial_value=n_kits * kit_size * 3)

        open_door = InstantaneousAction(
            f"open",
            robot=Robot,
            door=Door,
            c_r=RobotConfig,
            c_close=DoorConfig,
            c_open=DoorConfig,
        )
        r = open_door.parameter("robot")
        d = open_door.parameter("door")
        c_r = open_door.parameter("c_r")
        c_close = open_door.parameter("c_close")
        c_open = open_door.parameter("c_open")
        open_door.add_precondition(Equals(robot_at(r), c_r))
        open_door.add_precondition(Equals(button_config(d), c_r))
        open_door.add_precondition(Equals(door_at(d), c_close))
        open_door.add_precondition(Not(Equals(door_at(d), c_open)))
        open_door.add_precondition(Equals(close_config(d), c_close))
        open_door.add_precondition(Equals(open_config(d), c_open))
        open_door.add_effect(door_at(d), c_open)
        problem.add_action(open_door)

        move = DurativeMotionAction(
            "move", r=Robot, l_from=RobotConfig, l_to=RobotConfig
        )
        r = move.parameter("r")
        l_from = move.parameter("l_from")
        l_to = move.parameter("l_to")
        move.set_duration_constraint(
            DurationInterval(
                min_duration(r, l_from, l_to), max_duration(r, l_from, l_to)
            )
        )
        move.add_condition(StartTiming(), Not(robot_busy(r)))
        move.add_effect(StartTiming(), robot_busy(r), True)
        move.add_effect(EndTiming(), robot_busy(r), False)
        move.add_condition(StartTiming(), Not(Equals(l_from, l_to)))
        move.add_condition(StartTiming(), Equals(robot_at(r), l_from))
        move.add_condition(StartTiming(), GT(battery(r), 0))
        move.add_effect(StartTiming(), battery(r), Minus(battery(r), 1))
        move.add_effect(EndTiming(), robot_at(r), l_to)
        move.add_motion_constraint(
            Waypoints(r, l_from, [l_to], {d: door_at(d) for d in doors})
        )
        problem.add_action(move)

        load = DurativeAction(
            "load",
            r=Robot,
            l=RobotConfig,
            c=Component,
            k=Kit,
            i=IntType(0, kit_size - 1),
        )
        r = load.parameter("r")
        l = load.parameter("l")
        c = load.parameter("c")
        k = load.parameter("k")
        i = load.parameter("i")
        load.set_fixed_duration(5)
        load.add_condition(StartTiming(), Not(robot_busy(r)))
        load.add_effect(StartTiming(), robot_busy(r), True)
        load.add_effect(EndTiming(), robot_busy(r), False)
        load.add_condition(StartTiming(), Equals(robot_at(r), l))
        load.add_condition(StartTiming(), is_present(c, l))
        load.add_condition(StartTiming(), Equals(robot_cnt(r), i))
        load.add_condition(StartTiming(), Equals(components_on_robot(i, r), EMPTY))
        load.add_condition(StartTiming(), Equals(components_on_kit(k, i), c))
        load.add_effect(EndTiming(), components_on_robot(i, r), c)
        load.add_effect(EndTiming(), robot_cnt(r), Plus(i, 1))
        problem.add_action(load)

        prepare_unload = DurativeAction("prepare_unload", i=IntType(0, n_kits - 1))
        i = prepare_unload.parameter("i")
        prepare_unload.set_fixed_duration(30)
        prepare_unload.add_condition(StartTiming(), Not(human_busy))
        prepare_unload.add_effect(StartTiming(), human_busy, True)
        prepare_unload.add_effect(EndTiming(), human_busy, False)
        prepare_unload.add_condition(StartTiming(), Equals(kit_cnt, i))
        prepare_unload.add_effect(StartTiming(10), ready_to_receive(i), True)
        prepare_unload.add_effect(StartTiming(20), ready_to_receive(i), False)
        problem.add_action(prepare_unload)

        unload = DurativeAction("unload", r=Robot, k=Kit, i=IntType(0, n_kits - 1))
        r = unload.parameter("r")
        k = unload.parameter("k")
        i = unload.parameter("i")
        unload.set_fixed_duration(5)
        unload.add_condition(StartTiming(), Not(robot_busy(r)))
        unload.add_effect(StartTiming(), robot_busy(r), True)
        unload.add_effect(EndTiming(), robot_busy(r), False)
        unload.add_condition(StartTiming(), Equals(robot_at(r), unload_at(r)))
        unload.add_condition(StartTiming(), Equals(kit_cnt, i))
        unload.add_condition(
            ClosedTimeInterval(StartTiming(), EndTiming()), ready_to_receive(i)
        )
        for j in range(kit_size):
            unload.add_condition(
                StartTiming(),
                Equals(components_on_robot(j, r), components_on_kit(k, j)),
            )
            unload.add_effect(EndTiming(), components_on_robot(j, r), EMPTY)
        unload.add_effect(EndTiming(), robot_cnt(r), 0)
        unload.add_effect(EndTiming(), completed(i, k), True)
        unload.add_effect(EndTiming(), kit_cnt, Plus(i, 1))
        unload.add_effect(EndTiming(), battery(r), kit_size + 1)
        problem.add_action(unload)

        all_configs = itertools.permutations(
            components_configs + start_configs + button_locs, 2
        )

        for i in range(n_robots):
            problem.set_initial_value(robot_at(robots[i]), start_configs[i])
            # the robot unload the 'kit' at its start location
            problem.set_initial_value(unload_at(robots[i]), start_configs[i])

            for c in all_configs:
                # euclidean distance between (x0, y0) and (x1, y1) * v_max or v_min
                dist = math.dist(
                    (
                        c[0].configuration.x,
                        c[0].configuration.y,
                    ),
                    (
                        c[1].configuration.x,
                        c[1].configuration.y,
                    ),
                )
                problem.set_initial_value(
                    max_duration(robots[i], c[0], c[1]),
                    dist / v_min[i],
                )
                problem.set_initial_value(
                    min_duration(robots[i], c[0], c[1]),
                    dist / v_max[i],
                )

        for i in range(n_doors * 2):
            problem.set_initial_value(button_config(doors[i]), button_locs[i])
            problem.set_initial_value(open_config(doors[i]), open_locs[i])
            problem.set_initial_value(close_config(doors[i]), close_locs[i])

        for i in range(n_doors * 2):
            initial_config = close_locs[i]
            # initial_config = open_locs[i]
            problem.set_initial_value(door_at(doors[i]), initial_config)

        for i in range(n_components):
            problem.set_initial_value(
                is_present(components[i], components_configs[i]), True
            )

        random.shuffle(components)
        groups = [
            components[i : i + kit_size] for i in range(0, len(components), kit_size)
        ]

        for k in range(n_kits):
            c = groups[k]
            for i in range(len(c)):
                problem.set_initial_value(components_on_kit(kits[k], i), c[i])

        for i in range(n_kits):
            problem.add_goal(completed(i, kits[i]))

        return problem
