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
import random
import unified_planning as up
from unified_planning.shortcuts import *
import os

import yaml
from benchmarks.majsp.generator import SetupGenerator

random.seed(23)

SETUP_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "configs/2D")


class MaJSP:
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
        self, n_robots: int, n_pallets: int, n_treatments: int
    ) -> up.model.Problem:

        # n_treatments = n_doors
        # n_pallets = n_depots configs
        # n_robots = n_starting poses
        setup = SetupGenerator(n_treatments, n_pallets, n_robots, self.dirname)

        problem = Problem("MaJSP")

        occ_map = OccupancyMap(setup.map[0], SE2(0, 0, 0))

        # Setting up Types
        Robot = MovableType("robot")
        Door = MovableType("door")
        RobotConfig = ConfigurationType("robot_config", occ_map, ConfigurationKind.SE2)
        DoorConfig = ConfigurationType("door_config", occ_map, ConfigurationKind.SE2)
        Pallet = UserType("Pallet")

        # Setting up Objects

        # NOPALLET = Object("NOPALLET", Pallet)
        # problem.add_object(NOPALLET)

        pallets = [Object("pallet%s" % i, Pallet) for i in range(n_pallets)]
        problem.add_objects(pallets)

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

        # each robot starts and ends at depot
        depots_configs = [
            ConfigurationObject(
                "d%s" % i, RobotConfig, SE2(*locations_dict["depots"][i])
            )
            for i in range(n_pallets)
        ]
        treatment_configs = [
            ConfigurationObject(
                "t%s" % i, RobotConfig, SE2(*locations_dict["treatments"][i])
            )
            for i in range(n_treatments)
        ]
        robot_configs = [
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
            for i in range(n_treatments)
        ]
        open_locs = [
            ConfigurationObject(
                "o%s" % i, DoorConfig, SE2(*locations_dict["doors"][i]["open"])
            )
            for i in range(n_treatments)
        ]
        close_locs = [
            ConfigurationObject(
                "c%s" % i, DoorConfig, SE2(*locations_dict["doors"][i]["close"])
            )
            for i in range(n_treatments)
        ]
        button_locs = [
            ConfigurationObject(
                "b%s" % i, RobotConfig, SE2(*locations_dict["doors"][i]["button"])
            )
            for i in range(n_treatments)
        ]

        problem.add_objects(
            robots
            + depots_configs
            + treatment_configs
            + robot_configs
            + doors
            + open_locs
            + close_locs
            + button_locs
        )

        # Setting up Fluents
        robot_at = Fluent("robot_at", RobotConfig, r=Robot)
        problem.add_fluent(robot_at)

        moving = Fluent("moving", BoolType(), r=Robot)
        problem.add_fluent(moving, default_initial_value=False)

        robot_has = Fluent("robot_has", BoolType(), r=Robot, b=Pallet)
        problem.add_fluent(robot_has, default_initial_value=False)

        position_empty = Fluent("position_empty", BoolType(), p=RobotConfig)
        problem.add_fluent(position_empty, default_initial_value=True)

        at_depot = Fluent("at_depot", BoolType(), b=Pallet)
        problem.add_fluent(at_depot, default_initial_value=False)

        pallet_at = Fluent("pallet_at", RobotConfig, b=Pallet)
        problem.add_fluent(pallet_at)

        treated = Fluent("treated", BoolType(), b=Pallet, p=RobotConfig)
        problem.add_fluent(treated, default_initial_value=False)

        ready = Fluent("ready", BoolType(), b=Pallet, p=RobotConfig)
        problem.add_fluent(ready, default_initial_value=False)

        door_at = Fluent("door_at", DoorConfig, d=Door)
        open_config = Fluent("open_config", DoorConfig, d=Door)
        close_config = Fluent("close_config", DoorConfig, d=Door)
        button_config = Fluent("button_config", RobotConfig, d=Door)
        depot_config = Fluent("depot_config", RobotConfig, b=Pallet)

        treatment_config = Fluent("treatment_config", BoolType(), p=RobotConfig)
        problem.add_fluent(treatment_config, default_initial_value=False)

        is_treatment_config_of = Fluent(
            "is_treatment_config_of", BoolType(), b=Pallet, p=RobotConfig
        )
        problem.add_fluent(is_treatment_config_of, default_initial_value=False)

        problem.add_fluents(
            [door_at, open_config, close_config, button_config, depot_config]
        )

        min_duration = Fluent(
            "min_duration", RealType(), r=Robot, l1=RobotConfig, l2=RobotConfig
        )
        problem.add_fluent(min_duration, default_initial_value=0)
        max_duration = Fluent(
            "max_duration", RealType(), r=Robot, l1=RobotConfig, l2=RobotConfig
        )
        problem.add_fluent(max_duration, default_initial_value=0)

        battery_level = Fluent("battery_level", IntType(0, 100), r=Robot)
        problem.add_fluent(battery_level, default_initial_value=60)

        # Setting up Actions:

        open_door = InstantaneousAction(
            f"open",
            r=Robot,
            d=Door,
            c_r=RobotConfig,
            c_close=DoorConfig,
            c_open=DoorConfig,
        )
        r = open_door.parameter("r")
        d = open_door.parameter("d")
        c_r = open_door.parameter("c_r")
        c_close = open_door.parameter("c_close")
        c_open = open_door.parameter("c_open")
        open_door.add_precondition(Not(moving(r)))
        open_door.add_precondition(Equals(robot_at(r), c_r))
        open_door.add_precondition(Equals(button_config(d), c_r))
        open_door.add_precondition(Equals(door_at(d), c_close))
        open_door.add_precondition(Not(Equals(door_at(d), c_open)))
        open_door.add_precondition(Equals(close_config(d), c_close))
        open_door.add_precondition(Equals(open_config(d), c_open))
        open_door.add_effect(door_at(d), c_open)

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
        move.add_condition(StartTiming(), Not(Equals(l_from, l_to)))
        move.add_condition(StartTiming(), Not(moving(r)))
        move.add_condition(StartTiming(), Equals(robot_at(r), l_from))
        move.add_condition(StartTiming(), GE(battery_level(r), 1))
        move.add_effect(StartTiming(), battery_level(r), Minus(battery_level(r), 1))
        move.add_effect(StartTiming(), moving(r), True)
        move.add_effect(EndTiming(), robot_at(r), l_to)
        move.add_effect(EndTiming(), moving(r), False)
        move.add_motion_constraint(
            Waypoints(r, l_from, [l_to], {d: door_at(d) for d in doors})
        )

        unload_at_depot = InstantaneousAction(
            "unload_at_depot", r=Robot, b=Pallet, p=RobotConfig
        )
        r = unload_at_depot.parameter("r")
        b = unload_at_depot.parameter("b")
        p = unload_at_depot.parameter("p")
        unload_at_depot.add_precondition(Not(moving(r)))
        unload_at_depot.add_precondition(position_empty(p))
        unload_at_depot.add_precondition(Not(at_depot(b)))
        unload_at_depot.add_precondition(robot_has(r, b))
        unload_at_depot.add_precondition(Equals(robot_at(r), p))
        unload_at_depot.add_precondition(Equals(pallet_at(b), p))
        unload_at_depot.add_precondition(Equals(depot_config(b), p))
        unload_at_depot.add_effect(robot_has(r, b), False)
        unload_at_depot.add_effect(at_depot(b), True)

        load_at_depot = InstantaneousAction(
            "load_at_depot", r=Robot, b=Pallet, p=RobotConfig
        )
        r = load_at_depot.parameter("r")
        b = load_at_depot.parameter("b")
        p = unload_at_depot.parameter("p")
        load_at_depot.add_precondition(Not(moving(r)))
        load_at_depot.add_precondition(at_depot(b))
        load_at_depot.add_precondition(Not(position_empty(p)))
        load_at_depot.add_precondition(Not(robot_has(r, b)))
        load_at_depot.add_precondition(Equals(robot_at(r), p))
        load_at_depot.add_precondition(Equals(pallet_at(b), p))
        load_at_depot.add_precondition(Equals(depot_config(b), p))
        load_at_depot.add_effect(robot_has(r, b), True)
        load_at_depot.add_effect(position_empty(p), True)
        load_at_depot.add_effect(at_depot(b), False)

        make_treat = DurativeAction("make_treatment", r=Robot, b=Pallet, p=RobotConfig)
        r = make_treat.parameter("r")
        b = make_treat.parameter("b")
        p = make_treat.parameter("p")
        make_treat.set_fixed_duration(20)
        make_treat.add_condition(StartTiming(), Not(moving(r)))
        make_treat.add_condition(StartTiming(), treatment_config(p))
        make_treat.add_condition(StartTiming(), is_treatment_config_of(b, p))
        make_treat.add_condition(StartTiming(), position_empty(p))
        make_treat.add_condition(StartTiming(), Equals(robot_at(r), p))
        make_treat.add_condition(StartTiming(), robot_has(r, b))
        make_treat.add_condition(StartTiming(), Not(treated(b, p)))
        make_treat.add_effect(StartTiming(), position_empty(p), False)
        make_treat.add_effect(StartTiming(), pallet_at(b), p)
        make_treat.add_effect(StartTiming(10), ready(b, p), True)
        make_treat.add_effect(EndTiming(), treated(b, p), True)
        make_treat.add_effect(StartTiming(), robot_has(r, b), False)

        load_at_treatment = InstantaneousAction(
            "load", r=Robot, b=Pallet, p=RobotConfig
        )
        r = load_at_treatment.parameter("r")
        b = load_at_treatment.parameter("b")
        p = load_at_treatment.parameter("p")
        load_at_treatment.add_precondition(Not(position_empty(p)))
        load_at_treatment.add_precondition(Not(robot_has(r, b)))
        load_at_treatment.add_precondition(treatment_config(p))
        load_at_treatment.add_precondition(is_treatment_config_of(b, p))
        load_at_treatment.add_precondition(Equals(robot_at(r), p))
        load_at_treatment.add_precondition(Equals(pallet_at(b), p))
        load_at_treatment.add_effect(position_empty(p), True)
        load_at_treatment.add_effect(robot_has(r, b), True)

        problem.add_actions(
            [
                move,
                open_door,
                load_at_depot,
                unload_at_depot,
                make_treat,
                load_at_treatment,
            ]
        )

        all_configs = list(
            itertools.permutations(
                robot_configs + treatment_configs + depots_configs + button_locs, 2
            )
        )

        for i in range(n_robots):
            problem.set_initial_value(robot_at(robots[i]), robot_configs[i])

            for j in range(len(all_configs) - 1):
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

        for i in range(n_treatments):
            problem.set_initial_value(button_config(doors[i]), button_locs[i])
            problem.set_initial_value(open_config(doors[i]), open_locs[i])
            problem.set_initial_value(close_config(doors[i]), close_locs[i])

            problem.set_initial_value(treatment_config(treatment_configs[i]), True)

            initial_config = close_locs[i]
            # initial_config = open_locs[i]
            problem.set_initial_value(door_at(doors[i]), initial_config)

        # more than one treatment for component - randomly assigned based on the number of treatments and the number of components

        for pallet in pallets:
            X = random.randint(
                1, n_treatments
            )  # Random number of treatments for this pallet
            assignment = random.sample(treatment_configs, X)
            for a in assignment:
                problem.set_initial_value(is_treatment_config_of(pallet, a), True)
                problem.add_goal(treated(pallet, a))

        for i in range(n_pallets):
            problem.set_initial_value(depot_config(pallets[i]), depots_configs[i])
            problem.set_initial_value(pallet_at(pallets[i]), depots_configs[i])
            problem.set_initial_value(at_depot(pallets[i]), True)
            problem.set_initial_value(position_empty(depots_configs[i]), False)

        return problem
