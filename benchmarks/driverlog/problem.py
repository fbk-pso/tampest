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
import unified_planning as up
from unified_planning.shortcuts import *
import os

import yaml

from benchmarks.driverlog.generator import SetupGenerator

SETUP_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "configs/2D")


class Driverlog:
    def __init__(self, tmpdirname=None) -> None:
        if tmpdirname:
            self.dirname = tmpdirname
        else:
            self.dirname = SETUP_PATH

        self.truck_footprint = [(-1.0, 0.5), (1.0, 0.5), (1.0, -0.5), (-1.0, -0.5)]

    def SE2Control(self, q, u, qdot):
        robot_lenght = math.dist(self.truck_footprint[0], self.truck_footprint[1])
        theta = q[2]
        qdot[0] = u[0] * math.cos(theta)
        qdot[1] = u[0] * math.sin(theta)
        qdot[2] = u[0] * math.tan(u[1]) / robot_lenght

    def get_problem(
        self, n_drivers: int, n_trucks: int, n_packages: int
    ) -> up.model.Problem:

        setup = SetupGenerator(n_drivers, n_trucks, n_packages, self.dirname)

        occ_map = OccupancyMap(setup.map[0], SE2(0, 0, 0))

        problem = Problem("Driverlog")

        Truck = MovableType("truck")
        Door = MovableType("door")
        Location = ConfigurationType("location", occ_map, ConfigurationKind.SE2)
        DoorConfig = ConfigurationType("door_config", occ_map, ConfigurationKind.SE2)
        Driver = UserType("driver")
        Package = UserType("package")

        driver_at = Fluent("driver_at", BoolType(), d=Driver, l=Location)
        problem.add_fluent(driver_at, default_initial_value=False)

        truck_at = Fluent("truck_at", Location, t=Truck)
        problem.add_fluent(truck_at)

        pkg_at = Fluent("pkg_at", BoolType(), p=Package, l=Location)
        problem.add_fluent(pkg_at, default_initial_value=False)

        obj_in = Fluent("in", BoolType(), p=Package, t=Truck)
        problem.add_fluent(obj_in, default_initial_value=False)

        driving = Fluent("driving", BoolType(), d=Driver, t=Truck)
        problem.add_fluent(driving, default_initial_value=False)

        no_driver = Fluent("no_driver", BoolType(), t=Truck)
        problem.add_fluent(no_driver, default_initial_value=True)

        door_at = Fluent("door_at", DoorConfig, d=Door)
        open_config = Fluent("open_config", DoorConfig, d=Door)
        close_config = Fluent("close_config", DoorConfig, d=Door)
        button_config = Fluent("button_config", Location, d=Door)
        problem.add_fluents([door_at, open_config, close_config, button_config])

        moving = Fluent("moving", BoolType(), t=Truck)
        problem.add_fluent(moving, default_initial_value=False)

        min_duration = Fluent(
            "min_duration", RealType(), t=Truck, l1=Location, l2=Location
        )
        problem.add_fluent(min_duration, default_initial_value=0)
        max_duration = Fluent(
            "max_duration", RealType(), t=Truck, l1=Location, l2=Location
        )
        problem.add_fluent(max_duration, default_initial_value=0)

        open_door = InstantaneousAction(
            f"open",
            t=Truck,
            d=Door,
            c_r=Location,
            c_close=DoorConfig,
            c_open=DoorConfig,
        )
        t = open_door.parameter("t")
        d = open_door.parameter("d")
        c_r = open_door.parameter("c_r")
        c_close = open_door.parameter("c_close")
        c_open = open_door.parameter("c_open")
        open_door.add_precondition(Equals(truck_at(t), c_r))
        open_door.add_precondition(Equals(button_config(d), c_r))
        open_door.add_precondition(Equals(door_at(d), c_close))
        open_door.add_precondition(Not(Equals(door_at(d), c_open)))
        open_door.add_precondition(Equals(close_config(d), c_close))
        open_door.add_precondition(Equals(open_config(d), c_open))
        open_door.add_effect(door_at(d), c_open)

        load_truck = DurativeAction("load-truck", p=Package, t=Truck, l=Location)
        p = load_truck.parameter("p")
        t = load_truck.parameter("t")
        l = load_truck.parameter("l")
        load_truck.set_fixed_duration(2)
        load_truck.add_condition(StartTiming(), Equals(truck_at(t), l))
        load_truck.add_condition(StartTiming(), pkg_at(p, l))
        load_truck.add_effect(StartTiming(), pkg_at(p, l), False)
        load_truck.add_effect(EndTiming(), obj_in(p, t), True)
        load_truck.add_effect(EndTiming(), truck_at(t), l)

        unload_truck = DurativeAction("unload-truck", p=Package, t=Truck, l=Location)
        p = unload_truck.parameter("p")
        t = unload_truck.parameter("t")
        l = unload_truck.parameter("l")
        unload_truck.set_fixed_duration(2)
        unload_truck.add_condition(StartTiming(), Equals(truck_at(t), l))
        unload_truck.add_condition(StartTiming(), obj_in(p, t))
        unload_truck.add_condition(StartTiming(), Not(pkg_at(p, l)))
        unload_truck.add_effect(StartTiming(), pkg_at(p, l), True)
        unload_truck.add_effect(EndTiming(), obj_in(p, t), False)
        unload_truck.add_effect(EndTiming(), truck_at(t), l)

        board_truck = DurativeAction("board-truck", d=Driver, t=Truck, l=Location)
        d = board_truck.parameter("d")
        t = board_truck.parameter("t")
        l = board_truck.parameter("l")
        board_truck.set_fixed_duration(1)
        board_truck.add_condition(StartTiming(), Equals(truck_at(t), l))
        board_truck.add_condition(StartTiming(), driver_at(d, l))
        board_truck.add_condition(StartTiming(), no_driver(t))
        board_truck.add_effect(StartTiming(), driver_at(d, l), False)
        board_truck.add_effect(EndTiming(), driving(d, t), True)
        board_truck.add_effect(StartTiming(), no_driver(t), False)

        disembark_truck = DurativeAction(
            "disembark-truck", d=Driver, t=Truck, l=Location
        )
        d = disembark_truck.parameter("d")
        t = disembark_truck.parameter("t")
        l = disembark_truck.parameter("l")
        disembark_truck.set_fixed_duration(1)
        disembark_truck.add_condition(StartTiming(), Equals(truck_at(t), l))
        disembark_truck.add_condition(StartTiming(), driving(d, t))
        disembark_truck.add_effect(StartTiming(), driving(d, t), False)
        disembark_truck.add_effect(EndTiming(), driver_at(d, l), True)
        disembark_truck.add_effect(StartTiming(), no_driver(t), True)

        walk = DurativeAction("walk", d=Driver, l_from=Location, l_to=Location)
        d = walk.parameter("d")
        l_from = walk.parameter("l_from")
        l_to = walk.parameter("l_to")
        walk.set_fixed_duration(20)
        walk.add_condition(StartTiming(), driver_at(d, l_from))
        walk.add_effect(StartTiming(), driver_at(d, l_from), False)
        walk.add_effect(EndTiming(), driver_at(d, l_to), True)

        problem.add_actions(
            [load_truck, unload_truck, board_truck, open_door, disembark_truck, walk]
        )

        with open(setup.locations) as file:
            locations_dict = yaml.safe_load(file.read())

        doors = [
            MovableObject(
                "door%s" % i,
                Door,
                footprint=[(-0.5, 3.0), (0.5, 3.0), (0.5, -3.0), (-0.5, -3.0)],
                motion_model=MotionModels.SE2,
                motion_parameters={},
            )
            for i in range(n_packages)
        ]
        open_locs = [
            ConfigurationObject(
                "o%s" % i, DoorConfig, SE2(*locations_dict["doors"][i]["open"])
            )
            for i in range(n_packages)
        ]
        close_locs = [
            ConfigurationObject(
                "c%s" % i, DoorConfig, SE2(*locations_dict["doors"][i]["close"])
            )
            for i in range(n_packages)
        ]
        button_locs = [
            ConfigurationObject(
                "b%s" % i, Location, SE2(*locations_dict["doors"][i]["button"])
            )
            for i in range(n_packages)
        ]

        problem.add_objects(doors + open_locs + close_locs + button_locs)

        drive_truck = DurativeMotionAction(
            "drive-truck", t=Truck, d=Driver, l_from=Location, l_to=Location
        )
        d = drive_truck.parameter("d")
        t = drive_truck.parameter("t")
        l_from = drive_truck.parameter("l_from")
        l_to = drive_truck.parameter("l_to")
        drive_truck.set_duration_constraint(
            DurationInterval(
                min_duration(t, l_from, l_to), max_duration(t, l_from, l_to)
            )
        )
        drive_truck.add_condition(StartTiming(), Equals(truck_at(t), l_from))
        drive_truck.add_condition(StartTiming(), Not(no_driver(t)))
        drive_truck.add_condition(StartTiming(), driving(d, t))
        drive_truck.add_condition(StartTiming(), Not(moving(t)))
        drive_truck.add_effect(StartTiming(), moving(t), True)
        drive_truck.add_effect(EndTiming(), moving(t), False)
        drive_truck.add_effect(EndTiming(), truck_at(t), l_to)
        drive_truck.add_motion_constraint(
            Waypoints(t, l_from, [l_to], {d: door_at(d) for d in doors})
        )
        problem.add_action(drive_truck)

        for i in range(n_packages):
            problem.set_initial_value(button_config(doors[i]), button_locs[i])
            problem.set_initial_value(open_config(doors[i]), open_locs[i])
            problem.set_initial_value(close_config(doors[i]), close_locs[i])

            initial_config = close_locs[i]
            # initial_config = open_locs[i]
            problem.set_initial_value(door_at(doors[i]), initial_config)

        drivers = [Object("driver%s" % i, Driver) for i in range(n_drivers)]
        driver_start_locations = [
            ConfigurationObject(
                "ds%s" % i, Location, SE2(*locations_dict["drivers"][i])
            )
            for i in range(n_drivers)
        ]
        problem.add_objects(drivers + driver_start_locations)

        for i in range(n_drivers):
            problem.set_initial_value(
                driver_at(drivers[i], driver_start_locations[i]), True
            )

        packages = [Object("pkg%s" % i, Package) for i in range(n_packages)]
        pkg_start_locations = [
            ConfigurationObject(
                "ps%s" % i, Location, SE2(*locations_dict["packages"]["start"][i])
            )
            for i in range(n_packages)
        ]
        pkg_goal_locations = [
            ConfigurationObject(
                "pg%s" % i, Location, SE2(*locations_dict["packages"]["goal"][i])
            )
            for i in range(n_packages)
        ]
        problem.add_objects(packages + pkg_start_locations + pkg_goal_locations)

        for i in range(n_packages):
            problem.set_initial_value(pkg_at(packages[i], pkg_start_locations[i]), True)
            problem.add_goal(pkg_at(packages[i], pkg_goal_locations[i]))

        v_min = [1.0 for i in range(n_trucks)]
        v_max = [20.0 for i in range(n_trucks)]
        phi_min = [-1 for i in range(n_trucks)]
        phi_max = [1 for i in range(n_trucks)]

        trucks = [
            MovableObject(
                "truck%s" % i,
                Truck,
                footprint=self.truck_footprint,
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
            for i in range(n_trucks)
        ]

        truck_start_locations = [
            ConfigurationObject("ts%s" % i, Location, SE2(*locations_dict["trucks"][i]))
            for i in range(n_trucks)
        ]
        problem.add_objects(trucks + truck_start_locations)

        all_configs = list(
            itertools.permutations(
                pkg_start_locations
                + pkg_goal_locations
                + truck_start_locations
                + button_locs,
                2,
            )
        )

        for i in range(n_trucks):
            problem.set_initial_value(truck_at(trucks[i]), truck_start_locations[i])

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
                    max_duration(trucks[i], all_configs[j][0], all_configs[j][1]),
                    dist / v_min[i],
                )
                problem.set_initial_value(
                    min_duration(trucks[i], all_configs[j][0], all_configs[j][1]),
                    dist / v_max[i],
                )

        return problem


# # n_drivers, n_trucks, n_packages
# problem = Driverlog().get_problem(1, 2, 1)
# print(problem)

# from unified_planning.shortcuts import OneshotPlanner
# from unified_planning.environment import get_environment

# get_environment().factory.add_engine("tempest", "tempest.engine", "TempestEngine")

# with OneshotPlanner(name="tempest") as planner:
# #with OneshotPlanner(name="tamer") as planner:
#     res = planner.solve(problem)
#     print(res)
