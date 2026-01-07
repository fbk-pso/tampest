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

import math
from typing import Tuple
import numpy as np
from shapely import Polygon
from unified_planning.shortcuts import *
import os
from PIL import Image
import yaml

np.random.seed(23)

FILE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "configs/2D")
DOOR_IMG = os.path.join(FILE_PATH, "maps/door.png")


class SetupGenerator:
    def __init__(
        self,
        n_doors: int,
        n_robots: int,
        r_footprint,
        c0: Optional[int] = 0,
        c1: Optional[int] = 0,
        tmpdirname: str = None,
    ) -> None:

        if tmpdirname:
            self.dirname = tmpdirname
        else:
            self.dirname = FILE_PATH

        self.r_footprint = r_footprint
        self.map = self.get_map(n_doors)
        self.locations = self.get_locations(n_doors, n_robots, c0, c1)

    def get_map(self, n: int) -> str:

        file_path = os.path.join(self.dirname, "maps")
        yaml_file = os.path.join(file_path, f"{n}.yaml")
        png_file = os.path.join(file_path, f"{n}.png")

        if os.path.isfile(png_file) and os.path.isfile(yaml_file):
            return yaml_file
        else:

            map_image = Image.new(
                "RGBA", (300 + 100 * (n - 1), 300), (255, 255, 255, 255)
            )
            door = Image.open(DOOR_IMG)
            for i in range(n):
                offset = (142 + 100 * i, 0)
                map_image.paste(door, offset)

            os.makedirs(os.path.dirname(png_file), exist_ok=True)

            map_image.save(png_file)

            map = {}
            map["image"] = f"{n}.png"
            map["resolution"] = 0.1
            map["origin"] = [0, 0, 0]
            map["negate"] = 0
            map["occupied_thresh"] = 0.3
            map["free_thresh"] = 0.1

            with open(yaml_file, "w") as file:
                yaml.dump(map, file)

            return yaml_file

    def get_locations(self, n_doors: int, n_robots: int, c0: int, c1):

        yaml_file = os.path.join(
            self.dirname, f"locations/{n_doors}_{n_robots}_{c0}_{c1}.yaml"
        )

        if os.path.isfile(yaml_file):
            return yaml_file
        else:

            os.makedirs(os.path.dirname(yaml_file), exist_ok=True)

            data = {}

            data["doors"] = self.get_door_configs(n_doors)
            data["robots"] = self.get_robot_configs(n_doors, n_robots)
            data["extra_configs"] = self.get_extra_configs(n_doors, c0, c1)

            with open(yaml_file, "w") as file:
                yaml.dump(data, file)

            return yaml_file

    def get_door_configs(self, n_doors):
        doors = {}

        for i in range(n_doors):
            doors[i] = {}

            # open
            doors[i]["open"] = [16.3 + 10.0 * i, 21.0, 0.0]

            # close
            doors[i]["close"] = [16.3 + 10.0 * i, 15.0, 0.0]

            # button
            doors[i]["button"] = [12.2 + 10.0 * i, 15.0, -math.pi / 2.0]
        return doors

    def get_robot_footprint_at_position(self, config):
        cx, cy = config[0], config[1]
        scaled_coords = [(cx + (x * 2.0), cy + (y * 2.0)) for x, y in self.r_footprint]
        return Polygon(scaled_coords)

    def get_robot_configs(self, n_doors: int, n_robots: int):
        robots = {}

        for i in range(n_robots):
            # start
            while True:
                start_config = self.get_random_config(2.0, 13.0, 2.0, 28.0)
                robot_start_fp = self.get_robot_footprint_at_position(start_config)

                collision = False
                for other_id in robots:
                    other_fp = self.get_robot_footprint_at_position(
                        robots[other_id]["start"]
                    )
                    if robot_start_fp.intersects(other_fp):
                        collision = True
                        break

                if not collision:
                    break  # found a valid start

            # goal
            while True:
                goal_config = [
                    np.random.uniform(
                        20.0 + 10 * (n_doors - 1), 26.0 + 10 * (n_doors - 1)
                    ),
                    np.random.uniform(2.0, 28.0),
                    float(np.random.choice([-math.pi / 2.0, math.pi / 2.0])),
                ]
                robot_goal_fp = self.get_robot_footprint_at_position(goal_config)

                collision = False
                for other_id in robots:
                    other_goal_fp = self.get_robot_footprint_at_position(
                        robots[other_id]["goal"]
                    )
                    if robot_goal_fp.intersects(other_goal_fp):
                        collision = True
                        break

                if not collision:
                    break  # found a valid goal

            robots[i] = {"start": start_config, "goal": goal_config}

        return robots

    def get_random_config(
        self, minx: float, maxx: float, miny: float, maxy: float
    ) -> List[Tuple[float, ...]]:
        angle = float(np.random.choice([-math.pi / 2.0, math.pi / 2.0]))
        pnt = [np.random.uniform(minx, maxx), np.random.uniform(miny, maxy), angle]
        return pnt

    def get_extra_configs(self, n_doors: int, c0: int, c1: int):
        connections = []
        if c0 != 0 or c1 != 0:
            filename = os.path.join(FILE_PATH, f"targets/{n_doors}_doors.txt")

            dic = ""
            with open(filename, "r") as f:
                for i in f.readlines():
                    dic = i
            dic = eval(dic)

            if c0 != 0:
                connections += dic[0][:c0]
            if c1 != 0:
                connections += dic[1][:c1]

        return connections


# n_doors, n_robots, c0, c1
# SetupGenerator(1, 1)
