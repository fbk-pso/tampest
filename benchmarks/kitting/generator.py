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
import numpy as np
from unified_planning.shortcuts import *
import os
from PIL import Image
import yaml

np.random.seed(23)

FILE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "configs/2D")
TEMPLATE_IMG = os.path.join(FILE_PATH, "maps/office.png")


class SetupGenerator:
    def __init__(
        self, n_doors: int, n_components: int, n_robots: int, tmpdirname: str = None
    ) -> None:

        if tmpdirname:
            self.dirname = tmpdirname
        else:
            self.dirname = FILE_PATH

        self.map = self.get_map(n_doors)  # n_doors.yaml
        self.locations = self.get_locations(
            n_doors, n_components, n_robots
        )  # n_doors_n_components_n_robots.yaml

    def get_map(self, n_doors: int):

        file_path = os.path.join(self.dirname, "maps")
        yaml_file = os.path.join(file_path, f"{n_doors}.yaml")
        png_file = os.path.join(file_path, f"{n_doors}.png")

        if os.path.isfile(png_file) and os.path.isfile(yaml_file):
            return yaml_file
        else:
            map_image = Image.new(
                "RGBA", (300 + 92 * (n_doors - 1), 300), (255, 255, 255, 255)
            )
            door = Image.open(TEMPLATE_IMG)
            for i in range(n_doors):
                offset = (201 + 92 * i, 0)
                map_image.paste(door, offset)

            os.makedirs(os.path.dirname(png_file), exist_ok=True)

            map_image.save(png_file)

            map_config = {}
            map_config["image"] = f"{n_doors}.png"
            map_config["resolution"] = 0.1
            map_config["origin"] = [0, 0, 0]
            map_config["negate"] = 0
            map_config["occupied_thresh"] = 0.3
            map_config["free_thresh"] = 0.1

            with open(yaml_file, "w") as file:
                yaml.dump(map_config, file)

            return yaml_file

    def get_locations(self, n_doors: int, n_components: int, n_robots: int):

        yaml_file = os.path.join(
            self.dirname, f"locations/{n_doors}_{n_components}_{n_robots}.yaml"
        )

        if os.path.isfile(yaml_file):
            return yaml_file
        else:

            os.makedirs(os.path.dirname(yaml_file), exist_ok=True)

            data = {}

            data["doors"] = self.get_door_configs(n_doors)
            data["components"] = self.get_comp_configs(n_doors, n_components)
            data["robots"] = self.get_robot_configs(n_robots)

            with open(yaml_file, "w") as file:
                yaml.dump(data, file)

            return yaml_file

    def get_door_configs(self, n_doors):
        doors = {}

        for i in range(0, n_doors * 2, 2):
            if i not in doors:
                doors[i] = {}
                doors[i + 1] = {}

            # open
            doors[i]["open"] = [20.8 + 4.6 * i, 11, math.pi / 2.0]
            doors[i + 1]["open"] = [20.8 + 4.6 * i, 19.8, math.pi / 2.0]

            # close
            doors[i]["close"] = [25 + 4.6 * i, 11.0, math.pi / 2.0]
            doors[i + 1]["close"] = [25 + 4.6 * i, 19.8, math.pi / 2.0]

            # button
            doors[i]["button"] = [25 + 4.6 * i, 12.5, 0.0]
            doors[i + 1]["button"] = [25 + 4.6 * i, 18.3, 0.0]

        return doors

    def get_comp_configs(self, n_doors, n_components):
        components = []
        for i in range(n_components):
            y_choices = [(2, 8), (22, 28)]
            index = np.random.randint(0, len(y_choices) - 1)
            pnt = [
                np.random.uniform(22 + 10 * (n_doors - 1), 27 + 10 * (n_doors - 1)),
                np.random.uniform(y_choices[index][0], y_choices[index][1]),
                float(np.random.choice([-math.pi / 2.0, math.pi / 2.0])),
            ]
            components.append(pnt)
        return components

    def get_robot_configs(self, n_robots):
        robots = []
        for i in range(n_robots):
            pnt = [
                np.random.uniform(5, 15),
                np.random.uniform(5, 25),
                float(np.random.choice([-math.pi / 2.0, math.pi / 2.0])),
            ]
            robots.append(pnt)
        return robots


# SetupGenerator(1, 1, 1)
