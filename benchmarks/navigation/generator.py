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
DEFAULT_IMG = os.path.join(FILE_PATH, "maps/default_setup.png")
CORRIDOR_IMG = os.path.join(FILE_PATH, "maps/new_corridor.png")


class SetupGenerator:

    def __init__(
        self,
        robots: int,
        tmpdirname: str = None,
    ) -> None:

        if tmpdirname:
            self.dirname = tmpdirname
        else:
            self.dirname = FILE_PATH

        self.map_file = self.get_map(robots)
        self.locations_file = self.get_locations(robots)

    def get_map(self, robots: int):

        file_path = os.path.join(self.dirname, "maps")
        yaml_file = os.path.join(file_path, f"{robots}.yaml")
        png_file = os.path.join(file_path, f"{robots}.png")

        if os.path.isfile(png_file) and os.path.isfile(yaml_file):
            return yaml_file, png_file
        else:
            default_img = Image.open(DEFAULT_IMG)
            corridor_img = Image.open(CORRIDOR_IMG)

            height = default_img.height + corridor_img.height * (robots - 1)
            width = default_img.width

            map_image = Image.new("RGBA", (width, height), (255, 255, 255, 255))
            map_image.paste(default_img, (0, 0))

            if robots > 1:
                for i in range(1, robots):
                    offset = (0, default_img.height + corridor_img.height * (i - 1))
                    map_image.paste(corridor_img, offset)

            os.makedirs(os.path.dirname(png_file), exist_ok=True)

            map_image.save(png_file)

            map_config = {}
            map_config["image"] = f"{robots}.png"
            map_config["resolution"] = 0.1
            map_config["origin"] = [0, 0, 0]
            map_config["negate"] = 0
            map_config["occupied_thresh"] = 0.3
            map_config["free_thresh"] = 0.1

            with open(yaml_file, "w") as file:
                yaml.dump(map_config, file)

            return yaml_file, png_file

    def get_locations(self, robots: int):

        yaml_file = os.path.join(self.dirname, f"locations/{robots}.yaml")

        if os.path.isfile(yaml_file):
            return yaml_file
        else:

            os.makedirs(os.path.dirname(yaml_file), exist_ok=True)

            data = {}

            data["start_locs"] = self.get_start_locations(robots)
            data["goal_locs"] = self.get_goal_locations(robots)

            with open(yaml_file, "w") as file:
                yaml.dump(data, file)

            return yaml_file

    def get_start_locations(self, robots: int):

        default_img = Image.open(DEFAULT_IMG)
        corridor_img = Image.open(CORRIDOR_IMG)
        offset = corridor_img.height - default_img.height
        tot_height = default_img.height + corridor_img.height * (robots - 1)

        locs = []

        for i in range(robots):
            y = tot_height - (
                default_img.height / 2 + (default_img.height + offset) * i
            )
            x = 100
            theta = math.pi / 2
            locs.append([x / 10, y / 10, theta])

        return locs

    def get_goal_locations(self, robots: int):

        default_img = Image.open(DEFAULT_IMG)
        corridor_img = Image.open(CORRIDOR_IMG)
        offset = corridor_img.height - default_img.height
        tot_height = default_img.height + corridor_img.height * (robots - 1)

        locs = []

        for i in range(robots):
            y = tot_height - (
                default_img.height / 2 + (default_img.height + offset) * i
            )
            x = default_img.width - 200
            theta = math.pi / 2
            locs.append([x / 10, y / 10, theta])

        return locs
