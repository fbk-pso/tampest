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
SHELVES_IMG = os.path.join(FILE_PATH, "maps/shelves.png")
DEFAULT_IMG = os.path.join(FILE_PATH, "maps/default_setup.png")


class SetupGenerator:

    def __init__(
        self,
        shelves: int,
        robots: int,
        tmpdirname: str = None,
    ) -> None:

        if tmpdirname:
            self.dirname = tmpdirname
        else:
            self.dirname = FILE_PATH

        self.map_file = self.get_map(shelves)
        self.locations_file = self.get_locations(shelves, robots)

    def get_map(self, shelves: int):

        assert shelves % 2 == 0

        file_path = os.path.join(self.dirname, "maps")
        yaml_file = os.path.join(file_path, f"{shelves}.yaml")
        png_file = os.path.join(file_path, f"{shelves}.png")

        if os.path.isfile(png_file) and os.path.isfile(yaml_file):
            return yaml_file, png_file
        else:
            default_img = Image.open(DEFAULT_IMG)
            shelves_img = Image.open(SHELVES_IMG)

            height = 650 + ((shelves - 2) // 2) * 537
            map_image = Image.new("RGBA", (1000, height), (255, 255, 255, 255))
            map_image.paste(default_img, (0, 0))

            if shelves > 2:
                for i in range(1, shelves // 2):
                    offset = (0, 650 + (i - 1) * 537)
                    map_image.paste(shelves_img, offset)

            os.makedirs(os.path.dirname(png_file), exist_ok=True)

            map_image.save(png_file)

            map_config = {}
            map_config["image"] = f"{shelves}.png"
            map_config["resolution"] = 0.1
            map_config["origin"] = [0, 0, 0]
            map_config["negate"] = 0
            map_config["occupied_thresh"] = 0.3
            map_config["free_thresh"] = 0.1

            with open(yaml_file, "w") as file:
                yaml.dump(map_config, file)

            return yaml_file, png_file

    def get_locations(self, shelves: int, robots: int):

        yaml_file = os.path.join(self.dirname, f"locations/{shelves}_{robots}.yaml")

        if os.path.isfile(yaml_file):
            return yaml_file
        else:

            os.makedirs(os.path.dirname(yaml_file), exist_ok=True)

            data = {}

            data["external_locs"] = self.get_external_locations(shelves)
            data["internal_locs"] = self.get_internal_locations(shelves)
            data["depot_locs"] = self.get_depot_locations(robots, shelves)
            data["doors_open_locs"] = self.get_doors_open_locations(shelves)
            data["doors_close_locs"] = self.get_doors_close_locations(shelves)
            data["doors_button_locs"] = self.get_doors_button_locations(shelves)

            with open(yaml_file, "w") as file:
                yaml.dump(data, file)

            return yaml_file

    def get_external_locations(self, shelves: int):

        height = 650 + ((shelves - 2) // 2) * 537

        base_locs = [
            [590, height - 230, 0],
            [710, height - 230, 0],
            [830, height - 230, 0],
            [950, height - 230, 0],
            [590, height - 480, 0],
            [710, height - 480, 0],
            [830, height - 480, 0],
            [950, height - 480, 0],
        ]

        locs = []

        for i in range((shelves + 1) // 2):
            y_offset = i * 537
            for x, y, theta in base_locs:
                locs.append([x / 10, (y - y_offset) / 10, theta])

        return locs

    def get_internal_locations(self, shelves: int):

        height = 650 + ((shelves - 2) // 2) * 537

        base_locs = [
            [590, height - 330, 0],
            [710, height - 330, 0],
            [830, height - 330, 0],
            [950, height - 330, 0],
            [590, height - 380, 0],
            [710, height - 380, 0],
            [830, height - 380, 0],
            [950, height - 380, 0],
        ]

        locs = []

        for i in range((shelves + 1) // 2):
            y_offset = i * 537
            for x, y, theta in base_locs:
                locs.append([x / 10, (y - y_offset) / 10, theta])

        return locs

    def get_depot_locations(self, robots: int, shelves: int):

        height = 650 + ((shelves - 2) // 2) * 537

        assert 1 <= robots <= 10

        def row(y_offset: int, xs: List[int]):
            """Helper to generate a row of depot poses at given x positions."""
            return [[x, height - y_offset, math.pi / 2] for x in xs]

        depot_layouts = {
            1: row(60, [250]),
            2: row(60, [200, 300]),
            3: row(60, [150, 250, 350]),
            4: row(60, [100, 200, 300, 400]),
            5: row(60, [150, 200, 250, 300, 350]),
            6: row(40, [150, 250, 350]) + row(80, [150, 250, 350]),
            7: row(60, [100, 150, 200, 250, 300, 350, 400]),
            8: row(40, [100, 200, 300, 400]) + row(80, [100, 200, 300, 400]),
            9: row(80, [150, 200, 250, 300, 350]) + row(40, [100, 200, 300, 400]),
            10: row(40, [150, 200, 250, 300, 350]) + row(80, [150, 200, 250, 300, 350]),
        }

        depot = depot_layouts[robots]

        scaled_depot = [[x / 10, y / 10, theta] for x, y, theta in depot]

        return scaled_depot

    def get_doors_open_locations(self, shelves: int):

        height = 650 + ((shelves - 2) // 2) * 537

        base_locs = [515, height - 250, 0.0]

        locs = []

        for i in range((shelves + 1) // 2):
            y_offset = i * 537
            locs.append(
                [base_locs[0] / 10, (base_locs[1] - y_offset) / 10, base_locs[2]]
            )

        return locs

    def get_doors_close_locations(self, shelves: int):

        height = 650 + ((shelves - 2) // 2) * 537

        base_locs = [515, height - 350, 0.0]

        locs = []

        for i in range((shelves + 1) // 2):
            y_offset = i * 537
            locs.append(
                [base_locs[0] / 10, (base_locs[1] - y_offset) / 10, base_locs[2]]
            )

        return locs

    def get_doors_button_locations(self, shelves: int):

        height = 650 + ((shelves - 2) // 2) * 537

        base_locs = [460, height - 350, 0.0]

        locs = []

        for i in range(shelves // 2):
            y_offset = i * 537
            locs.append(
                [base_locs[0] / 10, (base_locs[1] - y_offset) / 10, base_locs[2]]
            )

        return locs
