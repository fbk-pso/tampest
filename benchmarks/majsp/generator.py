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
import numpy as np
from unified_planning.shortcuts import *
import os
from PIL import Image
import yaml

np.random.seed(23)

FILE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "configs/2D")
PALLET_IMG = os.path.join(FILE_PATH, "maps/pallet.png")
SINGLE_PALLET_CONFIG_IMG = os.path.join(FILE_PATH, "maps/1.png")


class SetupGenerator:
    def __init__(
        self, n_treatments: int, n_pallets: int, n_robots: int, tmpdirname: str = None
    ) -> None:

        if tmpdirname:
            self.dirname = tmpdirname
        else:
            self.dirname = FILE_PATH

        # n_treatments = n_doors
        # n_pallets = n_depots configs
        # n_robots = n_starting poses

        self.map = self.get_map(n_treatments)  # n_treatments.yaml / n_treatments.png
        self.locations = self.get_locations(
            n_treatments, n_pallets, n_robots, self.map[1]
        )  # n_treatments_n_pallets_n_robots.yaml

    def get_map(self, n_treatments: int):

        file_path = os.path.join(self.dirname, "maps")
        yaml_file = os.path.join(file_path, f"{n_treatments}.yaml")
        png_file = os.path.join(file_path, f"{n_treatments}.png")

        if os.path.isfile(png_file) and os.path.isfile(yaml_file):
            return yaml_file, png_file
        else:
            single_pallet_config = Image.open(SINGLE_PALLET_CONFIG_IMG)
            pallet = Image.open(PALLET_IMG)

            map_image = Image.new(
                "RGBA", (500 * n_treatments, 500), (255, 255, 255, 255)
            )
            map_image.paste(single_pallet_config)

            if n_treatments > 1:
                for i in range(1, n_treatments):
                    offset = (500 * i, 0)
                    map_image.paste(pallet, offset)

            os.makedirs(os.path.dirname(png_file), exist_ok=True)

            map_image.save(png_file)

            map_config = {}
            map_config["image"] = f"{n_treatments}.png"
            map_config["resolution"] = 0.1
            map_config["origin"] = [0, 0, 0]
            map_config["negate"] = 0
            map_config["occupied_thresh"] = 0.3
            map_config["free_thresh"] = 0.1

            with open(yaml_file, "w") as file:
                yaml.dump(map_config, file)

            return yaml_file, png_file

    def get_locations(
        self, n_treatments: int, n_pallets: int, n_robots: int, map_png_file: str
    ):

        yaml_file = os.path.join(
            self.dirname, f"locations/{n_treatments}_{n_pallets}_{n_robots}.yaml"
        )

        if os.path.isfile(yaml_file):
            return yaml_file
        else:

            os.makedirs(os.path.dirname(yaml_file), exist_ok=True)

            data = {}

            data["doors"] = self.get_door_configs(
                n_treatments
            )  # n_pallets open, close, button
            data["treatments"] = self.get_treatment_configs(
                n_treatments
            )  # n_treatments inside space closed by the door
            data["depots"] = self.get_pallets_in_depots_configs(
                n_pallets
            )  # n_pallets random inside
            data["robots"] = self.get_random_robot_configs(
                n_robots, map_png_file
            )  # n_robots start configs (in depot)

            with open(yaml_file, "w") as file:
                yaml.dump(data, file)

            return yaml_file

    def get_door_configs(self, n_doors):
        doors = {}

        for i in range(n_doors):
            doors[i] = {}

            # open
            doors[i]["open"] = [21 + 50 * i, 32.0, math.pi / 2.0]

            # close
            doors[i]["close"] = [25 + 50 * i, 32.0, math.pi / 2.0]

            # button
            doors[i]["button"] = [25 + 50 * i, 29.0, math.pi / 2.0]
        return doors

    def get_treatment_configs(self, n_treatments):
        treatments = []
        for i in range(n_treatments):
            pnt = [
                np.random.uniform(22.0 + 50 * i, 30 + 50 * i),
                np.random.uniform(35, 47),
                float(np.random.choice([-math.pi / 2.0, math.pi / 2.0])),
            ]
            treatments.append(pnt)
        return treatments

    def get_pallets_in_depots_configs(self, n_pallets):
        pallets = []
        for i in range(n_pallets):
            pnt = [
                np.random.uniform(20.0, 30.0),
                np.random.uniform(7, 9),
                float(np.random.choice([-math.pi / 2.0, math.pi / 2.0])),
            ]
            pallets.append(pnt)
        return pallets

    def get_random_robot_configs(self, n_robots, map_png_file):
        robots = []
        image_array = np.array(Image.open(map_png_file))
        white_pixel_indices = np.argwhere(image_array == 255)

        i = 0
        while i < n_robots:
            sampled_indice = white_pixel_indices[
                np.random.choice(white_pixel_indices.shape[0], 1)
            ]
            for x, y, _ in sampled_indice:

                # Define the radius (delta)
                delta = 5  # For example, a radius of 5 pixels

                # Define the region around the sampled pixel
                row_min = max(x - delta, 0)
                row_max = min(x + delta, image_array.shape[0] - 1)
                col_min = max(y - delta, 0)
                col_max = min(y + delta, image_array.shape[1] - 1)

                # Extract the region of interest (ROI)
                region_of_interest = image_array[
                    row_min : row_max + 1, col_min : col_max + 1
                ]

                # Check if all pixels in the region are white
                all_white = np.all(region_of_interest == 255)

                if all_white and x in [250, 300]:
                    i += 1
                    pnt = [
                        float(y / 10),
                        float((500.0 - x)) / 10,
                        float(np.random.choice([-math.pi / 2.0, math.pi / 2.0])),
                    ]
                    robots.append(pnt)

        return robots


# n_treatments, n_pallets, n_robots
# SetupGenerator(1, 1, 1)
