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
PALLET_IMG = os.path.join(FILE_PATH, "maps/package.png")
SINGLE_PALLET_CONFIG_IMG = os.path.join(FILE_PATH, "maps/1.png")


class SetupGenerator:
    def __init__(
        self, n_drivers: int, n_trucks: int, n_packages: int, tmpdirname: str = None
    ) -> None:

        if tmpdirname:
            self.dirname = tmpdirname
        else:
            self.dirname = FILE_PATH

        self.map = self.get_map(n_packages)  # n_packages.yaml / n_packages.png
        self.locations = self.get_locations(
            n_drivers, n_trucks, n_packages, self.map[1]
        )  # n_treatments_n_pallets_n_robots.yaml

    def get_map(self, n_packages: int):

        file_path = os.path.join(self.dirname, "maps")
        yaml_file = os.path.join(file_path, f"{n_packages}.yaml")
        png_file = os.path.join(file_path, f"{n_packages}.png")

        if os.path.isfile(png_file) and os.path.isfile(yaml_file):
            return yaml_file, png_file
        else:
            single_pallet_config = Image.open(SINGLE_PALLET_CONFIG_IMG)
            pallet = Image.open(PALLET_IMG)

            map_image = Image.new("RGBA", (500 * n_packages, 500), (255, 255, 255, 255))
            map_image.paste(single_pallet_config)

            if n_packages > 1:
                for i in range(1, n_packages):
                    offset = (500 * i, 0)
                    map_image.paste(pallet, offset)

            os.makedirs(os.path.dirname(png_file), exist_ok=True)

            map_image.save(png_file)

            map_config = {}
            map_config["image"] = f"{n_packages}.png"
            map_config["resolution"] = 0.1
            map_config["origin"] = [0, 0, 0]
            map_config["negate"] = 0
            map_config["occupied_thresh"] = 0.3
            map_config["free_thresh"] = 0.1

            with open(yaml_file, "w") as file:
                yaml.dump(map_config, file)

            return yaml_file, png_file

    def get_locations(
        self, n_drivers: int, n_trucks: int, n_packages: int, map_png_file: str
    ):

        yaml_file = os.path.join(
            self.dirname, f"locations/{n_drivers}_{n_trucks}_{n_packages}.yaml"
        )

        if os.path.isfile(yaml_file):
            return yaml_file
        else:

            os.makedirs(os.path.dirname(yaml_file), exist_ok=True)

            data = {}

            data["doors"] = self.get_door_configs(n_packages)
            data["packages"] = self.get_packages_configs(n_packages)
            data["trucks"] = self.get_trucks_configs(n_trucks)
            data["drivers"] = self.get_random_drivers_configs(n_drivers, map_png_file)

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

    def get_packages_configs(self, n_packages):
        packages = {"start": [], "goal": []}
        for i in range(n_packages):
            pnt_start = [
                np.random.uniform(22.0 + 50 * i, 30 + 50 * i),
                np.random.uniform(35, 47),
                float(np.random.choice([-math.pi / 2.0, math.pi / 2.0])),
            ]
            pnt_goal = [
                np.random.uniform(22.0 + 50 * i, 30 + 50 * i),
                np.random.uniform(35, 47),
                float(np.random.choice([-math.pi / 2.0, math.pi / 2.0])),
            ]
            packages["start"].append(pnt_start)
            packages["goal"].append(pnt_goal)
        return packages

    def get_trucks_configs(self, n_trucks):
        trucks = []
        for i in range(n_trucks):
            pnt = [
                np.random.uniform(20.0, 30.0),
                np.random.uniform(7, 9),
                float(np.random.choice([-math.pi / 2.0, math.pi / 2.0])),
            ]
            trucks.append(pnt)
        return trucks

    def get_random_drivers_configs(self, n_drivers, map_png_file):
        drivers = []
        image_array = np.array(Image.open(map_png_file))
        white_pixel_indices = np.argwhere(image_array == 255)
        i = 0
        while i < n_drivers:
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
                    drivers.append(pnt)

        return drivers


# n_drivers, n_trucks, n_packages
# SetupGenerator(1, 1, 1)
