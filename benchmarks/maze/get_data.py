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

import os
import random
import yaml

random.seed(23)

FILE_PATH = os.path.dirname(os.path.abspath(__file__))


def main():

    targets_file = os.path.join(FILE_PATH, "configs/tests/targets.yaml")
    doors_file = os.path.join(FILE_PATH, "configs/tests/doors.yaml")

    configs = {}
    robot_configs = [f"c{item}" for item in range(1, 10)]
    for c in range(1, len(robot_configs) + 1):
        configs[c] = random.sample(robot_configs, c)

    doors = {}
    door_configs = [f"d{item}" for item in range(0, 10)]
    for d in range(1, len(door_configs) + 1):
        doors[d] = random.sample(door_configs, d)

    with open(targets_file, "w") as file:
        yaml.dump(configs, file)

    with open(doors_file, "w") as file:
        yaml.dump(doors, file)


if __name__ == "__main__":
    main()
