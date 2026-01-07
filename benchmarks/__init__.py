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
from typing import Optional
import yaml


def get_problem(
    domain,
    dim,
    d,
    *,
    c: Optional[int] = None,
    n_robots: Optional[int] = 1,
    capacity: Optional[int] = None,
    kit_size: Optional[int] = None,
    n_kit: Optional[int] = None,
    n_pallets: Optional[int] = None,
    n_drivers: Optional[int] = None,
    n_tiles: Optional[int] = None,
    n_colors: Optional[int] = None
):
    if domain == "doors":
        from benchmarks.doors.problem import Doors

        c0, c1 = [(0, 0), (10, 0), (0, 10), (5, 5)][c]
        if dim == "3D":
            raise NotImplementedError("Benchmark Doors is only available in 2D.")
        return Doors().get_problem(d, c0=c0, c1=c1)
    if domain == "maze":
        from benchmarks.maze.problem import Maze

        return Maze().get_problem(dim, d, c)
    if domain == "delivery":
        from benchmarks.delivery.problem import Delivery

        FILE_PATH = os.path.dirname(os.path.abspath(__file__))
        CONFIG_FILE = os.path.join(FILE_PATH, "delivery/configs/tests/all.yaml")
        with open(CONFIG_FILE) as f:
            config_data = yaml.safe_load(f)
        current_config = config_data[
            c
        ]  # (red parcels, green parcels, red delivered parcels, green delivered parcels)
        return Delivery().get_problem(
            dim,
            d,
            current_config["c0"],
            current_config["c1"],
            current_config["d0"],
            current_config["d1"],
            capacity,
        )
    if domain == "rover":
        from benchmarks.rover.problem import Rover

        return Rover().get_problem(dim, d, c)
    if domain == "tdoors":
        from benchmarks.tdoors.problem import TemporalDoors

        c0, c1 = [(0, 0), (10, 0), (0, 10), (5, 5)][c]
        if dim == "3D":
            raise NotImplementedError(
                "Benchmark Temporal Doors is only available in 2D."
            )
        return TemporalDoors().get_problem(d, n_robots, c0=c0, c1=c1)
    if domain == "kitting":
        from benchmarks.kitting.problem import Kitting

        if dim == "3D":
            raise NotImplementedError("Benchmark Kitting is only available in 2D.")
        # n_doors, n_components, n_robots, kit_size, n_kit, max_kit
        return Kitting().get_problem(d, n_robots, kit_size, n_kit)
    if domain == "majsp":
        from benchmarks.majsp.problem import MaJSP

        if dim == "3D":
            raise NotImplementedError("Benchmark MaJSP is only available in 2D.")
        # n_robots, n_pallets, n_treatments = n_doors
        return MaJSP().get_problem(n_robots, n_pallets, d)
    if domain == "driverlog":
        from benchmarks.driverlog.problem import Driverlog

        if dim == "3D":
            raise NotImplementedError("Benchmark Driverlog is only available in 2D.")
        # n_drivers, n_trucks, n_packages = n_doors
        return Driverlog().get_problem(n_drivers, n_robots, d)
    if domain == "floortile":
        from benchmarks.floortile.problem import Floortile

        if dim == "3D":
            raise NotImplementedError("Benchmark Floortile is only available in 2D.")
        # n_obstacles = n_doors, n_robots, n_tiles, n_colors
        return Floortile().get_problem(d, n_robots, n_tiles, n_colors)
    # if domain == "trover":
    #     from benchmarks.trover.problem import TRover
    #     if dim == '3D':
    #         raise NotImplementedError('Benchmark Temporal Rover is only available in 2D.')
    #     # n_robots, d, c
    #     return Rover().get_problem(n_robots, d, c)

    return None
