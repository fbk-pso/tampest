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

import os
from typing import Optional
import yaml


def get_problem(
    domain,
    dim,
    d,
    *,
    samp=False,
    c: Optional[int] = None,
    n_robots: Optional[int] = 1,
    capacity: Optional[int] = None,
    kit_size: Optional[int] = None,
    n_kit: Optional[int] = None,
    n_pallets: Optional[int] = None,
    n_drivers: Optional[int] = None,
    n_tiles: Optional[int] = None,
    n_colors: Optional[int] = None,
    n_shelves: Optional[int] = None,
    n_components: Optional[int] = None,
    use_external_locations: Optional[int] = None,
    use_fluents: bool = False,
    use_resources: bool = False,
    add_metric: bool = False,
    force_activity_duration: bool = False,
    force_sequential_schedule: bool = False,
):
    if domain == "doors":
        if dim == "3D":
            raise NotImplementedError("Benchmark Doors is only available in 2D.")
        from benchmarks.doors.problem import Doors

        c0, c1 = [(0, 0), (10, 0), (0, 10), (5, 5)][c]
        return Doors().get_problem(d, c0=c0, c1=c1)
    elif domain == "maze":
        from benchmarks.maze.problem import Maze

        return Maze().get_problem(dim, d, c)
    elif domain == "delivery":
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
    elif domain == "rover":
        from benchmarks.rover.problem import Rover

        return Rover().get_problem(dim, d, c)
    elif domain == "tdoors":
        if dim == "3D":
            raise NotImplementedError(
                "Benchmark Temporal Doors is only available in 2D."
            )
        c0, c1 = [(0, 0), (10, 0), (0, 10), (5, 5)][c]
        if samp:
            from benchmarks.tdoors.scheduling_problem import TemporalDoors

            return TemporalDoors().get_problem(
                d,
                n_robots,
                c0=c0,
                c1=c1,
                use_fluents=use_fluents,
                use_resources=use_resources,
                force_activity_duration=force_activity_duration,
            )
        else:
            from benchmarks.tdoors.problem import TemporalDoors

            return TemporalDoors().get_problem(d, n_robots, c0=c0, c1=c1)
    elif domain == "kitting":
        if dim == "3D":
            raise NotImplementedError("Benchmark Kitting is only available in 2D.")
        # n_doors, n_components, n_robots, kit_size, n_kit, max_kit
        if samp:
            from benchmarks.kitting.scheduling_problem import Kitting

            return Kitting().get_problem(
                d,
                n_robots,
                kit_size,
                n_kit,
                use_fluents=use_fluents,
                use_resources=use_resources,
                force_activity_duration=force_activity_duration,
            )
        else:
            from benchmarks.kitting.problem import Kitting

            return Kitting().get_problem(d, n_robots, kit_size, n_kit)
    elif domain in ("majsp", "jsp"):
        if dim == "3D":
            raise NotImplementedError("Benchmark MaJSP is only available in 2D.")
        # n_robots, n_pallets, n_treatments = n_doors
        if samp:
            from benchmarks.majsp.scheduling_problem import JSP

            return JSP().get_problem(
                n_robots,
                n_pallets,
                d,
                use_fluents=use_fluents,
                use_resources=use_resources,
                force_activity_duration=force_activity_duration,
                add_metric=add_metric,
                force_sequential_schedule=force_sequential_schedule,
            )
        else:
            from benchmarks.majsp.problem import MaJSP

            return MaJSP().get_problem(n_robots, n_pallets, d)
    elif domain == "logistics":
        if dim == "3D":
            raise NotImplementedError("Benchmark Logistics is only available in 2D.")
        if samp:
            from benchmarks.logistics.scheduling_problem import Logistics

            assert d in [0, 1]
            return Logistics().get_problem(
                n_robots,
                n_shelves,
                n_components,
                use_doors=(d == 1),
                use_external_locations=use_external_locations,
                use_fluents=use_fluents,
                use_resources=use_resources,
                force_activity_duration=force_activity_duration,
                add_metric=add_metric,
                force_sequential_schedule=force_sequential_schedule,
            )
        else:
            raise NotImplementedError

    elif domain == "navigation":
        if dim == "3D":
            raise NotImplementedError("Benchmark navigation is only available in 2D.")
        if samp:
            from benchmarks.navigation.scheduling_problem import Navigation

            return Navigation().get_problem(
                n_robots,
                use_fluents=use_fluents,
                add_metric=add_metric,
                force_sequential_schedule=force_sequential_schedule,
            )
        else:
            raise NotImplementedError

    elif domain == "driverlog":
        if dim == "3D":
            raise NotImplementedError("Benchmark Driverlog is only available in 2D.")
        # n_drivers, n_trucks, n_packages = n_doors
        from benchmarks.driverlog.problem import Driverlog

        return Driverlog().get_problem(n_drivers, n_robots, d)
    elif domain == "floortile":
        if dim == "3D":
            raise NotImplementedError("Benchmark Floortile is only available in 2D.")
        # n_obstacles = n_doors, n_robots, n_tiles, n_colors
        from benchmarks.floortile.problem import Floortile

        return Floortile().get_problem(d, n_robots, n_tiles, n_colors)
    else:
        raise NotImplementedError(
            f"Benchmark {domain} is not implemented. Please check the domain name."
        )
