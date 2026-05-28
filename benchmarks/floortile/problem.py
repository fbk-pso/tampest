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

import itertools
import math
import random
import unified_planning as up
from unified_planning.shortcuts import *
import os

random.seed(23)

SETUP_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "configs/2D")
MAP_FILE = os.path.join(SETUP_PATH, "maps/map.yaml")


class Floortile:
    def __init__(self, tmpdirname=None) -> None:
        if tmpdirname:
            self.dirname = tmpdirname
        else:
            self.dirname = SETUP_PATH

        self.footprint = [(-1.0, 0.5), (1.0, 0.5), (1.0, -0.5), (-1.0, -0.5)]

    def SE2Control(q, u, qdot):
        robot_lenght = math.dist(self.footprint[0], self.footprint[1])
        theta = q[2]
        qdot[0] = u[0] * math.cos(theta)
        qdot[1] = u[0] * math.sin(theta)
        qdot[2] = u[0] * math.tan(u[1]) / robot_lenght

    def find_neighbors(self, grid):
        rows = len(grid)
        cols = len(grid[0]) if rows > 0 else 0
        neighbors = {}

        for r in range(rows):
            for c in range(cols):
                current_point = grid[r][c]
                neighbors[current_point] = {
                    "down": grid[r - 1][c] if r > 0 else None,
                    "up": grid[r + 1][c] if r < rows - 1 else None,
                    "left": grid[r][c - 1] if c > 0 else None,
                    "right": grid[r][c + 1] if c < cols - 1 else None,
                }

        return neighbors

    def get_points(self):
        map_size = 40
        start_at = 5
        grid_spacing = 10

        grid_points = {}

        for x in range(start_at, map_size, grid_spacing):
            grid_points[(x / start_at - 1) / 2] = []
            for y in range(start_at, map_size, grid_spacing):
                grid_points[(x / start_at - 1) / 2].append(SE2(x, y, 0))

        return grid_points

    def get_problem(
        self, n_obstacles: int, n_robots: int, n_tiles_to_paint: int, n_colors: int
    ) -> up.model.Problem:

        problem = Problem("Floortile")

        occ_map = OccupancyMap(MAP_FILE, SE2(0, 0, 0))

        MovingObject = MovableType("moving_object")
        Tile = ConfigurationType("tile", occ_map, ConfigurationKind.SE2)
        Color = UserType("color")

        at = Fluent("at", Tile, mo=MovingObject)
        problem.add_fluent(at)

        up = Fluent("up", BoolType(), x=Tile, y=Tile)
        problem.add_fluent(up, default_initial_value=False)

        down = Fluent("down", BoolType(), x=Tile, y=Tile)
        problem.add_fluent(down, default_initial_value=False)

        right = Fluent("right", BoolType(), x=Tile, y=Tile)
        problem.add_fluent(right, default_initial_value=False)

        left = Fluent("left", BoolType(), x=Tile, y=Tile)
        problem.add_fluent(left, default_initial_value=False)

        clear = Fluent("clear", BoolType(), x=Tile)
        problem.add_fluent(clear, default_initial_value=True)

        painted = Fluent("painted", BoolType(), x=Tile, c=Color)
        problem.add_fluent(painted, default_initial_value=False)

        robot_has = Fluent("robot_has", BoolType(), mo=MovingObject, x=Color)
        problem.add_fluent(robot_has, default_initial_value=False)

        available_color = Fluent("available_color", BoolType(), c=Color)
        problem.add_fluent(available_color, default_initial_value=False)

        min_duration = Fluent(
            "min_duration", RealType(), mo=MovingObject, x=Tile, y=Tile
        )
        problem.add_fluent(min_duration, default_initial_value=0)
        max_duration = Fluent(
            "max_duration", RealType(), mo=MovingObject, x=Tile, y=Tile
        )
        problem.add_fluent(max_duration, default_initial_value=0)

        moving = Fluent("moving", BoolType(), mo=MovingObject)
        problem.add_fluent(moving, default_initial_value=False)

        v_min = [1.0 for i in range(n_robots + n_obstacles)]
        v_max = [20.0 for i in range(n_robots + n_obstacles)]
        phi_min = [-1 for i in range(n_robots + n_obstacles)]
        phi_max = [1 for i in range(n_robots + n_obstacles)]

        robots = [
            MovableObject(
                "robot%s" % i,
                MovingObject,
                footprint=self.footprint,
                motion_model=MotionModels.SE2,
                control_model=self.SE2Control,
                control_parameters={
                    "v_min": v_min[i],
                    "v_max": v_max[i],
                    "phi_min": phi_min[i],
                    "phi_max": phi_max[i],
                },
            )
            for i in range(n_robots)
        ]
        problem.add_objects(robots)

        obstacles = [
            MovableObject(
                "obs%s" % i,
                MovingObject,
                footprint=self.footprint,
                motion_model=MotionModels.SE2,
                control_model=self.SE2Control,
                control_parameters={
                    "v_min": v_min[i],
                    "v_max": v_max[i],
                    "phi_min": phi_min[i],
                    "phi_max": phi_max[i],
                },
            )
            for i in range(n_robots, n_obstacles)
        ]
        problem.add_objects(obstacles)

        colors = [Object("color%s" % i, Color) for i in range(n_colors)]
        problem.add_objects(colors)
        change_color = DurativeAction(
            "change_color", mo=MovingObject, c=Color, c2=Color
        )
        mo = change_color.parameter("mo")
        c = change_color.parameter("c")
        c2 = change_color.parameter("c2")
        change_color.set_fixed_duration(5)
        change_color.add_condition(StartTiming(), robot_has(mo, c))
        change_color.add_condition(StartTiming(), available_color(c2))
        change_color.add_effect(StartTiming(), robot_has(mo, c), False)
        change_color.add_effect(EndTiming(), robot_has(mo, c2), True)
        change_color.add_effect(EndTiming(), available_color(c2), False)
        problem.add_action(change_color)

        paint_up = DurativeAction("paint_up", mo=MovingObject, x=Tile, y=Tile, c=Color)
        mo = paint_up.parameter("mo")
        y = paint_up.parameter("y")
        x = paint_up.parameter("x")
        c = paint_up.parameter("c")
        paint_up.set_fixed_duration(2)
        paint_up.add_condition(StartTiming(), robot_has(mo, c))
        paint_up.add_condition(StartTiming(), Equals(at(mo), x))
        paint_up.add_condition(StartTiming(), up(y, x))
        paint_up.add_condition(StartTiming(), clear(y))
        paint_up.add_effect(StartTiming(), clear(y), False)
        paint_up.add_effect(EndTiming(), painted(y, c), True)
        paint_up.add_effect(EndTiming(), clear(y), True)
        problem.add_action(paint_up)

        paint_down = DurativeAction(
            "paint_down", mo=MovingObject, x=Tile, y=Tile, c=Color
        )
        mo = paint_down.parameter("mo")
        y = paint_down.parameter("y")
        x = paint_down.parameter("x")
        c = paint_down.parameter("c")
        paint_down.set_fixed_duration(2)
        paint_down.add_condition(StartTiming(), robot_has(mo, c))
        paint_down.add_condition(StartTiming(), Equals(at(mo), x))
        paint_down.add_condition(StartTiming(), down(y, x))
        paint_down.add_condition(StartTiming(), clear(y))
        paint_down.add_effect(StartTiming(), clear(y), False)
        paint_down.add_effect(EndTiming(), painted(y, c), True)
        paint_down.add_effect(EndTiming(), clear(y), True)
        problem.add_action(paint_down)

        move_up = DurativeMotionAction("move_up", mo=MovingObject, x=Tile, y=Tile)
        mo = move_up.parameter("mo")
        x = move_up.parameter("x")
        y = move_up.parameter("y")
        # move_up.set_fixed_duration(3)
        move_up.set_duration_constraint(
            DurationInterval(min_duration(mo, x, y), max_duration(mo, x, y))
        )
        move_up.add_condition(StartTiming(), Equals(at(mo), x))
        move_up.add_condition(StartTiming(), up(y, x))
        move_up.add_condition(StartTiming(), clear(y))
        move_up.add_condition(StartTiming(), Not(moving(mo)))
        move_up.add_effect(StartTiming(), moving(mo), True)
        move_up.add_effect(EndTiming(), moving(mo), False)
        move_up.add_effect(StartTiming(), at(mo), y)
        move_up.add_effect(StartTiming(), clear(y), False)
        move_up.add_effect(StartTiming(), clear(x), True)
        move_up.add_motion_constraint(
            Waypoints(mo, x, [y], {o: at(o) for o in obstacles})
        )
        problem.add_action(move_up)

        move_down = DurativeMotionAction("move_down", mo=MovingObject, x=Tile, y=Tile)
        mo = move_down.parameter("mo")
        x = move_down.parameter("x")
        y = move_down.parameter("y")
        # move_down.set_fixed_duration(1)
        move_down.set_duration_constraint(
            DurationInterval(min_duration(mo, x, y), max_duration(mo, x, y))
        )
        move_down.add_condition(StartTiming(), Equals(at(mo), x))
        move_down.add_condition(StartTiming(), down(y, x))
        move_down.add_condition(StartTiming(), clear(y))
        move_down.add_condition(StartTiming(), Not(moving(mo)))
        move_down.add_effect(StartTiming(), moving(mo), True)
        move_down.add_effect(EndTiming(), moving(mo), False)
        move_down.add_effect(StartTiming(), at(mo), y)
        move_down.add_effect(StartTiming(), clear(y), False)
        move_down.add_effect(StartTiming(), clear(x), True)
        move_down.add_motion_constraint(
            Waypoints(mo, x, [y], {o: at(o) for o in obstacles})
        )
        problem.add_action(move_down)

        move_right = DurativeMotionAction("move_right", mo=MovingObject, x=Tile, y=Tile)
        mo = move_right.parameter("mo")
        x = move_right.parameter("x")
        y = move_right.parameter("y")
        # move_right.set_fixed_duration(1)
        move_right.set_duration_constraint(
            DurationInterval(min_duration(mo, x, y), max_duration(mo, x, y))
        )
        move_right.add_condition(StartTiming(), Equals(at(mo), x))
        move_right.add_condition(StartTiming(), right(y, x))
        move_right.add_condition(StartTiming(), clear(y))
        move_right.add_condition(StartTiming(), Not(moving(mo)))
        move_right.add_effect(StartTiming(), moving(mo), True)
        move_right.add_effect(EndTiming(), moving(mo), False)
        move_right.add_effect(StartTiming(), at(mo), y)
        move_right.add_effect(StartTiming(), clear(y), False)
        move_right.add_effect(StartTiming(), clear(x), True)
        move_right.add_motion_constraint(
            Waypoints(mo, x, [y], {o: at(o) for o in obstacles})
        )
        problem.add_action(move_right)

        move_left = DurativeMotionAction(
            "move_left", mo=MovingObject, x=Tile, y=Tile
        )  # durative motion actions / motion constraint obstacle_at
        mo = move_left.parameter("mo")
        x = move_left.parameter("x")
        y = move_left.parameter("y")
        # move_left.set_fixed_duration(1)
        move_left.set_duration_constraint(
            DurationInterval(min_duration(mo, x, y), max_duration(mo, x, y))
        )
        move_left.add_condition(StartTiming(), Equals(at(mo), x))
        move_left.add_condition(StartTiming(), left(y, x))
        move_left.add_condition(StartTiming(), clear(y))
        move_left.add_condition(StartTiming(), Not(moving(mo)))
        move_left.add_effect(StartTiming(), moving(mo), True)
        move_left.add_effect(EndTiming(), moving(mo), False)
        move_left.add_effect(StartTiming(), at(mo), y)
        move_left.add_effect(StartTiming(), clear(y), False)
        move_left.add_effect(StartTiming(), clear(x), True)
        move_left.add_motion_constraint(
            Waypoints(mo, x, [y], {o: at(o) for o in obstacles})
        )
        problem.add_action(move_left)

        points = self.get_points()
        points_list = list(itertools.chain(*self.get_points().values()))

        tiles = [
            ConfigurationObject("o%s" % i, Tile, points_list[i])
            for i in range(len(points_list))
        ]
        problem.add_objects(tiles)

        neighbors = self.find_neighbors(points)
        for k, v in neighbors.items():

            current_tile = [t for t in tiles if t.configuration == k][0]

            if [t for t in tiles if t.configuration == v["up"]]:
                up_tile = [t for t in tiles if t.configuration == v["up"]][0]
                problem.set_initial_value(up(up_tile, current_tile), True)

            if [t for t in tiles if t.configuration == v["down"]]:
                down_tile = [t for t in tiles if t.configuration == v["down"]][0]
                problem.set_initial_value(down(down_tile, current_tile), True)

            if [t for t in tiles if t.configuration == v["right"]]:
                right_tile = [t for t in tiles if t.configuration == v["right"]][0]
                problem.set_initial_value(right(right_tile, current_tile), True)

            if [t for t in tiles if t.configuration == v["left"]]:
                left_tile = [t for t in tiles if t.configuration == v["left"]][0]
                problem.set_initial_value(left(left_tile, current_tile), True)

        obs_locs = random.sample(tiles, n_obstacles)

        for i in range(len(obstacles)):
            problem.set_initial_value(at(obstacles[i]), obs_locs[i])
            problem.set_initial_value(clear(obs_locs[i]), False)

        robot_locs = random.sample(sorted(set(tiles) - set(obs_locs), key=lambda x: x.name), n_robots)

        all_configs = list(itertools.permutations(tiles, 2))

        assigned_colors = []

        for i in range(len(robots)):
            problem.set_initial_value(at(robots[i]), robot_locs[i])
            problem.set_initial_value(clear(robot_locs[i]), False)

            color = random.choice(colors)
            problem.set_initial_value(robot_has(robots[i], color), True)
            assigned_colors.append(color)

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
                    max_duration(robots[i], all_configs[j][0], all_configs[j][1]),
                    dist / v_min[i],
                )
                problem.set_initial_value(
                    min_duration(robots[i], all_configs[j][0], all_configs[j][1]),
                    dist / v_max[i],
                )

        for c in list(set(colors) - set(assigned_colors)):
            problem.set_initial_value(available_color(c), True)

        tiles_to_paint = random.sample(tiles, n_tiles_to_paint)

        for t in tiles_to_paint:
            problem.add_goal(painted(t, random.choice(colors)))

        return problem
