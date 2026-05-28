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

import bisect
from collections import defaultdict
import math
import os
from typing import Dict, List, Optional, Set, Tuple
import numpy as np
from shapely.geometry import Polygon
from ompl import base as ob
from unified_planning.shortcuts import *
from shapely.affinity import rotate
from tampest.motion.map import Map, Map2D, Map3D
import cv2
from trimesh import transformations
import trimesh
from tampest.motion.motion_planning_data import SupportedTopologicalRefinement


class CollisionChecker:

    def __init__(
        self,
        moving_objects: Dict[int, MovableObject],
        map: Map,
        topological_refinement: SupportedTopologicalRefinement,
        index_map: Dict[int, int],
        delays: Optional[Dict[int, float]] = None,
        sequential_check: Optional[bool] = None,
        trajectories: Optional[Dict[int, list]] = None,
        safety_distance: Optional[float] = None,
        d_max: Optional[List[float]] = None,
        max_radius_bound: Optional[bool] = False,
        all_movable_objects: Optional[Dict[int, MovableObject]] = None,
        all_start_configs: Optional[Dict[int, ConfigurationObject]] = None,
        all_goal_configs: Optional[Dict[int, ConfigurationObject]] = None,
        action_timings: Optional[Dict[int, Tuple[float, float]]] = None,
    ) -> None:

        self.moving_objects = moving_objects
        self.map = map
        self._collision_objects = {}
        self.topological_refinement = topological_refinement
        self.max_radius_bound = max_radius_bound
        self.delays = delays
        self.sequential_check = sequential_check
        self.trajectories = trajectories
        self.safety_distance = safety_distance
        self.all_movable_objects = all_movable_objects
        self.all_start_configs = all_start_configs
        self.all_goal_configs = all_goal_configs
        self.action_timings = action_timings

        self.d_max = []
        self.v_max = []

        # moving_object, start, d_max
        for _, o in self.moving_objects.items():
            self.v_max.append(
                float(o.control_parameters["v_max"])
                if o.control_parameters is not None
                and o.control_parameters["v_max"] is not None
                else None
            )

        self.index_map = index_map
        self.d_max = d_max

    @property
    def collision_objects(self) -> Dict[int, List[MovableObject]]:
        return self._collision_objects

    def get_collision_objects(self) -> Set[MovableObject]:
        return self._collision_objects

    def add_collision_object(self, co: Tuple[int, MovableObject]):
        (index, obj) = co
        if not index in self._collision_objects.keys():
            self._collision_objects[index] = [obj]
        else:
            if obj not in self._collision_objects[index]:
                self._collision_objects[index].append(obj)


class CollisionChecker3D(CollisionChecker):

    def __init__(
        self,
        moving_objects: Dict[int, MovableObject],
        start_configs: Dict[int, ConfigurationObject],
        map: Map3D,
        movable_obstacles: Dict[MovableObject, ConfigurationObject],
        topological_refinement: SupportedTopologicalRefinement,
        index_map: Dict[int, int],
        delays: Optional[Dict[int, float]] = None,
        sequential_check: Optional[bool] = None,
        trajectories: Optional[Dict[int, list]] = None,
        safety_distance: Optional[float] = None,
        d_max: Optional[List[float]] = None,
        max_radius_bound: Optional[bool] = False,
        all_movable_objects: Optional[Dict[int, MovableObject]] = None,
        all_start_configs: Optional[Dict[int, ConfigurationObject]] = None,
        all_goal_configs: Optional[Dict[int, ConfigurationObject]] = None,
        action_timings: Optional[Dict[int, Tuple[float, float]]] = None,
    ) -> None:
        super().__init__(
            moving_objects,
            map,
            topological_refinement,
            index_map,
            delays=delays,
            sequential_check=sequential_check,
            trajectories=trajectories,
            safety_distance=safety_distance,
            d_max=d_max,
            max_radius_bound=max_radius_bound,
            all_movable_objects=all_movable_objects,
            all_start_configs=all_start_configs,
            all_goal_configs=all_goal_configs,
            action_timings=action_timings,
        )

        self.env_mesh = ("env", map.mesh)
        self.movable_obstacles = {obj.name: obj for obj, _ in movable_obstacles.items()}
        self.movable_obstacles_meshes = [
            (obj.name, self.get_obj_mesh(obj, config.configuration))
            for obj, config in movable_obstacles.items()
        ]

        self.start_poses = {}
        self.moving_objects_meshes = {}

        for k, o in self.moving_objects.items():
            moving_object = o
            start_pose = start_configs[k]
            self.start_poses[k] = (
                start_pose.configuration.x,
                start_pose.configuration.y,
                start_pose.configuration.z,
            )
            self.moving_objects_meshes[k] = (
                f"moving_object_{k}",
                self.get_obj_mesh(moving_object),
            )

        self.collision_manager = self.get_collision_manager(
            self.movable_obstacles_meshes
            + [self.env_mesh]
            + list(self.moving_objects_meshes.values())
        )

    def get_obj_mesh(
        self, obj: MovableObject, transform: Optional[Tuple[float]] = None
    ) -> trimesh.Trimesh:
        if os.path.exists(obj.geometric_model):
            if transform is not None:
                if not obj.motion_model == MotionModels.SE3:
                    raise NotImplementedError(
                        f"Motion model {obj.motion_model} not yet supported."
                    )

                T = transformations.translation_matrix(
                    [transform.x, transform.y, transform.z]
                )
                rotation = (transform.rw, transform.rx, transform.ry, transform.rz)
                if all(v == 0 for v in rotation):
                    R = transformations.quaternion_matrix([1, 0, 0, 0])
                else:

                    R = transformations.quaternion_matrix(rotation)

                tf = transformations.concatenate_matrices(T, R)
                return trimesh.load(obj.geometric_model, force="mesh").apply_transform(
                    tf
                )
            else:
                return trimesh.load(obj.geometric_model, force="mesh")
        else:
            raise FileNotFoundError(f"File {obj.geometric_model} not found.")

    def get_collision_manager(
        self, objs: Dict[str, trimesh.Trimesh]
    ) -> trimesh.collision.CollisionManager:
        m = trimesh.collision.CollisionManager()
        for k, v in objs:
            m.add_object(k, v)
        return m

    def isStateValid(self, state: ob.State) -> bool:

        collision = False

        for k, _ in self.moving_objects.items():

            if len(self.index_map) > 1:
                subspace_index = self.index_map[k]
                state = state[subspace_index]

            T = transformations.translation_matrix(
                [state.getX(), state.getY(), state.getZ()]
            )
            R = transformations.quaternion_matrix(
                [
                    state.rotation().w,
                    state.rotation().x,
                    state.rotation().y,
                    state.rotation().z,
                ]
            )
            tf = transformations.concatenate_matrices(T, R)

            colliding, collision_objs = self.collision_manager.in_collision_single(
                self.moving_objects_meshes[k][1], tf, return_names=True
            )

            collision = collision or colliding

            if (
                self.topological_refinement
                in [
                    SupportedTopologicalRefinement.ALL,
                    SupportedTopologicalRefinement.OBS,
                ]
                and collision_objs
            ):
                keys = set(self.movable_obstacles.keys())
                for elem in keys.intersection(collision_objs):
                    self.add_collision_object((k, self.movable_obstacles[elem]))

        return not collision

    def plot_current_state(self, starts, goals):

        import pyvista as pv

        plotter = pv.Plotter()

        # plot map
        env_vertices = self.env_mesh[1].vertices
        env_faces = self.env_mesh[1].faces
        faces = np.hstack((np.full((env_faces.shape[0], 1), 3), env_faces))

        mesh = pv.PolyData(env_vertices, faces)
        plotter.add_mesh(mesh, color="lightgrey", opacity=1.0)

        for k, _ in self.moving_objects.items():
            # plot moving object at its start and goal configuration
            obj_color = tuple(np.random.uniform(range(0, 1), size=3))

            # Start configuration
            obj_start = self.moving_objects_meshes[k][1].copy()
            T = transformations.translation_matrix(
                [
                    starts[k].configuration.x,
                    starts[k].configuration.y,
                    starts[k].configuration.z,
                ]
            )
            rotation = (
                starts[k].configuration.rw,
                starts[k].configuration.rx,
                starts[k].configuration.ry,
                starts[k].configuration.rz,
            )
            if all(v == 0 for v in rotation):
                R = transformations.quaternion_matrix([1, 0, 0, 0])
            else:
                R = transformations.quaternion_matrix(rotation)
            tf = transformations.concatenate_matrices(T, R)
            obj_start.apply_transform(tf)

            start_faces = np.hstack(
                (np.full((obj_start.faces.shape[0], 1), 3), obj_start.faces)
            )
            plotter.add_mesh(
                pv.PolyData(obj_start.vertices, start_faces),
                color=obj_color,
                opacity=1.0,
            )

            # Goal configuration
            obj_goal = self.moving_objects_meshes[k][1].copy()
            T = transformations.translation_matrix(
                [
                    goals[k].configuration.x,
                    goals[k].configuration.y,
                    goals[k].configuration.z,
                ]
            )
            rotation = (
                goals[k].configuration.rw,
                goals[k].configuration.rx,
                goals[k].configuration.ry,
                goals[k].configuration.rz,
            )
            if all(v == 0 for v in rotation):
                R = transformations.quaternion_matrix([1, 0, 0, 0])
            else:
                R = transformations.quaternion_matrix(rotation)
            tf = transformations.concatenate_matrices(T, R)
            obj_goal.apply_transform(tf)
            goal_faces = np.hstack(
                (np.full((obj_goal.faces.shape[0], 1), 3), obj_goal.faces)
            )
            plotter.add_mesh(
                pv.PolyData(obj_goal.vertices, goal_faces), color=obj_color, opacity=1.0
            )

        # Plot movable obstacles
        for _, obs_mesh in self.movable_obstacles_meshes:
            obs_color = np.random.rand(3)

            obs_faces = np.hstack(
                (np.full((obs_mesh.faces.shape[0], 1), 3), obs_mesh.faces)
            )
            plotter.add_mesh(
                pv.PolyData(obs_mesh.vertices, obs_faces), color=obs_color, opacity=1.0
            )

        plotter.show()


class CollisionChecker2D(CollisionChecker):

    def __init__(
        self,
        moving_objects: Dict[int, MovableObject],
        start_configs: Dict[int, ConfigurationObject],
        map: Map,
        movable_obstacles: Dict[MovableObject, ConfigurationObject],
        topological_refinement: SupportedTopologicalRefinement,
        index_map: Dict[int, int],
        delays: Optional[Dict[int, float]] = None,
        sequential_check: Optional[bool] = None,
        trajectories: Optional[Dict[int, list]] = None,
        safety_distance: Optional[float] = None,
        d_max: Optional[List[float]] = None,
        max_radius_bound: Optional[bool] = False,
        all_movable_objects: Optional[Dict[int, MovableObject]] = None,
        all_start_configs: Optional[Dict[int, ConfigurationObject]] = None,
        all_goal_configs: Optional[Dict[int, ConfigurationObject]] = None,
        action_timings: Optional[Dict[int, Tuple[float, float]]] = None,
    ) -> None:
        super().__init__(
            moving_objects,
            map,
            topological_refinement,
            index_map,
            delays=delays,
            sequential_check=sequential_check,
            trajectories=trajectories,
            safety_distance=safety_distance,
            d_max=d_max,
            max_radius_bound=max_radius_bound,
            all_movable_objects=all_movable_objects,
            all_start_configs=all_start_configs,
            all_goal_configs=all_goal_configs,
            action_timings=action_timings,
        )

        self.start_configs = start_configs
        self.start_poses = {}
        self.objects_vertices = {}

        # moving_object, start, d_max
        for k, o in self.moving_objects.items():
            start_pose = start_configs[k]
            self.start_poses[k] = (
                start_pose.configuration.x / map.resolution,
                map.image.size[1] - start_pose.configuration.y / map.resolution,
            )
            self.objects_vertices[k] = [
                (
                    o.footprint[0][0] / map.resolution,
                    o.footprint[0][1] / map.resolution,
                ),
                (
                    o.footprint[1][0] / map.resolution,
                    o.footprint[1][1] / map.resolution,
                ),
                (
                    o.footprint[2][0] / map.resolution,
                    o.footprint[2][1] / map.resolution,
                ),
                (
                    o.footprint[3][0] / map.resolution,
                    o.footprint[3][1] / map.resolution,
                ),
            ]

        self.is_time_space = False
        if self.d_max and len(self.d_max) > 0:
            self.is_time_space = True

        self.fixed_obstacles = self.get_fixed_obstacles(map)
        self.movable_obstacles = {
            k: self.get_polygon_from_config(k, v) for k, v in movable_obstacles.items()
        }

    def get_fixed_obstacles(self, map: Map2D) -> List[Polygon]:

        # Threshold the image of the map
        open_cv_image = np.array(map.image)

        gray = cv2.cvtColor(open_cv_image, cv2.COLOR_BGR2GRAY)

        # binarize image
        _, bw = cv2.threshold(gray, 0, 1, cv2.THRESH_BINARY_INV)

        # find contour
        contours, _ = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Get fix obstacles
        fix_obstacles = [Polygon(c.reshape(-1, 2)) for c in contours]

        return fix_obstacles

    def get_polygon_from_state(self, state: ob.State, k: int) -> Polygon:

        obj_state = state
        if self.is_time_space:
            obj_state = state[0]
        if len(self.moving_objects) > 1:
            key = self.index_map[k]
            obj_state = obj_state[key]
        x = obj_state.getX()
        y = obj_state.getY()
        yaw = obj_state.getYaw()

        points = [(x + sx, y + sy) for (sx, sy) in self.objects_vertices[k]]
        polygon = rotate(Polygon(points), yaw, use_radians=True)

        return polygon

    def get_polygon_from_config(
        self, obj: MovableObject, config: ConfigurationObject
    ) -> Polygon:
        x = config.configuration.x / self.map.resolution
        y = config.configuration.y / self.map.resolution
        yaw = config.configuration.theta

        points = [
            (
                x + obj.footprint[0][0] / self.map.resolution,
                self.map.image.size[1] - y + obj.footprint[0][1] / self.map.resolution,
            ),
            (
                x + obj.footprint[1][0] / self.map.resolution,
                self.map.image.size[1] - y + obj.footprint[1][1] / self.map.resolution,
            ),
            (
                x + obj.footprint[2][0] / self.map.resolution,
                self.map.image.size[1] - y + obj.footprint[2][1] / self.map.resolution,
            ),
            (
                x + obj.footprint[3][0] / self.map.resolution,
                self.map.image.size[1] - y + obj.footprint[3][1] / self.map.resolution,
            ),
        ]

        polygon = rotate(Polygon(points), yaw, use_radians=True)

        return polygon

    def isStateValid(self, si, state: ob.State) -> bool:

        if not si.satisfiesBounds(state):
            return False

        current_time = None
        if self.is_time_space:
            current_time = si.getStateSpace().getStateTime(state)
            if current_time < 0:
                return False

            # Sequential check: shift the sampled time by this robot's start offset
            if self.delays and self.sequential_check:
                current_time += list(self.delays.values())[0]
            # Group MP: pin robots at their start position until their delay elapses
            elif self.delays:
                for i, t in self.delays.items():
                    if t > 0 and current_time <= t + 0.1:
                        x = self.start_configs[i].configuration.x / self.map.resolution
                        y = (
                            self.map.image.size[1]
                            - self.start_configs[i].configuration.y / self.map.resolution
                        )
                        yaw = self.start_configs[i].configuration.theta
                        if (
                            len(self.moving_objects) == 1
                            and self.all_movable_objects
                            and len(self.all_movable_objects) > 1
                        ):
                            i = 0
                        state[0][i].setXY(x, y)
                        state[0][i].setYaw(yaw)

        # bounds based on action max duration
        if self.is_time_space is None or (
            self.d_max and self.v_max and len(self.v_max) != len(self.d_max)
        ):
            raise ValueError(
                "Duration bounds are not set due to the absence of either the maximum duration or the maximum robot velocity."
            )

        if self.max_radius_bound:
            for k, _ in self.moving_objects.items():
                obj_state = state
                if self.is_time_space:
                    obj_state = state[0]
                if (
                    len(self.moving_objects) == 1
                    and self.all_movable_objects
                    and len(self.all_movable_objects) > 1
                ):
                    obj_state = obj_state[0]
                elif len(self.moving_objects) > 1:
                    key = self.index_map[k]
                    obj_state = obj_state[key]
                x = obj_state.getX()
                y = obj_state.getY()

                distance = math.sqrt(
                    (x - self.start_poses[k][0]) ** 2
                    + (y - self.start_poses[k][1]) ** 2
                )
                if distance > self.d_max[k] * self.v_max[k]:
                    return False

        # Build multi-motion-per-object time windows (only when time-space)
        equal_objs = None
        if self.is_time_space:
            equal_objs = defaultdict(list)
            for k, obj in self.moving_objects.items():
                equal_objs[obj].append((k, self.delays.get(k, 0) if self.delays else 0, math.inf))
            for obj in equal_objs:
                equal_objs[obj].sort(key=lambda x: x[1])
                for i in range(1, len(equal_objs[obj])):
                    equal_objs[obj][i - 1] = (
                        equal_objs[obj][i - 1][0],
                        equal_objs[obj][i - 1][1],
                        equal_objs[obj][i][1],
                    )

        for k, obj in self.moving_objects.items():

            # Skip this robot if it is not active at current_time
            if self.is_time_space and equal_objs is not None:
                start_time, end_time = None, None
                for obj_id, st, et in equal_objs[obj]:
                    if k == obj_id:
                        start_time, end_time = st, et
                        break
                if start_time is None:
                    raise Exception("Unmatched moving object in equal_objs.")
                if current_time < start_time or current_time > end_time:
                    continue

            # Get moving object's current polygon
            if (
                len(self.moving_objects) == 1
                and self.all_movable_objects
                and len(self.all_movable_objects) > 1
            ):
                moving = self.get_polygon_from_state(state, 0)
            else:
                moving = self.get_polygon_from_state(state, k)

            # Check against movable obstacles at their fixed positions
            for kobs, v in self.movable_obstacles.items():
                if moving.intersects(v):
                    if self.topological_refinement in [
                        SupportedTopologicalRefinement.ALL,
                        SupportedTopologicalRefinement.OBS,
                    ]:
                        self.add_collision_object((k, kobs))
                    return False

            # Sequential check: collide against other robots that are stationary
            if self.all_movable_objects and self.sequential_check:
                l = {}
                for k_all, v in self.all_movable_objects.items():
                    start_t = self.action_timings[k_all][0]
                    end_t = start_t + self.action_timings[k_all][1]
                    l.setdefault(v, []).append(
                        (start_t, end_t, self.all_start_configs[k_all], self.all_goal_configs[k_all])
                    )

                for v, times_configs in l.items():
                    if v == obj:
                        continue
                    times_configs.sort(key=lambda x: x[0])
                    t0 = 0
                    last_config = None
                    static_intervals = []
                    for (start_t, end_t, start_cfg, goal_cfg) in times_configs:
                        if last_config is not None:
                            assert last_config == start_cfg
                        static_intervals.append((t0, start_t, start_cfg))
                        t0 = end_t
                        last_config = goal_cfg
                    static_intervals.append((t0, math.inf, last_config))

                    for st, et, config in static_intervals:
                        if st <= current_time <= et:
                            other_static = self.get_polygon_from_config(v, config)
                            if moving.intersects(other_static):
                                return False
                            break

            # Sequential check: collide against pre-computed trajectories with safety margin
            if self.is_time_space and self.sequential_check and self.trajectories:
                time_margin = self.safety_distance / max(self.v_max[k] if k < len(self.v_max) else 1.0, 1e-6)
                left = current_time - time_margin
                right = current_time + time_margin

                for oth_obj_id, oth_obj_traj in self.trajectories.items():
                    times = [oth_obj_traj[0][5]]
                    for idx in range(1, len(oth_obj_traj)):
                        times.append(times[-1] + oth_obj_traj[idx][5])

                    i1 = bisect.bisect_left(times, left)
                    i2 = bisect.bisect_right(times, right)

                    for state_to_check in oth_obj_traj[i1:i2]:
                        sx, sy, syaw = state_to_check[0], state_to_check[1], state_to_check[2]
                        pts = [(sx + dx, sy + dy) for (dx, dy) in self.objects_vertices[oth_obj_id]]
                        poly_to_check = rotate(Polygon(pts), syaw, use_radians=True)
                        if moving.intersects(poly_to_check):
                            return False

            # Group MP: check against other moving robots in the same group
            if not self.sequential_check:
                for other_k, other_obj in self.moving_objects.items():
                    if other_obj == obj:
                        continue
                    if self.is_time_space and equal_objs is not None:
                        other_active = False
                        for obj_id, st, et in equal_objs[other_obj]:
                            if other_k == obj_id and st <= current_time <= et:
                                other_active = True
                                break
                        if not other_active:
                            continue
                    other_moving = self.get_polygon_from_state(state, other_k)
                    if moving.intersects(other_moving):
                        return False

            # Check against fixed obstacles
            for fo in self.fixed_obstacles:
                if moving.intersects(fo):
                    return False

        return True

    def plot_current_state(
        self,
        start_configs: Dict[int, ConfigurationObject],
        goal_configs: Dict[int, ConfigurationObject],
    ):
        from matplotlib import pyplot as plt
        from matplotlib.patches import Polygon as MplPolygon

        _, ax = plt.subplots()

        ax.imshow(self.map.image)

        for obs in self.fixed_obstacles:
            ax.add_patch(
                MplPolygon(list(obs.exterior.coords), color="black", alpha=0.3)
            )

        if len(start_configs) != len(goal_configs):
            raise ValueError(
                "Missing data - different number of start and end configs."
            )

        for i in range(len(start_configs)):
            start = self.get_polygon_from_config(
                self.moving_objects[i], start_configs[i]
            )
            ax.add_patch(
                MplPolygon(list(start.exterior.coords), color="green", alpha=0.3)
            )

            goal = self.get_polygon_from_config(self.moving_objects[i], goal_configs[i])
            ax.add_patch(MplPolygon(list(goal.exterior.coords), color="red", alpha=0.3))

        for _, v in self.movable_obstacles.items():
            ax.add_patch(MplPolygon(list(v.exterior.coords), color="blue", alpha=0.3))

        plt.show()
