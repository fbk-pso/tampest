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
import os
from typing import Dict, List, Set, Tuple
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
        d_max: Optional[List[float]] = None,
        max_radius_bound: Optional[bool] = False,
    ) -> None:

        self.moving_objects = moving_objects
        self.map = map
        self._collision_objects = {}
        self.topological_refinement = topological_refinement
        self.max_radius_bound = max_radius_bound

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
        d_max: Optional[List[float]] = None,
        max_radius_bound: Optional[bool] = False,
    ) -> None:
        super().__init__(
            moving_objects,
            map,
            topological_refinement,
            index_map,
            d_max,
            max_radius_bound,
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
                start_pose.configuration[0],
                start_pose.configuration[1],
                start_pose.configuration[2],
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
                # Assumption for SE3: (x, y, z, rw, rx, ry, rz)
                T = transformations.translation_matrix(
                    [transform[0], transform[1], transform[2]]
                )
                rotation = (transform[3], transform[4], transform[5], transform[6])
                if all(v == 0 for v in rotation):
                    R = transformations.quaternion_matrix([1, 0, 0, 0])
                else:
                    R = transformations.rotation_matrix(
                        transform[6], [transform[3], transform[4], transform[5]]
                    )
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

        for k, o in self.moving_objects.items():
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
                self.moving_objects_meshs[k][1], tf, return_names=True
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

    def plot_current_state(
        self,
        starts: Dict[int, ConfigurationObject],
        goals: Dict[int, ConfigurationObject],
    ):
        from mayavi import mlab

        # plot map
        env_vertices = self.env_mesh[1].vertices
        env_indices = self.env_mesh[1].faces

        mlab.figure()
        mlab.triangular_mesh(
            env_vertices[:, 0],
            env_vertices[:, 1],
            env_vertices[:, 2],
            env_indices,
            opacity=1.0,
            color=(0.5, 0.5, 0.5),
        )

        for k, _ in self.moving_objects.items():
            # plot moving object at its start and goal configuration
            obj_color = tuple(np.random.uniform(range(0, 1), size=3))

            # obj at start - Assumption for SE3: (x, y, z, rx, ry, rz, rangle)
            obj_at_start = self.moving_objects_meshes[k][1].copy()
            T = transformations.translation_matrix(
                [
                    starts[k].configuration.x,
                    starts[k].configuration.y,
                    starts[k].configuration.z,
                ]
            )
            rotation = (
                starts[k].configuration.rx,
                starts[k].configuration.ry,
                starts[k].configuration.rz,
                starts[k].configuration.rw,
            )
            if all(v == 0 for v in rotation):
                R = transformations.quaternion_matrix([1, 0, 0, 0])
            else:
                R = transformations.rotation_matrix(
                    starts[k].configuration.rw,
                    [
                        starts[k].configuration.rx,
                        starts[k].configuration.ry,
                        starts[k].configuration.rz,
                    ],
                )
            tf = transformations.concatenate_matrices(T, R)
            obj_at_start.apply_transform(tf)

            mlab.triangular_mesh(
                obj_at_start.vertices[:, 0],
                obj_at_start.vertices[:, 1],
                obj_at_start.vertices[:, 2],
                obj_at_start.faces,
                opacity=1.0,
                color=obj_color,
            )

            # obj at goal - Assumption for SE3: (x, y, z, rx, ry, rz, rangle)
            obj_at_goal = self.moving_object_mesh[1].copy()
            T = transformations.translation_matrix(
                [
                    goals[k].configuration.x,
                    goals[k].configuration.y,
                    goals[k].configuration.z,
                ]
            )
            rotation = (
                goals[k].configuration.rx,
                goals[k].configuration.ry,
                goals[k].configuration.rz,
                goals[k].configuration.rw,
            )
            if all(v == 0 for v in rotation):
                R = transformations.quaternion_matrix([1, 0, 0, 0])
            else:
                R = transformations.rotation_matrix(
                    goals[k].configuration.rw,
                    [
                        goals[k].configuration.rx,
                        goals[k].configuration.ry,
                        goals[k].configuration.rz,
                    ],
                )
            tf = transformations.concatenate_matrices(T, R)
            obj_at_goal.apply_transform(tf)

            mlab.triangular_mesh(
                obj_at_goal.vertices[:, 0],
                obj_at_goal.vertices[:, 1],
                obj_at_goal.vertices[:, 2],
                obj_at_goal.faces,
                opacity=1.0,
                color=obj_color,
            )

        # plot movable obstacles at their current configuration
        obs_color = tuple(np.random.uniform(range(0, 1), size=3))

        for _, obs_mesh in self.movable_obstacles_meshes:
            mlab.triangular_mesh(
                obs_mesh.vertices[:, 0],
                obs_mesh.vertices[:, 1],
                obs_mesh.vertices[:, 2],
                obs_mesh.faces,
                opacity=1.0,
                color=obs_color,
            )

        mlab.show()


class CollisionChecker2D(CollisionChecker):

    def __init__(
        self,
        moving_objects: Dict[int, MovableObject],
        start_configs: Dict[int, ConfigurationObject],
        map: Map,
        movable_obstacles: Dict[MovableObject, ConfigurationObject],
        topological_refinement: SupportedTopologicalRefinement,
        index_map: Dict[int, int],
        d_max: Optional[List[float]] = None,
        max_radius_bound: Optional[bool] = False,
    ) -> None:
        super().__init__(
            moving_objects,
            map,
            topological_refinement,
            index_map,
            d_max,
            max_radius_bound,
        )

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

    def get_polygon_from_state(self, state: ob.State, k: int, n_robots: int) -> Polygon:

        obj_state = state
        if self.is_time_space:
            obj_state = state[0]
        if n_robots > 1:
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

        if self.is_time_space:
            if si.getStateSpace().getStateTime(state) < 0:
                return False

        # bounds based on action max duration
        if self.is_time_space is None or (
            self.d_max and self.v_max and len(self.v_max) != len(self.d_max)
        ):
            raise ValueError(
                "Duration bounds are not set due to the absence of either the maximum duration or the maximum robot velocity."
            )

        if self.max_radius_bound:
            # Check if the distance is less than or equal to the radius
            for k, _ in self.moving_objects.items():
                obj_state = state
                if self.is_time_space:
                    obj_state = state[0]
                if len(self.moving_objects.values()) > 1:
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

        # Find intersections with fix obstacles

        collision_found = False

        for k, _ in self.moving_objects.items():
            moving = self.get_polygon_from_state(
                state, k, len(self.moving_objects)
            )  # Get moving object current state
            for fo in self.fixed_obstacles:
                if moving.intersects(fo):
                    collision_found = True

            # Find intersections with movable objects
            for kobs, v in self.movable_obstacles.items():
                if moving.intersects(v):
                    collision_found = True
                    if self.topological_refinement in [
                        SupportedTopologicalRefinement.ALL,
                        SupportedTopologicalRefinement.OBS,
                    ]:
                        self.add_collision_object((k, kobs))

        return not collision_found

    def plot_current_state(
        self,
        start_configs: Dict[int, ConfigurationObject],
        goal_configs: Dict[int, ConfigurationObject],
    ):
        from descartes import PolygonPatch
        from matplotlib import pyplot as plt

        _, ax = plt.subplots()

        ax.imshow(self.map.image)

        for obs in self.fixed_obstacles:
            ax.add_patch(PolygonPatch(obs, alpha=0))

        if len(start_configs) != len(goal_configs):
            raise ValueError(
                "Missing data - different number of start and end configs."
            )

        for i in range(len(start_configs)):
            start = self.get_polygon_from_config(
                self.moving_objects[i], start_configs[i]
            )
            ax.add_patch(PolygonPatch(start, alpha=0.1))

            goal = self.get_polygon_from_config(self.moving_objects[i], goal_configs[i])
            ax.add_patch(PolygonPatch(goal, alpha=0.1))

        for _, v in self.movable_obstacles.items():
            ax.add_patch(PolygonPatch(v, alpha=1.0))

        plt.show()
