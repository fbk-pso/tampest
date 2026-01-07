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

from typing import Optional, List, Dict, Union
from PIL import Image
import os
import math
from matplotlib import patches
from matplotlib import pyplot as plt
from matplotlib.cm import get_cmap
from matplotlib.patches import Polygon as MplPolygon
from mayavi import mlab
import numpy as np
from trimesh import transformations
import trimesh
from tampest.motion.collision_checker import CollisionChecker2D, CollisionChecker3D
from tampest.motion.map import Map2D, Map3D
from unified_planning.shortcuts import ConfigurationObject, MovableObject, MotionModels
from scipy.spatial import ConvexHull
from shapely.geometry import MultiPolygon

from unified_planning.plans.time_triggered_plan import TimeTriggeredPlan
from unified_planning.plans.sequential_plan import SequentialPlan


def plot_reachability_data(
    map: Union[Map2D, Map3D],
    motion_model: MotionModels,
    *,
    hull: Optional[ConvexHull] = None,
    points: Optional[np.ndarray] = None,
    start_poses: Optional[List[ConfigurationObject]],
    goal_poses: Optional[List[ConfigurationObject]],
    obstacles: Optional[Dict[MovableObject, ConfigurationObject]] = None,
    cc: Optional[CollisionChecker2D] = None,
    unreachable_configurations: Optional[List[ConfigurationObject]] = None,
):

    if motion_model == MotionModels.REEDSSHEPP or motion_model == MotionModels.SE2:
        plot_2d_reachability_data(
            map,
            hull=hull,
            points=points,
            start_poses=start_poses,
            goal_poses=goal_poses,
            obstacles=obstacles,
            cc=cc,
            unreachable_configurations=unreachable_configurations,
        )
    elif motion_model == MotionModels.SE3:
        plot_3d_reachability_data(
            map,
            hull=hull,
            points=points,
            start_poses=start_poses,
            goal_poses=goal_poses,
            obstacles=obstacles,
            cc=cc,
            unreachable_configurations=unreachable_configurations,
        )
    else:
        raise NotImplementedError


def plot_3d_reachability_data(
    map: Map3D,
    *,
    hull: Optional[ConvexHull] = None,
    points: Optional[np.ndarray] = None,
    start_poses: Optional[List[ConfigurationObject]],
    goal_poses: Optional[List[ConfigurationObject]],
    obstacles: Optional[Dict[MovableObject, ConfigurationObject]] = None,
    cc: Optional[CollisionChecker3D] = None,
    unreachable_configurations: Optional[List[ConfigurationObject]] = None,
):

    mlab.figure()

    map_vertices = map.mesh.vertices
    map_indices = map.mesh.faces
    mlab.triangular_mesh(
        map_vertices[:, 0],
        map_vertices[:, 1],
        map_vertices[:, 2],
        map_indices,
        opacity=1.0,
        color=(0.5, 0.5, 0.5),
    )  # representation='wireframe'

    if hull:
        mlab.triangular_mesh(
            hull.points[:, 0], hull.points[:, 1], hull.points[:, 2], hull.simplices
        )

    if points.any():
        mlab.points3d(points[:, 0], points[:, 1], points[:, 2], scale_factor=1.0)

    for s in start_poses:
        mlab.points3d(
            s.configuration.x,
            s.configuration.y,
            s.configuration.theta,
            scale_factor=1.0,
            color=(0, 1, 0),
        )  # green = (0, 1, 0)(RGB normalized)

    for g in goal_poses:
        mlab.points3d(
            g.configuration.x,
            g.configuration.y,
            g.configuration.theta,
            scale_factor=1.0,
            color=(0, 1, 0),
        )  # green = (0, 1, 0)(RGB normalized)

    if obstacles:
        for k, v in obstacles.items():
            obstacle_mesh = cc.get_obj_mesh(k, v.configuration)
            obstacle_vertices = obstacle_mesh.vertices
            obstacle_indices = obstacle_mesh.faces
            mlab.triangular_mesh(
                obstacle_vertices[:, 0],
                obstacle_vertices[:, 1],
                obstacle_vertices[:, 2],
                obstacle_indices,
                opacity=1.0,
                color=(1, 0, 0),
            )  # red = (1, 0, 0) (RGB normalized)

    if unreachable_configurations:
        for u in unreachable_configurations:
            mlab.points3d(
                u.configuration.x,
                u.configuration.y,
                u.configuration.theta,
                scale_factor=1.0,
                color=(1, 0, 0),
            )  # red = (1, 0, 0)(RGB normalized)

    plt.show()


def plot_2d_reachability_data(
    map: Map2D,
    *,
    hull: Dict[int, Optional[ConvexHull]] = None,
    points: Dict[int, Optional[np.ndarray]] = None,
    start_poses: Dict[int, Optional[List[ConfigurationObject]]],
    goal_poses: Dict[int, Optional[List[ConfigurationObject]]],
    obstacles: Dict[int, Optional[Dict[MovableObject, ConfigurationObject]]] = None,
    cc: Optional[CollisionChecker2D] = None,
    unreachable_configurations: Dict[int, Optional[List[ConfigurationObject]]] = None,
):
    _, ax = plt.subplots()
    ax.imshow(map.image)
    y_bound = map.image.size[1]
    cmap = get_cmap("tab10")  # Color map with 10 distinct colors
    keys = sorted(set(points.keys()) if points else set())
    key_to_color = {k: cmap(i % 10) for i, k in enumerate(keys)}
    for k in keys:
        color = key_to_color[k]
        # Plot hulls if available
        if hull and k in hull and hull[k] is not None:
            if isinstance(hull[k], MultiPolygon):
                for polygon in hull[k].geoms:
                    x, y = polygon.exterior.xy
                    plt.plot(x, y, color=color, label=k, lw=1)
            else:
                x, y = hull[k].exterior.xy
                plt.plot(x, y, color=color, label=k, lw=1)

        # Plot points
        if points and k in points and points[k] is not None:
            ax.scatter(*zip(*points[k]))
        # Plot start poses
        if k in start_poses and start_poses[k]:
            x = start_poses[k].configuration.x / map.resolution
            y = y_bound - start_poses[k].configuration.y / map.resolution
            plt.plot(
                x,
                y,
                marker="o",
                markersize=5,
                markeredgecolor="black",
                markerfacecolor="black",
            )
            plt.text(
                x,
                y - 5,  # adjust this value to move the text higher or lower
                start_poses[k].name,
                ha="center",
                va="bottom",
                fontsize=10,
                color="black",
            )
        # Plot goal poses
        if k in goal_poses and goal_poses[k]:
            x = goal_poses[k].configuration.x / map.resolution
            y = y_bound - goal_poses[k].configuration.y / map.resolution
            plt.plot(
                x,
                y,
                marker="o",
                markersize=5,
                markeredgecolor="black",
                markerfacecolor="black",
            )
            plt.text(
                x,
                y - 5,  # adjust this value to move the text higher or lower
                goal_poses[k].name,
                ha="center",
                va="bottom",
                fontsize=10,
                color="black",
            )
        # Plot unreachable
        if (
            unreachable_configurations
            and k in unreachable_configurations
            and unreachable_configurations[k]
        ):
            for u in unreachable_configurations[k]:
                x = u.configuration.x / map.resolution
                y = y_bound - u.configuration.y / map.resolution
                plt.plot(
                    x,
                    y,
                    marker="o",
                    markersize=5,
                    markeredgecolor="black",
                    markerfacecolor="black",
                )
                plt.plot(
                    x,
                    y,
                    marker="x",
                    markersize=10,
                    color=color,
                )
                plt.text(
                    x,
                    y - 5,  # adjust this value to move the text higher or lower
                    u.name,
                    ha="center",
                    va="bottom",
                    fontsize=10,
                    color="black",
                )
        # Plot obstacles
        if obstacles:
            for obj, cfg in obstacles.items():
                obstacle = cc.get_polygon_from_config(obj, cfg)
                ax.add_patch(
                    MplPolygon(list(obstacle.exterior.coords), color="black", alpha=0.3)
                )
    plt.legend()
    plt.show()


def plot_plan(
    problem_objects, plan, *, save: Optional[bool] = False, name: Optional[str] = None
):

    for o in problem_objects:
        if o.type.is_movable_type():
            if (
                o.motion_model == MotionModels.REEDSSHEPP
                or o.motion_model == MotionModels.SE2
            ):
                plot_2d_plan(problem_objects, plan, save=save, name=name)
            elif o.motion_model == MotionModels.SE3:
                plot_3d_plan(problem_objects, plan, save=save, name=name)
            else:
                raise NotImplementedError
            break


def plot_2d_plan(
    problem_objects, plan, *, save: Optional[bool] = False, name: Optional[str] = None
):
    ax = None
    footprints = {}
    moving_objs = []
    map_file = None
    resolution = 1.0

    # Estrarre informazioni dagli oggetti del problema
    for o in problem_objects:
        if o.type.is_movable_type():
            footprints[str(o)] = o.footprint
            moving_objs.append(str(o))
        if o.type.is_configuration_type():
            filename = o.type.occupancy_map.filename
            filepath = os.path.dirname(filename)  # Più robusto di replace()
            with open(filename, "r") as f:
                for line in f:
                    if "image: " in line:
                        map_file = os.path.join(
                            filepath, line.replace("image: ", "").strip()
                        )
                    elif "resolution: " in line:
                        resolution = float(line.replace("resolution: ", "").strip())

    # Caricare la mappa se disponibile
    if map_file:
        im = Image.open(map_file)
        _, ax = plt.figimage(im)
        # ax.imshow(im)

    # Assegnare colori distinti agli oggetti mobili
    cmap = plt.get_cmap("rainbow", len(moving_objs))

    # Estrarre le azioni dal piano
    actions = None
    if isinstance(plan.plan, TimeTriggeredPlan):
        actions = plan.plan.timed_actions
    elif isinstance(plan.plan, SequentialPlan):
        actions = plan.plan.actions

    # Evitare di procedere se `actions` è None
    if actions is None:
        raise ValueError(
            "Il piano fornito non è né TimeTriggeredPlan né SequentialPlan."
        )

    # Disegnare il percorso di ogni oggetto mobile
    for i, mo in enumerate(moving_objs):
        points_plotted = []

        for a in actions:
            if isinstance(plan.plan, TimeTriggeredPlan):
                a = a[1]  # Estrarre l'azione dalla tupla (timestamp, azione)

            if a.motion_paths:
                r = a.actual_parameters[0]

                if str(r) == mo:
                    for v in a.motion_paths.values():
                        for points in v:
                            x, y, w = points

                            height = (
                                footprints[str(r)][1][1] - footprints[str(r)][2][1]
                            ) / resolution
                            width = (
                                footprints[str(r)][1][0] - footprints[str(r)][0][0]
                            ) / resolution
                            point_of_rotation = np.array([width / 2, height / 2])

                            # Creazione del rettangolo rappresentante l'oggetto mobile
                            moving_obj = patches.Rectangle(
                                (x, y),
                                width,
                                height,
                                angle=math.degrees(w),
                                linewidth=1,
                                edgecolor=cmap(i),
                                facecolor="none",
                            )

                            # Calcolare lo spostamento per l'orientamento corretto
                            m_trans = np.array(
                                [[np.cos(w), -np.sin(w)], [np.sin(w), np.cos(w)]]
                            )
                            shift = -m_trans @ point_of_rotation
                            moving_obj.set_xy(moving_obj.get_xy() + shift)

                            ax.add_patch(moving_obj)

                            # Disegnare una freccia tra i punti successivi
                            if points_plotted:
                                plt.arrow(
                                    points_plotted[-1][0],
                                    points_plotted[-1][1],
                                    x - points_plotted[-1][0],
                                    y - points_plotted[-1][1],
                                    head_width=0.1,
                                    head_length=0.1,
                                    fc=cmap(i),
                                    ec=cmap(i),
                                )

                            points_plotted.append((x, y))

    # Salvare o mostrare il grafico
    if save and name:
        plt.savefig(name)
        plt.close()
    else:
        plt.show()


def plot_3d_plan(
    problem_objects, plan, *, save: Optional[bool] = False, name: Optional[str] = None
):

    meshes = {}
    moving_objs = []
    map_file = None

    mlab.figure()

    for o in problem_objects:
        if o.type.is_movable_type():
            meshes[str(o)] = trimesh.load(o.geometric_model, force="mesh")
            moving_objs.append(str(o))
        if o.type.is_configuration_type():
            filename = o.type.occupancy_map.filename
            filepath = filename.replace(filename.split(os.sep)[-1], "")
            f = open(filename)
            for l in f.readlines():
                if "mesh: " in l:
                    map_file = filepath + os.sep + l.replace("mesh: ", "").strip()

    if map_file is not None and os.path.exists(map_file):
        map_mesh = trimesh.load(map_file, force="mesh")
        map_vertices = map_mesh.vertices
        map_indices = map_mesh.faces
        mlab.triangular_mesh(
            map_vertices[:, 0],
            map_vertices[:, 1],
            map_vertices[:, 2],
            map_indices,
            opacity=1.0,
            color=(0.5, 0.5, 0.5),
        )  # representation='wireframe'
    else:
        raise FileNotFoundError(f"File {map_file} not found.")

    cmap = []
    for i in range(len(moving_objs)):
        cmap.append(tuple(np.random.uniform(range(0, 1), size=3)))

    i = 0

    for mo in moving_objs:
        points_plotted = []

        actions = None
        if isinstance(plan.plan, TimeTriggeredPlan):
            actions = plan.plan.timed_actions

        if isinstance(plan.plan, SequentialPlan):
            actions = plan.plan.actions

        for a in actions:

            if isinstance(plan.plan, TimeTriggeredPlan):
                a = a[1]

            if a.motion_paths is not None:
                r, _, _ = a.actual_parameters

                if str(r) == mo:
                    for _, v in a.motion_paths.items():
                        for points in v:

                            obj_at_current_state = meshes[mo].copy()
                            T = transformations.translation_matrix(
                                [points[0], points[1], points[2]]
                            )
                            # [w, x, y, z]
                            R = transformations.quaternion_matrix(
                                [points[3], points[4], points[5], points[6]]
                            )
                            tf = transformations.concatenate_matrices(T, R)
                            obj_at_current_state.apply_transform(tf)

                            obj_vertices = obj_at_current_state.vertices
                            obj_indices = obj_at_current_state.faces

                            mlab.triangular_mesh(
                                obj_vertices[:, 0],
                                obj_vertices[:, 1],
                                obj_vertices[:, 2],
                                obj_indices,
                                opacity=1.0,
                                color=cmap[i],
                            )

                            if points_plotted:
                                mlab.plot3d(
                                    [points_plotted[-1][0], points[0]],
                                    [points_plotted[-1][1], points[1]],
                                    [points_plotted[-1][2], points[2]],
                                    color=cmap[i],
                                    tube_radius=0.1,
                                )

                            points_plotted.append((points[0], points[1], points[2]))

        i += 1

    if save:
        mlab.savefig(name)
    else:
        mlab.show()
