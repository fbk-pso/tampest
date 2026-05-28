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

from typing import Optional, List, Dict
from PIL import Image
import os
import math
from matplotlib import patches
from matplotlib import pyplot as plt
from matplotlib.cm import get_cmap
from matplotlib.patches import Polygon as MplPolygon
import numpy as np
from trimesh import transformations
import trimesh
from tampest.motion.collision_checker import CollisionChecker2D, CollisionChecker3D
from tampest.motion.map import Map, Map2D, Map3D
from unified_planning.shortcuts import (
    ConfigurationObject,
    MovableObject,
    MotionModels,
    Object,
)
from scipy.spatial import ConvexHull
from shapely.geometry import MultiPolygon

from unified_planning.plans.time_triggered_plan import TimeTriggeredPlan
from unified_planning.plans.sequential_plan import SequentialPlan
from unified_planning.engines.results import PlanGenerationResult

import pyvista as pv


def plot_reachability_data(
    map: Map,
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

    plotter = pv.Plotter()

    # plot map
    env_vertices = map.mesh.vertices
    env_faces = map.mesh.faces
    faces = np.hstack((np.full((env_faces.shape[0], 1), 3), env_faces))
    mesh = pv.PolyData(env_vertices, faces)
    plotter.add_mesh(mesh, color="lightgrey", opacity=1.0)

    if hull and len(hull) == 1:

        # PyVista expects a flattened connectivity array with face sizes prepended
        faces = np.hstack([np.insert(face, 0, 3) for face in hull[0].faces])

        hull_mesh = pv.PolyData(hull[0].vertices, faces)
        plotter.add_mesh(hull_mesh, color="lightgrey", opacity=1.0)

    if points and len(points) == 1:

        cloud = pv.PolyData(points[0])
        plotter.add_mesh(cloud, point_size=10, render_points_as_spheres=True)

    for _, s in start_poses.items():

        start_point = np.array(
            [[s.configuration.x, s.configuration.y, s.configuration.z]]
        )

        start_cloud = pv.PolyData(start_point)

        plotter.add_mesh(
            start_cloud,
            color=(0, 1, 0),  # green
        )

    for _, g in goal_poses.items():

        goal_point = np.array(
            [[g.configuration.x, g.configuration.y, g.configuration.z]]
        )

        goal_cloud = pv.PolyData(goal_point)

        plotter.add_mesh(
            goal_cloud,
            color=(0, 1, 0),  # green
        )

    if obstacles:
        for k, v in obstacles.items():
            obstacle_mesh = cc.get_obj_mesh(k, v.configuration)

            # PyVista expects a flattened connectivity array with face sizes prepended
            faces = np.hstack([np.insert(face, 0, 3) for face in obstacle_mesh.faces])

            obs_mesh = pv.PolyData(obstacle_mesh.vertices, faces)
            plotter.add_mesh(obs_mesh, opacity=1.0, color=(1, 0, 0))  # red

    if unreachable_configurations:
        for _, unreach in unreachable_configurations.items():

            for u in unreach:

                unreach_point = np.array(
                    [[u.configuration.x, u.configuration.y, u.configuration.z]]
                )

                unreach_cloud = pv.PolyData(unreach_point)

                plotter.add_mesh(
                    unreach_cloud,
                    opacity=1.0,
                    color=(1, 0, 0),  # red
                )

    plotter.show()


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
    used_labels = set()
    for k in keys:
        color = key_to_color[k]
        label = str(k) if k not in used_labels else None
        # Plot hulls if available
        if hull and k in hull and hull[k] is not None:
            if isinstance(hull[k], MultiPolygon):
                for i, polygon in enumerate(hull[k].geoms):
                    x, y = polygon.exterior.xy
                    lbl = label if i == 0 else None  # Only label first polygon
                    plt.plot(x, y, color=color, label=lbl, lw=1)
            else:
                x, y = hull[k].exterior.xy
                plt.plot(x, y, color=color, label=label, lw=1)

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
    problem_objects: List[Object],
    plan: PlanGenerationResult,
    *,
    save: Optional[bool] = False,
    name: Optional[str] = None,
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
    problem_objects: List[Object],
    plan: PlanGenerationResult,
    *,
    save: Optional[bool] = False,
    name: Optional[str] = None,
):
    _, ax = plt.subplots()
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
        ax.imshow(im)
        ax.set_aspect("equal")

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

                        if isinstance(plan.plan, SequentialPlan):
                            v = v[0]

                        for points in v:

                            if isinstance(plan.plan, SequentialPlan):
                                x, y, w = points
                            elif isinstance(plan.plan, TimeTriggeredPlan):
                                x, y, w, _, _, _ = points
                            else:
                                raise NotImplementedError

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
    problem_objects: List[Object],
    plan: PlanGenerationResult,
    *,
    save: Optional[bool] = False,
    name: Optional[str] = None,
):

    meshes = {}
    moving_objs = []
    map_file = None

    plotter = pv.Plotter()

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

        env_vertices = map_mesh.vertices
        env_faces = map_mesh.faces
        faces = np.hstack((np.full((env_faces.shape[0], 1), 3), env_faces))
        mesh = pv.PolyData(env_vertices, faces)
        plotter.add_mesh(mesh, color="lightgrey", opacity=1.0)

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
                        for _, points in v.items():

                            for p in points:

                                obj_at_current_state = meshes[mo].copy()
                                T = transformations.translation_matrix(
                                    [p[0], p[1], p[2]]
                                )
                                # [w, x, y, z]
                                R = transformations.quaternion_matrix(
                                    [p[3], p[4], p[5], p[6]]
                                )
                                tf = transformations.concatenate_matrices(T, R)
                                obj_at_current_state.apply_transform(tf)

                                obj_vertices = obj_at_current_state.vertices
                                obj_faces = obj_at_current_state.faces

                                faces = np.hstack(
                                    (np.full((obj_faces.shape[0], 1), 3), obj_faces)
                                )
                                mesh = pv.PolyData(obj_vertices, faces)
                                plotter.add_mesh(mesh, color=cmap[i], opacity=1.0)

        i += 1

    if save:
        plotter.save_graphic(name)
    else:
        plotter.show()
