from typing import Optional, List, Dict, Union
from descartes import PolygonPatch
from PIL import Image
import os
import math
from matplotlib import patches
from matplotlib import pyplot as plt
from mayavi import mlab
import numpy as np
from trimesh import  transformations
import trimesh
from tampest.collision_checker import CollisionChecker2D, CollisionChecker3D
from tampest.map import Map2D, Map3D
from unified_planning.shortcuts import ConfigurationObject, MovableObject, MotionModels
from scipy.spatial import ConvexHull

def plot_reachability_data(map: Union[Map2D, Map3D], motion_model: MotionModels, *, hull: Optional[ConvexHull] = None, points:Optional[np.ndarray] = None, start_pose: Optional[List[ConfigurationObject]], goal_pose: Optional[List[ConfigurationObject]],  obstacles: Optional[Dict[MovableObject, ConfigurationObject]] = None, cc: Optional[CollisionChecker2D] = None, unreachable_configurations: Optional[List[ConfigurationObject]] = None):

    if motion_model == MotionModels.REEDSSHEPP or motion_model == MotionModels.SE2:
        plot_2d_reachability_data(map, hull=hull, points=points, start_pose=start_pose, goal_pose=goal_pose, obstacles=obstacles, cc=cc, unreachable_configurations=unreachable_configurations)
    elif motion_model == MotionModels.SE3:
        plot_3d_reachability_data(map, hull=hull, points=points, start_pose=start_pose, goal_pose=goal_pose, obstacles=obstacles, cc=cc, unreachable_configurations=unreachable_configurations)
    else:
        raise NotImplementedError

def plot_2d_reachability_data(map: Map2D, *, hull: Optional[ConvexHull] = None, points:Optional[np.ndarray] = None, start_pose: Optional[List[ConfigurationObject]], goal_pose: Optional[List[ConfigurationObject]],  obstacles: Optional[Dict[MovableObject, ConfigurationObject]] = None, cc: Optional[CollisionChecker2D] = None, unreachable_configurations: Optional[List[ConfigurationObject]] = None):

    _, ax = plt.subplots()

    ax.imshow(map.image)

    if hull:
        plt.plot(points[hull.vertices,0], points[hull.vertices,1], 'b-', lw=1)

    if points.any():
        ax.scatter(*zip(*points))

    y_bound = map.image.size[1]

    if start_pose:
        plt.plot([start_pose.configuration[0]/map.resolution], [y_bound - start_pose.configuration[1]/map.resolution], marker="o", markersize=5, markeredgecolor="green", markerfacecolor="green")

    if goal_pose:
        plt.plot([goal_pose.configuration[0]/map.resolution], [y_bound - goal_pose.configuration[1]/map.resolution], marker="o", markersize=5, markeredgecolor="green", markerfacecolor="green")

    if obstacles:
        for k, v in obstacles.items():
            obstacle = cc.get_polygon_from_config(k, v)
            ax.add_patch(PolygonPatch(obstacle, alpha=1.0))

    if unreachable_configurations:
        for u in unreachable_configurations:
            plt.plot([u.configuration[0]/map.resolution], [y_bound - u.configuration[1]/map.resolution], marker="o", markersize=5, markeredgecolor="red", markerfacecolor="red")

    plt.show()

def plot_3d_reachability_data(map: Map3D, *, hull: Optional[ConvexHull] = None, points:Optional[np.ndarray] = None, start_pose: Optional[List[ConfigurationObject]], goal_pose: Optional[List[ConfigurationObject]],  obstacles: Optional[Dict[MovableObject, ConfigurationObject]] = None, cc: Optional[CollisionChecker3D] = None, unreachable_configurations: Optional[List[ConfigurationObject]] = None):

    mlab.figure()

    map_vertices = map.mesh.vertices
    map_indices = map.mesh.faces
    mlab.triangular_mesh(map_vertices[:,0], map_vertices[:,1], map_vertices[:,2],  map_indices, opacity=1.0, color=(0.5, 0.5, 0.5)) # representation='wireframe'

    #if hull:
    #    mlab.triangular_mesh(hull.points[:,0], hull.points[:,1], hull.points[:,2], hull.simplices)

    if points.any():
        mlab.points3d(points[:,0], points[:,1], points[:,2], scale_factor=1.0)

    if start_pose:
        mlab.points3d(start_pose.configuration[0], start_pose.configuration[1], start_pose.configuration[2], scale_factor=1.0, color=(0, 1, 0)) # green = (0, 1, 0)(RGB normalized)

    if goal_pose:
        mlab.points3d(goal_pose.configuration[0], goal_pose.configuration[1], goal_pose.configuration[2], scale_factor=1.0, color=(0, 1, 0)) # green = (0, 1, 0)(RGB normalized)

    if obstacles:
        for k, v in obstacles.items():
            obstacle_mesh = cc.get_obj_mesh(k, v.configuration)
            obstacle_vertices = obstacle_mesh.vertices
            obstacle_indices = obstacle_mesh.faces
            mlab.triangular_mesh(obstacle_vertices[:,0], obstacle_vertices[:,1], obstacle_vertices[:,2], obstacle_indices, opacity=1.0, color=(1, 0, 0)) # red = (1, 0, 0) (RGB normalized)

    if unreachable_configurations:
        for u in unreachable_configurations:
            mlab.points3d(u.configuration[0], u.configuration[1], u.configuration[2], scale_factor=1.0, color=(1, 0, 0)) # red = (1, 0, 0)(RGB normalized)

    plt.show()

def plot_plan(problem_objects, plan, *, save:Optional[bool]=False, name:Optional[str]=None):

    for o in problem_objects:
        if o.type.is_movable_type():
            if o.motion_model == MotionModels.REEDSSHEPP or o.motion_model == MotionModels.SE2:
               plot_2d_plan(problem_objects, plan, save=save, name=name) 
            elif o.motion_model == MotionModels.SE3:
                plot_3d_plan(problem_objects, plan, save=save, name=name)
            else:
                raise NotImplementedError
            break

def plot_2d_plan(problem_objects, plan, *, save:Optional[bool]=False, name:Optional[str]=None):

    ax = None
    footprints = {}
    moving_objs = []
    map_file = None
    resolution = 1.0


    for o in problem_objects:
        if o.type.is_movable_type():
            footprints[str(o)] = o.footprint
            moving_objs.append(str(o))
        if o.type.is_configuration_type():
            filename = o.type.occupancy_map.filename
            filepath = filename.replace(filename.split(os.sep)[-1], "")
            f = open(filename)
            for l in f.readlines():
                if "image: " in l:
                    map_file = filepath + os.sep + l.replace("image: ", "").strip()
                if "resolution: " in l:
                    resolution = float(l.replace("resolution: ", "").strip())

    if map_file is not None:
        im = Image.open(map_file)
        _, ax = plt.subplots()
        ax.imshow(im)

    cmap = plt.get_cmap('rainbow', len(moving_objs))
    i = 0

    for mo in moving_objs:
        points_plotted = []

        for a in plan.plan.actions:

            if a.motion_paths is not None:
                r = a.actual_parameters[0]

                if str(r) == mo:
                    for _, v in a.motion_paths.items():
                        for points in v:
                            x = points[0]
                            y = points[1]
                            w = points[2]

                            height = (footprints[str(r)][1][1] - footprints[str(r)][2][1])/resolution
                            width = (footprints[str(r)][1][0] - footprints[str(r)][0][0])/resolution
                            point_of_rotation = np.array([width/2, height/2])

                            moving_obj = patches.Rectangle((x, y), width, height, math.degrees(w), linewidth=1, edgecolor=cmap(i), facecolor='none')
                            m_trans = np.array([[np.cos(w), -np.sin(w)],
                                        [np.sin(w), np.cos(w)]])
                            shift = -m_trans @ point_of_rotation
                            moving_obj.set_xy(moving_obj.get_xy() + shift)

                            ax.add_patch(moving_obj)

                            if points_plotted:
                                plt.arrow(points_plotted[-1][0], points_plotted[-1][1], x - points_plotted[-1][0], y - points_plotted[-1][1])

                            points_plotted.append((x, y))
        i+=1

    if save:
        plt.savefig(name)
    else:
        plt.show()

def plot_3d_plan(problem_objects, plan, *, save:Optional[bool]=False, name:Optional[str]=None):

    meshes = {}
    moving_objs = []
    map_file = None

    mlab.figure()

    for o in problem_objects:
        if o.type.is_movable_type():
            meshes[str(o)] = trimesh.load(o.model, force='mesh')
            moving_objs.append(str(o))
        if o.type.is_configuration_type():
            filename = o.type.occupancy_map.filename
            filepath = filename.replace(filename.split(os.sep)[-1], "")
            f = open(filename)
            for l in f.readlines():
                if "mesh: " in l:
                    map_file = filepath + os.sep + l.replace("mesh: ", "").strip()

    if map_file is not None and os.path.exists(map_file):
        map_mesh = trimesh.load(map_file, force='mesh')
        map_vertices = map_mesh.vertices
        map_indices = map_mesh.faces
        mlab.triangular_mesh(map_vertices[:,0], map_vertices[:,1], map_vertices[:,2],  map_indices, opacity=1.0, color=(0.5, 0.5, 0.5)) # representation='wireframe'
    else:
        raise FileNotFoundError(f"File {map_file} not found.")

    cmap = []
    for i in range(len(moving_objs)):
        cmap.append(tuple(np.random.uniform(range(0, 1), size=3)))

    i = 0

    for mo in moving_objs:
        points_plotted = []

        for a in plan.plan.actions:

            if a.motion_paths is not None:
                r, _, _ = a.actual_parameters

                if str(r) == mo:
                    for _, v in a.motion_paths.items():
                        for points in v:

                            obj_at_current_state = meshes[mo].copy()
                            T = transformations.translation_matrix([points[0], points[1], points[2]])
                            #[w, x, y, z]
                            R = transformations.quaternion_matrix([points[3], points[4], points[5], points[6]])
                            tf = transformations.concatenate_matrices(T, R)
                            obj_at_current_state.apply_transform(tf)

                            obj_vertices = obj_at_current_state.vertices
                            obj_indices = obj_at_current_state.faces

                            mlab.triangular_mesh(obj_vertices[:,0], obj_vertices[:,1], obj_vertices[:,2], obj_indices, opacity=1.0, color=cmap[i])

                            if points_plotted:
                                mlab.plot3d([points_plotted[-1][0], points[0]], [points_plotted[-1][1], points[1]], [points_plotted[-1][2], points[2]], color=cmap[i], tube_radius=0.1)

                            points_plotted.append((points[0], points[1], points[2]))

        i+=1

    if save:
        mlab.savefig(name)
    else:
        mlab.show()
