import os
from typing import Dict, List, Set, Tuple
import numpy as np
from shapely.geometry import Polygon
from ompl import base as ob
from unified_planning.shortcuts import *
from shapely.affinity import rotate
from matplotlib import pyplot as plt
from descartes import PolygonPatch
from tampest.map import Map, Map2D, Map3D
import cv2
from matplotlib import pyplot as plt
import numpy as np
from ompl import base as ob
from trimesh import  transformations
import trimesh
from mayavi import mlab
from tampest.motion_planning_data import SupportedTopologicalRefinement


class CollisionChecker:

    def __init__(self, moving_object: MovableObject, map: Map, movable_obstacles: Dict[MovableObject, ConfigurationObject],
                 topological_refinement: SupportedTopologicalRefinement) -> None:
        self.moving_object = moving_object
        self.map = map
        self.movable_obstacles = movable_obstacles
        self._collision_objects = set()
        self.topological_refinement = topological_refinement

    def get_collision_objects(self) -> Set[MovableObject]:
        return self._collision_objects

    def add_collision_object(self, co: MovableObject):
        self._collision_objects.add(co)

class CollisionChecker3D(CollisionChecker):

    def __init__(self, moving_object: MovableObject, map: Map3D, movable_obstacles: Dict[MovableObject, ConfigurationObject],
                 topological_refinement: SupportedTopologicalRefinement) -> None:
        super().__init__(moving_object, map, movable_obstacles, topological_refinement)
        self.movable_obstacles = {obj.name: obj for obj, _ in movable_obstacles.items()}

        self.env_mesh = ("env", map.mesh)
        self.moving_object_mesh = ("moving_object", self.get_obj_mesh(moving_object))
        self.movable_obstacles_meshes = [(obj.name, self.get_obj_mesh(obj, config.configuration)) for obj, config in movable_obstacles.items()]

        self.collision_manager = self.get_collision_manager(self.movable_obstacles_meshes + [self.env_mesh, self.moving_object_mesh])

    def get_obj_mesh(self, obj: MovableObject, transform: Optional[Tuple[float]] = None) -> trimesh.Trimesh:
        if os.path.exists(obj.model):
            if transform is not None:
                if not obj.motion_model == MotionModels.SE3:
                    raise NotImplementedError(f"Motion model {obj.motion_model} not yet supported.")
                # Assumption for SE3: (x, y, z, rw, rx, ry, rz)
                T = transformations.translation_matrix([transform[0], transform[1], transform[2]])
                rotation = (transform[3], transform[4], transform[5], transform[6])
                if all(v == 0 for v in rotation):
                    R = transformations.quaternion_matrix([1, 0, 0, 0])
                else:
                    R = transformations.rotation_matrix(transform[6], [transform[3], transform[4], transform[5]])
                tf = transformations.concatenate_matrices(T, R)
                return trimesh.load(obj.model, force='mesh').apply_transform(tf)
            else:
                return trimesh.load(obj.model, force='mesh')
        else:
            raise FileNotFoundError(f"File {obj.model} not found.")

    def get_collision_manager(self, objs: Dict[str, trimesh.Trimesh]) -> trimesh.collision.CollisionManager:
        m = trimesh.collision.CollisionManager()
        for (k, v) in objs:
            m.add_object(k, v)
        return m

    def isStateValid(self, state: ob.State) -> bool:
        T = transformations.translation_matrix([state.getX(), state.getY(), state.getZ()])
        R = transformations.quaternion_matrix([state.rotation().w, state.rotation().x, state.rotation().y, state.rotation().z])
        tf = transformations.concatenate_matrices(T, R)

        collision, collision_objs = self.collision_manager.in_collision_single(self.moving_object_mesh[1], tf, return_names=True)

        if self.topological_refinement in [SupportedTopologicalRefinement.ALL, SupportedTopologicalRefinement.OBS] and collision_objs:
                keys = set(self.movable_obstacles.keys())
                for elem in keys.intersection(collision_objs):
                        self.add_collision_object(self.movable_obstacles[elem])

        return not collision

    def plot_current_state(self, start: ConfigurationObject, goal: ConfigurationObject):

        # plot map
        env_vertices = self.env_mesh[1].vertices
        env_indices = self.env_mesh[1].faces

        mlab.figure()
        mlab.triangular_mesh(env_vertices[:,0], env_vertices[:,1], env_vertices[:,2],  env_indices, opacity=1.0, color=(0.5, 0.5, 0.5))

        # plot moving object at its start and goal configuration
        obj_color = tuple(np.random.uniform(range(0, 1), size=3))

        # obj at start - Assumption for SE3: (x, y, z, rx, ry, rz, rangle)
        obj_at_start = self.moving_object_mesh[1].copy()
        T = transformations.translation_matrix([start.configuration[0], start.configuration[1], start.configuration[2]])
        rotation = (start.configuration[3], start.configuration[4], start.configuration[5], start.configuration[6])
        if all(v == 0 for v in rotation):
            R = transformations.quaternion_matrix([1, 0, 0, 0])
        else:
            R = transformations.rotation_matrix(start.configuration[6], [start.configuration[3], start.configuration[4], start.configuration[5]])
        tf = transformations.concatenate_matrices(T, R)
        obj_at_start.apply_transform(tf)

        mlab.triangular_mesh(obj_at_start.vertices[:,0], obj_at_start.vertices[:,1], obj_at_start.vertices[:,2], obj_at_start.faces, opacity=1.0, color=obj_color)

        # obj at goal - Assumption for SE3: (x, y, z, rx, ry, rz, rangle)
        obj_at_goal = self.moving_object_mesh[1].copy()
        T = transformations.translation_matrix([goal.configuration[0], goal.configuration[1], goal.configuration[2]])
        rotation = (goal.configuration[3], goal.configuration[4], goal.configuration[5], goal.configuration[6])
        if all(v == 0 for v in rotation):
            R = transformations.quaternion_matrix([1, 0, 0, 0])
        else:
            R = transformations.rotation_matrix(goal.configuration[6], [goal.configuration[3], goal.configuration[4], goal.configuration[5]])
        tf = transformations.concatenate_matrices(T, R)
        obj_at_goal.apply_transform(tf)

        mlab.triangular_mesh(obj_at_goal.vertices[:,0], obj_at_goal.vertices[:,1], obj_at_goal.vertices[:,2], obj_at_goal.faces, opacity=1.0, color=obj_color)

        # plot movable obstacles at their current configuration
        obs_color = tuple(np.random.uniform(range(0, 1), size=3))

        for (_, obs_mesh) in self.movable_obstacles_meshes:
            mlab.triangular_mesh(obs_mesh.vertices[:,0], obs_mesh.vertices[:,1], obs_mesh.vertices[:,2], obs_mesh.faces, opacity=1.0, color=obs_color)

        mlab.show()

class CollisionChecker2D(CollisionChecker):

    def __init__(self, moving_object: MovableObject, map: Map2D,  movable_obstacles: Dict[MovableObject, ConfigurationObject],
                 topological_refinement: SupportedTopologicalRefinement) -> None:
        super().__init__(moving_object, map, movable_obstacles, topological_refinement)
        self.moving_object_vertices = [(moving_object.footprint[0][0]/map.resolution, moving_object.footprint[0][1]/map.resolution),
                              (moving_object.footprint[1][0]/map.resolution, moving_object.footprint[1][1]/map.resolution),
                              (moving_object.footprint[2][0]/map.resolution, moving_object.footprint[2][1]/map.resolution),
                              (moving_object.footprint[3][0]/map.resolution, moving_object.footprint[3][1]/map.resolution)]
        self.fixed_obstacles = self.get_fixed_obstacles(map)
        self.movable_obstacles = {k:self.get_polygon_from_config(k, v) for k, v in movable_obstacles.items()}

    def get_fixed_obstacles(self, map: Map2D) -> List[Polygon]:

        # Threshold the image of the map
        open_cv_image = np.array(map.image)

        gray = cv2.cvtColor(open_cv_image, cv2.COLOR_BGR2GRAY)

        # binarize image
        _, bw = cv2.threshold(gray, 0, 1, cv2.THRESH_BINARY_INV)

        # find contour
        contours,_ = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Get fix obstacles
        fix_obstacles = [Polygon(c.reshape(-1, 2)) for c in contours]

        return fix_obstacles

    def get_polygon_from_state(self, state: ob.State) -> Polygon:
        x = state.getX()
        y = state.getY()
        yaw = state.getYaw()

        points = [(x+sx, y+sy) for (sx, sy) in self.moving_object_vertices]
        polygon = rotate(Polygon(points), yaw, use_radians=True)

        return polygon

    def get_polygon_from_config(self, obj: MovableObject, config: ConfigurationObject) -> Polygon:
        x = config.configuration[0]/self.map.resolution
        y = config.configuration[1]/self.map.resolution
        yaw = config.configuration[2]

        points = [(x + obj.footprint[0][0]/self.map.resolution, self.map.image.size[1] - y + obj.footprint[0][1]/self.map.resolution),
                  (x + obj.footprint[1][0]/self.map.resolution, self.map.image.size[1] - y  + obj.footprint[1][1]/self.map.resolution),
                  (x + obj.footprint[2][0]/self.map.resolution, self.map.image.size[1] - y  + obj.footprint[2][1]/self.map.resolution),
                  (x + obj.footprint[3][0]/self.map.resolution, self.map.image.size[1] - y  + obj.footprint[3][1]/self.map.resolution)]

        polygon = rotate(Polygon(points), yaw, use_radians=True)

        return polygon

    def isStateValid(self, state: ob.State) -> bool:

        # Get moving object current state
        moving = self.get_polygon_from_state(state)

        # Find intersections with fix obstacles
        for o in self.fixed_obstacles:
            if moving.intersects(o):
                return False

        collision_found = False

        # Find intersections with movable objects
        for k, v in self.movable_obstacles.items():
            if moving.intersects(v):
                if self.topological_refinement in [SupportedTopologicalRefinement.ALL, SupportedTopologicalRefinement.OBS]:
                    collision_found = True
                    self.add_collision_object(k)
                else:
                    return False

        return not collision_found

    def plot_current_state(self, start: ConfigurationObject, goal: ConfigurationObject):

        _, ax = plt.subplots()

        ax.imshow(self.map.image)

        for obs in self.fixed_obstacles:
            ax.add_patch(PolygonPatch(obs, alpha=0))

        start = self.get_polygon_from_config(self.moving_object, start)
        ax.add_patch(PolygonPatch(start, alpha=0.1))

        goal = self.get_polygon_from_config(self.moving_object, goal)
        ax.add_patch(PolygonPatch(goal, alpha=0.1))

        for _, v in self.movable_obstacles.items():
            ax.add_patch(PolygonPatch(v, alpha=1.0))

        plt.show()
