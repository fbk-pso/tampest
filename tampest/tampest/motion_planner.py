from ompl import base as ob
from ompl import geometric as og
from typing import Optional, Tuple
import numpy as np
import time
from shapely.affinity import *
from tampest.map import Map
from tampest.collision_checker import CollisionChecker, CollisionChecker2D, CollisionChecker3D
from unified_planning.shortcuts import *
from tampest.motion_planning_data import MotionPlanningData, SupportedPlanner, SupportedTopologicalRefinement
from tampest.utils import *
from scipy.spatial import ConvexHull

class MotionPlanner:

    def __init__(self) -> None:
        pass

    def set_problem(self, map: Map, moving_obj: (MovableObject, ConfigurationObject, ConfigurationObject), planner: SupportedPlanner, cc: CollisionChecker, distance: Optional[float]=None) -> og.SimpleSetup:

        # set state space space
        if moving_obj[0].motion_model == MotionModels.REEDSSHEPP:
            space = ob.ReedsSheppStateSpace(moving_obj[0].parameters["turning_radius"])
        elif moving_obj[0].motion_model == MotionModels.SE2:
            space = ob.SE2StateSpace()
        elif moving_obj[0].motion_model == MotionModels.SE3:
            space = ob.SE3StateSpace()
        else:
            raise NotImplementedError

        # set map bounds
        if moving_obj[0].motion_model == MotionModels.REEDSSHEPP or moving_obj[0].motion_model == MotionModels.SE2:
            bounds = ob.RealVectorBounds(2)
            bounds.setLow(0.0)
            bounds.high[0] = map.image.size[0]
            bounds.high[1] = map.image.size[1]

        if moving_obj[0].motion_model == MotionModels.SE3:
            env_bounds = map.mesh.bounds.T
            bounds = ob.RealVectorBounds(3)
            bounds.low[0] = env_bounds[0][0]
            bounds.low[1] = env_bounds[1][0]
            bounds.low[2] = env_bounds[2][0]
            bounds.high[0] = env_bounds[0][1]
            bounds.high[1] = env_bounds[1][1]
            bounds.high[2] = env_bounds[2][1]

        space.setBounds(bounds)

        # motion setup
        motion_setup = og.SimpleSetup(space)

        # set the planner
        if planner == SupportedPlanner.RRT:
            selected_planner = og.RRT(motion_setup.getSpaceInformation())
        elif planner == SupportedPlanner.LazyRRT:
            selected_planner = og.LazyRRT(motion_setup.getSpaceInformation())
        elif planner == SupportedPlanner.RRTstar:
            selected_planner = og.RRTstar(motion_setup.getSpaceInformation())
        elif planner == SupportedPlanner.KPIECE1:
            selected_planner = og.KPIECE1(motion_setup.getSpaceInformation())
        elif planner == SupportedPlanner.PRM:
            selected_planner = og.PRM(motion_setup.getSpaceInformation())
        elif planner == SupportedPlanner.LazyPRM:
            selected_planner = og.LazyPRM(motion_setup.getSpaceInformation())
        elif planner == SupportedPlanner.EST:
            selected_planner = og.EST(motion_setup.getSpaceInformation())
        elif planner == SupportedPlanner.SBL:
            selected_planner = og.SBL(motion_setup.getSpaceInformation())
        else:
            raise NotImplementedError

        # set the range of the planner
        if distance is not None:
            selected_planner.setRange(distance)

        motion_setup.setPlanner(selected_planner)

        # set state validity checking for this space
        motion_setup.setStateValidityChecker(ob.StateValidityCheckerFn(cc.isStateValid))

        if moving_obj[0].motion_model == MotionModels.REEDSSHEPP or moving_obj[0].motion_model == MotionModels.SE2:
            motion_setup.getSpaceInformation().setStateValidityCheckingResolution(0.01)

        if moving_obj[0].motion_model == MotionModels.SE3:
            motion_setup.getSpaceInformation().setStateValidityCheckingResolution(0.05)

        # start and goal configurations
        ss = ob.State(motion_setup.getStateSpace())
        gs = ob.State(motion_setup.getStateSpace())

        if moving_obj[0].motion_model == MotionModels.REEDSSHEPP or moving_obj[0].motion_model == MotionModels.SE2:
            ss().setXY(moving_obj[1].configuration[0]/map.resolution, bounds.high[1] - moving_obj[1].configuration[1]/map.resolution)
            ss().setYaw(moving_obj[1].configuration[2])

            gs().setXY(moving_obj[2].configuration[0]/map.resolution, bounds.high[1] - moving_obj[2].configuration[1]/map.resolution)
            gs().setYaw(moving_obj[2].configuration[2])

        # Assumption for SE3: (x, y, z, rw, rx, ry, rz)
        if moving_obj[0].motion_model == MotionModels.SE3:
            ss().setXYZ(moving_obj[1].configuration[0], moving_obj[1].configuration[1], moving_obj[1].configuration[2])
            ss().rotation().setAxisAngle(moving_obj[1].configuration[3], moving_obj[1].configuration[4], moving_obj[1].configuration[5], moving_obj[1].configuration[6])
            gs().setXYZ(moving_obj[2].configuration[0], moving_obj[2].configuration[1], moving_obj[2].configuration[2])
            gs().rotation().setAxisAngle(moving_obj[2].configuration[3], moving_obj[2].configuration[4], moving_obj[2].configuration[5], moving_obj[2].configuration[6])

        motion_setup.setStartAndGoalStates(ss, gs)

        return motion_setup

    def get_solution(self, motion_setup, *, planning_time: Optional[float] = 1.0, interpolate: Optional[bool] = True, simplified: Optional[bool] = True, topological_refinement: Optional[SupportedTopologicalRefinement] = SupportedTopologicalRefinement.ALL):
        planning_data = MotionPlanningData()
        solution_path = None
        planner_data = None

        start_time = time.time()
        result = motion_setup.solve(planning_time)
        planning_data.planning_time = time.time() - start_time

        if result.getStatus() == ob.PlannerStatus.EXACT_SOLUTION:
            if simplified:
                motion_setup.simplifySolution()
            solution_path = motion_setup.getSolutionPath()
            planning_data.n_waypoints = solution_path.getStateCount()

            if interpolate:
                start_time = time.time()
                solution_path.interpolate()
                planning_data.interpolation_time = time.time() - start_time
                planning_data.n_waypoints_after_interpolation = solution_path.getStateCount()
                #print(path.printAsMatrix())

            planning_data.path_length = solution_path.length()

        if not motion_setup.haveExactSolutionPath():

            print(
                    "Exact solution not found. Distance to actual goal equals to %g"
                    % motion_setup.getProblemDefinition().getSolutionDifference()
                )

            planner_data = ob.PlannerData(motion_setup.getSpaceInformation())
            motion_setup.getPlannerData(planner_data)

        return solution_path, planner_data, planning_data

    def check_motion_constraint(self, moving_object, start, goal, obstacles, problem_objects, *, planning_time: Optional[float] = 1.0, interpolate: Optional[bool] = True, simplified: Optional[bool] = True, tolerance: Optional[float] = 0.0, distance: Optional[float] = None, motion_planner: Optional[SupportedPlanner] = SupportedPlanner.RRT, topological_refinement: Optional[SupportedTopologicalRefinement] = SupportedTopologicalRefinement.ALL) -> Tuple[bool, Optional[List[Tuple[float, ...]]], Optional[List[ConfigurationObject]], Optional[List[MovableObject]], MotionPlanningData]:

        if moving_object.motion_model != MotionModels.REEDSSHEPP and moving_object.motion_model != MotionModels.SE2 and moving_object.motion_model != MotionModels.SE3:
            raise NotImplementedError

        is_valid = False
        path = None
        unreachable_configurations = []

        map = None

        for o in problem_objects:
            if o.type.is_configuration_type():
                if map is None:
                    map = Map().get_from_file(o.type.occupancy_map.filename)
                    break

        if moving_object.motion_model == MotionModels.REEDSSHEPP or moving_object.motion_model == MotionModels.SE2:
            cc = CollisionChecker2D(moving_object=moving_object, map=map, movable_obstacles=obstacles, topological_refinement=topological_refinement)

        if moving_object.motion_model == MotionModels.SE3:
            cc = CollisionChecker3D(moving_object=moving_object, map=map, movable_obstacles=obstacles, topological_refinement=topological_refinement)

        motion_problem = self.set_problem(map, (moving_object, start, goal), motion_planner, cc, distance)
        #cc.plot_current_state(start, goal)

        solution_path, planner_data, planning_data = self.get_solution(motion_problem, planning_time=planning_time, interpolate=interpolate, simplified=simplified, topological_refinement=topological_refinement)

        unreachable_configurations = []
        collision_obstacles = []

        if solution_path:
            is_valid = True
            path = []
            for state in solution_path.getStates():
                if moving_object.motion_model == MotionModels.REEDSSHEPP or moving_object.motion_model == MotionModels.SE2:
                    path.append((state.getX(), state.getY(), state.getYaw()))
                if moving_object.motion_model == MotionModels.SE3:
                    path.append((state.getX(), state.getY(), state.getZ(), state.rotation().w, state.rotation().x, state.rotation().y, state.rotation().z))

        else:

            appended = [str(goal)]
            unreachable_configurations.append(goal)
            collision_obstacles = obstacles.keys()

            hull = None

            if planner_data:
                sampled_vertices =[]
                for i in range(planner_data.numVertices()):
                    state = planner_data.getVertex(i).getState()
                    if moving_object.motion_model == MotionModels.REEDSSHEPP or moving_object.motion_model == MotionModels.SE2:
                        sampled_vertices.append((state.getX(), state.getY()))
                    if moving_object.motion_model == MotionModels.SE3:
                        sampled_vertices.append((state.getX(), state.getY(), state.getZ()))

                
                if sampled_vertices and topological_refinement in [SupportedTopologicalRefinement.ALL, SupportedTopologicalRefinement.UNREACH]:
                    
                    start_time = time.time()
                    hull = ConvexHull(np.array(sampled_vertices))
                    planning_data.convex_hull_time = time.time() - start_time

                    if hull is not None:
                        for obj in problem_objects:
                            if obj.type == start.type and obj.name!= start.name and not str(obj) in appended:
                                if moving_object.motion_model == MotionModels.REEDSSHEPP or moving_object.motion_model == MotionModels.SE2:
                                    point = (obj.configuration[0]/map.resolution, map.image.size[1] - obj.configuration[1]/map.resolution)
                                if moving_object.motion_model == MotionModels.SE3:
                                    point = (obj.configuration[0], obj.configuration[1], obj.configuration[2])

                                # a point is in the hull if and only if for every equation (describing the facets) the dot product between the point and
                                # the normal vector (eq[:-1]) plus the offset (eq[-1]) is less than or equal to zero. You may want to compare to a small,
                                # positive constant tolerance = 1e-12 rather than to zero because of issues of numerical precision
                                # (otherwise, you may find that a vertex of the convex hull is not in the convex hull)
                                if not all((np.dot(eq[:-1], point) + eq[-1] <= tolerance) for eq in hull.equations):
                                    unreachable_configurations.append(obj)
                                    appended.append(str(obj))

                #plot_reachability_data(map, moving_object.motion_model, hull=hull, points=np.array(sampled_vertices), start_pose=start, goal_pose=goal, obstacles= obstacles, cc= cc, unreachable_configurations=unreachable_configurations)

        
        if topological_refinement in [SupportedTopologicalRefinement.ALL, SupportedTopologicalRefinement.OBS]:
            collision_obstacles = cc.get_collision_objects()

        return is_valid, path, unreachable_configurations, collision_obstacles, planning_data
