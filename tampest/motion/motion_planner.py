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

from functools import partial
import alphashape
from shapely import Point
from shapely.geometry import Polygon
from ompl import base as ob
from ompl import geometric as og
from ompl import control as oc
from typing import Optional, Tuple
import numpy as np
import time
from shapely.affinity import *
from tampest.motion.map import Map
from tampest.motion.collision_checker import (
    CollisionChecker,
    CollisionChecker3D,
    CollisionChecker2D,
)
from tampest.motion.motion_validator import SpaceTimeMotionValidator
from unified_planning.shortcuts import *
from tampest.motion.motion_planning_data import (
    MotionPlanningData,
    SupportedPlanner,
    SupportedTopologicalRefinement,
)


class MotionPlanner:

    def __init__(self) -> None:
        pass

    def get_motion_model(self, moving_objects: List[MovableObject]) -> MotionModels:
        motion_model = None
        for mo in moving_objects:
            if mo.motion_model is None:
                print("Missing at least one motion model")
            else:
                if motion_model is None:
                    motion_model = mo.motion_model
                if mo.motion_model != motion_model:
                    print(f"Objects have different motion models")

        return motion_model

    def get_control_model(self, moving_objects: List[MovableObject]):
        control_model = None
        for mo in moving_objects:
            if mo.control_model is None:
                print("Missing at least one control model")
            else:
                if control_model is None:
                    control_model = mo.control_model
                if mo.control_model != control_model:
                    print(f"Objects have different control models")
        return control_model

    def get_state_space(
        self, moving_objects: Dict[int, MovableObject], motion_model: MotionModels
    ) -> ob.StateSpace:
        space = None
        selected_space = None

        mov_objs = list(moving_objects.values())

        if len(mov_objs) > 1:
            space = ob.CompoundStateSpace()

        if motion_model == MotionModels.REEDSSHEPP:
            for i in range(len(mov_objs)):
                if (
                    mov_objs[0].motion_parameters is None
                    or mov_objs[0].motion_parameters["turning_radius"] is None
                ):
                    raise ValueError("At least one turning radius missing.")
            if len(mov_objs) == 1:
                k = list(moving_objects.keys())[0]
                space = ob.ReedsSheppStateSpace(
                    mov_objs[0].motion_parameters["turning_radius"]
                )
            else:
                i = 0
                for k, v in moving_objects.items():
                    space.addSubspace(
                        ob.ReedsSheppStateSpace(v.motion_parameters["turning_radius"]),
                        1.0,
                    )
                    i += 1

        elif motion_model == MotionModels.SE2:
            selected_space = ob.SE2StateSpace()
        elif motion_model == MotionModels.SE3:
            selected_space = ob.SE3StateSpace()

        if selected_space is not None:
            if len(moving_objects) == 1:
                space = selected_space
            else:
                for _, v in moving_objects.items():
                    space.addSubspace(selected_space, 1.0)

        return space

    def get_map_bounds(
        self, map: Map, motion_model: MotionModels
    ) -> ob.RealVectorBounds:
        bounds = None
        if motion_model == MotionModels.REEDSSHEPP or motion_model == MotionModels.SE2:
            bounds = ob.RealVectorBounds(2)
            bounds.setLow(0.0)
            bounds.high[0] = map.image.size[0]
            bounds.high[1] = map.image.size[1]

        if motion_model == MotionModels.SE3:
            env_bounds = map.mesh.bounds.T
            bounds = ob.RealVectorBounds(3)
            bounds.low[0] = env_bounds[0][0]
            bounds.low[1] = env_bounds[1][0]
            bounds.low[2] = env_bounds[2][0]
            bounds.high[0] = env_bounds[0][1]
            bounds.high[1] = env_bounds[1][1]
            bounds.high[2] = env_bounds[2][1]

        # print(f"map bounds: bounds.low: {bounds.low[0]}, {bounds.low[1]}")
        # print(f"map bounds: bounds.high: {bounds.high[0]}, {bounds.high[1]}")
        return bounds

    def get_planner(
        self,
        si: ob.SpaceInformation,
        planner: SupportedPlanner,
        control_model,
        is_state_space: bool,
    ):
        selected_planner = None

        # set the planner
        if planner == SupportedPlanner.RRT:
            if control_model is not None and not is_state_space:
                selected_planner = oc.RRT(si)
            selected_planner = og.RRT(si)
        elif planner == SupportedPlanner.STRRTstar:
            if control_model is not None and not is_state_space:
                raise ("STRRTstar not available for ompl control")
            if is_state_space:
                strrt = og.STRRTstar(si)
                selected_planner = strrt
                # selected_planner.setOptimumApproxFactor(0.001)
            else:
                raise ("STRRTstar not available without v_max")
        elif planner == SupportedPlanner.LazyRRT:
            if control_model is not None and not is_state_space:
                raise ("LazyRRT not available for ompl control")
            selected_planner = og.LazyRRT(si)
        elif planner == SupportedPlanner.RRTConnect:
            if control_model is not None and not is_state_space:
                raise ("RRTConnect not available for ompl control")
            selected_planner = og.RRTConnect(si)
        elif planner == SupportedPlanner.RRTstar:
            if control_model is not None and not is_state_space:
                raise ("RRTstar not available for ompl control")
            selected_planner = og.RRTstar(si)
        elif planner == SupportedPlanner.KPIECE1:
            if control_model is not None:
                if is_state_space:
                    selected_planner = oc.KPIECE1(si)
                else:
                    raise ("KPIECE1 not available for space time state space")
            else:
                selected_planner = og.KPIECE1(si)
        elif planner == SupportedPlanner.PRM:
            if control_model is not None:
                raise ("PRM not available for ompl control")
            selected_planner = og.PRM(si)
        elif planner == SupportedPlanner.LazyPRM:
            if control_model is not None:
                raise ("LazyPRM not available for ompl control")
            selected_planner = og.LazyPRM(si)
        elif planner == SupportedPlanner.EST:
            if control_model is not None and not is_state_space:
                selected_planner = oc.EST(si)
            selected_planner = og.EST(si)
        elif planner == SupportedPlanner.SBL:
            if control_model is not None:
                raise ("SBL not available for ompl control")
            selected_planner = og.SBL(si)

        return selected_planner

    def set_config(
        self,
        state,
        *,
        x: float,
        y: float,
        z: Optional[float] = None,
        rot: Union[float, tuple],
        is_se3=False,
    ):
        if is_se3:
            qx, qy, qz, qw = rot
            state.setXYZ(x, y, z)
            state.rotation().setAxisAngle(qx, qy, qz, qw)
        else:
            state.setXY(x, y)
            state.setYaw(rot)

    def set_problem(
        self,
        map: Map,
        state_space_map: Dict[int, int],
        moving_objects: Dict[int, MovableObject],
        start_configs: Dict[int, ConfigurationObject],
        goal_configs: Dict[int, ConfigurationObject],
        planner: SupportedPlanner,
        cc: CollisionChecker,
        *,
        action_starts: Optional[Dict[int, float]] = None,
        d_max: Optional[Dict[int, float]] = None,
        distance: Optional[float] = None,
    ) -> og.SimpleSetup:

        # set state space space

        motion_model = self.get_motion_model(moving_objects.values())

        if motion_model is None:
            raise ("Missing motion model. Unable to set up state space.")

        control_model = self.get_control_model(moving_objects.values())

        space = self.get_state_space(moving_objects, motion_model)
        if space is None:
            raise NotImplementedError

        # set map bounds
        map_bounds = self.get_map_bounds(map, motion_model)

        if len(moving_objects) == 1:
            space.setBounds(map_bounds)
        else:
            for i in range(len(moving_objects)):
                space.getSubspace(i).setBounds(map_bounds)

        # set up the motion setup
        is_space_time = False
        if d_max and len(d_max) == len(moving_objects):
            is_space_time = True  # update is_state_space according to d_max

        if control_model is not None and motion_model in {
            MotionModels.REEDSSHEPP,
            MotionModels.SE2,
        }:

            if is_space_time:

                v_max = None

                for _, v in moving_objects.items():
                    if (
                        v.control_parameters is None
                        or v.control_parameters["v_max"] is None
                    ):
                        raise ValueError("Missing at least one v_max")
                    else:
                        obj_v_max = v.control_parameters["v_max"]
                        if v_max is None or obj_v_max > v_max:
                            v_max = obj_v_max

                tspace = ob.SpaceTimeStateSpace(space, v_max)

                if d_max and len(d_max) == 1 and list(d_max.values())[0] is not None:
                    d_max = float(list(d_max.values())[0])
                    tspace.setTimeBounds(
                        0.0, d_max
                    )  # Set lower and upper time bounds for the time component

                motion_setup = og.SimpleSetup(tspace)
                motion_setup.getSpaceInformation().setMotionValidator(
                    SpaceTimeMotionValidator(
                        motion_setup.getSpaceInformation(),
                        list(moving_objects.values()),
                        action_starts,
                    )
                )

            else:
                raise NotImplementedError("Pure control setup not implemented.")

        elif motion_model in {
            MotionModels.REEDSSHEPP,
            MotionModels.SE2,
            MotionModels.SE3,
        }:
            motion_setup = og.SimpleSetup(space)
        else:
            raise NotImplementedError

        # set the planner
        selected_planner = self.get_planner(
            motion_setup.getSpaceInformation(), planner, control_model, is_space_time
        )

        if selected_planner is None:
            raise ValueError("Unable to set up planner.")

        # set the range of the planner
        if distance is not None:
            selected_planner.setRange(distance)

        motion_setup.setPlanner(selected_planner)

        # set state validity checker for this space
        motion_setup.setStateValidityChecker(
            ob.StateValidityCheckerFn(
                partial(cc.isStateValid, motion_setup.getSpaceInformation())
            )
        )

        if motion_model in {MotionModels.REEDSSHEPP, MotionModels.SE2}:
            motion_setup.getSpaceInformation().setStateValidityCheckingResolution(0.01)

        if motion_model == MotionModels.SE3:
            motion_setup.getSpaceInformation().setStateValidityCheckingResolution(0.05)

        # start and goal configurations
        ss = ob.State(motion_setup.getStateSpace())
        gs = ob.State(motion_setup.getStateSpace())

        start = ss()
        goal = gs()

        if motion_model in {MotionModels.REEDSSHEPP, MotionModels.SE2}:
            if len(moving_objects) > 1:
                if is_space_time:
                    start = start[0]
                    goal = goal[0]
                for k, _ in moving_objects.items():
                    i = state_space_map[k]
                    self.set_config(
                        start[i],
                        x=start_configs[k].configuration.x / map.resolution,
                        y=map_bounds.high[1]
                        - start_configs[k].configuration.y / map.resolution,
                        rot=start_configs[k].configuration.theta,
                    )
                    self.set_config(
                        goal[i],
                        x=goal_configs[k].configuration.x / map.resolution,
                        y=map_bounds.high[1]
                        - goal_configs[k].configuration.y / map.resolution,
                        rot=goal_configs[k].configuration.theta,
                    )

            else:
                if is_space_time:
                    start = start[0]
                    goal = goal[0]
                start_configs = list(start_configs.values())
                self.set_config(
                    start,
                    x=start_configs[0].configuration.x / map.resolution,
                    y=map_bounds.high[1]
                    - start_configs[0].configuration.y / map.resolution,
                    rot=start_configs[0].configuration.theta,
                )
                self.set_config(
                    goal,
                    x=goal_configs[0].configuration.x / map.resolution,
                    y=map_bounds.high[1]
                    - goal_configs[0].configuration.y / map.resolution,
                    rot=goal_configs[0].configuration.theta,
                )

        # Assumption for SE3: (x, y, z, rw, rx, ry, rz)
        if motion_model == MotionModels.SE3:
            if len(moving_objects) > 1:
                for k, _ in moving_objects.items():
                    i = state_space_map[k]
                    self.set_config(
                        start[i],
                        x=start_configs[k].configuration.x,
                        y=start_configs[k].configuration.y,
                        z=start_configs[k].configuration.theta,
                        rot=(
                            start_configs[k].configuration[3],
                            start_configs[k].configuration[4],
                            start_configs[k].configuration[5],
                            start_configs[k].configuration[6],
                        ),
                    )
                    self.set_config(
                        goal[i],
                        x=goal_configs[k].configuration.x,
                        y=goal_configs[k].configuration.y,
                        z=goal_configs[k].configuration.theta,
                        rot=(
                            goal_configs[k].configuration[3],
                            goal_configs[k].configuration[4],
                            goal_configs[k].configuration[5],
                            goal_configs[k].configuration[6],
                        ),
                        is_se3=True,
                    )
            else:
                start_configs = list(start_configs.values())
                self.set_config(
                    start,
                    x=start_configs[0].configuration.x,
                    y=start_configs[0].configuration.y,
                    z=start_configs[0].configuration.theta,
                    rot=(
                        start_configs[0].configuration[3],
                        start_configs[0].configuration[4],
                        start_configs[0].configuration[5],
                        start_configs[0].configuration[6],
                    ),
                )
                self.set_config(
                    goal,
                    x=goal_configs[0].configuration.x,
                    y=goal_configs[0].configuration.y,
                    z=goal_configs[0].configuration.theta,
                    rot=(
                        goal_configs[0].configuration[3],
                        goal_configs[0].configuration[4],
                        goal_configs[0].configuration[5],
                        goal_configs[0].configuration[6],
                    ),
                    is_se3=True,
                )

        motion_setup.setStartAndGoalStates(ss, gs)

        return motion_setup

    def get_solution(
        self,
        motion_setup,
        *,
        planning_time: Optional[float] = 1.0,
        interpolate: Optional[bool] = True,
        simplified: Optional[bool] = True,
    ):
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
                planning_data.n_waypoints_after_interpolation = (
                    solution_path.getStateCount()
                )
                # print(path.printAsMatrix())

            planning_data.path_length = solution_path.length()

        else:
            print(
                "Exact solution not found. Distance to actual goal equals to %g"
                % motion_setup.getProblemDefinition().getSolutionDifference()
            )

        planner_data = ob.PlannerData(motion_setup.getSpaceInformation())
        motion_setup.getPlannerData(planner_data)

        return solution_path, planner_data, planning_data

    def get_control_path_from_time_space_solution(
        self, motion_setup, solution_path, n_sub: int
    ) -> Optional[List[Tuple[float, ...]]]:

        paths = {}
        durations = {}
        states = solution_path.getStates()

        for i in range(n_sub):

            durations[i] = 0.0

            # state 0 - controls: 0 0 - durations: 0
            # state i (i>0) - controls needed to reach state i from state i+1 - durations needed to reach state i from state i+1
            if n_sub > 1:
                state = states[0][0][i]
            else:
                state = states[0][0]

            paths[i] = [(state.getX(), state.getY(), state.getYaw(), 0, 0, 0)]

        starts = {}
        for i in range(n_sub):
            starts[i] = 0.0

            started = False
            tmp_dt = 0

            for j in range(1, solution_path.getStateCount()):

                if n_sub > 1:
                    state0 = states[j - 1][0][i]
                    state1 = states[j][0][i]
                else:
                    state0 = states[j - 1][0]
                    state1 = states[j][0]

                t0 = (
                    motion_setup.getSpaceInformation()
                    .getStateSpace()
                    .getStateTime(states[j - 1])
                )
                x0 = state0.getX()
                y0 = state0.getY()
                yaw0 = state0.getYaw()

                t1 = (
                    motion_setup.getSpaceInformation()
                    .getStateSpace()
                    .getStateTime(states[j])
                )
                x1 = state1.getX()
                y1 = state1.getY()
                yaw1 = state1.getYaw()

                # u = retrieve_u((x0, y0, yaw0), (x1, y1, yaw1))

                dt = t1 - t0

                dx = x1 - x0
                dy = y1 - y0
                dyaw = yaw1 - yaw0

                v_x = dx / dt
                v_y = dy / dt
                v = np.sqrt(v_x**2 + v_y**2)

                omega = dyaw / dt

                # remove initial wait
                # if (dx!=0 or dy!=0 or dyaw!=0): # epsilon
                if (
                    abs(dx) > 0.00001 or abs(dy) > 0.00001 or abs(dyaw) > 0.00001
                ):  # epsilon

                    if not started:
                        starts[i] = t0
                    started = True
                    durations[i] += tmp_dt + dt
                    tmp_dt = 0

                elif started:
                    tmp_dt += dt

                paths[i].append(
                    (state1.getX(), state1.getY(), state1.getYaw(), v, omega, dt)
                )
                # path[i].append((states[j][0][i].getX(), states[j][0][i].getY(), states[j][0][i].getYaw(), *u, dt))

        return paths, {i: (v, durations[i]) for i, v in starts.items()}

    def get_map(self, problem_objects):
        map = None

        for o in problem_objects:
            if o.type.is_configuration_type():
                if map is None:
                    map = Map().get_from_file(o.type.occupancy_map.filename)
                    break
        return map

    # motion_constraints = {mc: (action_start, d_max, movable.object(), starting.object(), waypoint.object(), fixed_obstacles_pos)}
    # constraints_map = {(ai, mc): 0} ------------ used to assigned an id (name) to each motion planning problem
    # paths = {(ai, mc): path}
    # current_durations = {ai: current_duration}
    # obstacles / unreachables = {ai: ...}

    def find_key_by_value(self, constraints_map, N_value):
        for (ai, mc), N in constraints_map.items():
            if N == N_value:
                return (ai, mc)
        return None

    def check_motion_constraints(
        self,
        motion_constraints,
        constraints_map,
        problem_objects,
        *,
        planning_time: Optional[float] = 1.0,
        interpolate: Optional[bool] = True,
        simplified: Optional[bool] = True,
        tolerance: Optional[float] = 0.0,
        distance: Optional[float] = None,
        motion_planner: Optional[SupportedPlanner] = SupportedPlanner.RRT,
        topological_refinement: Optional[
            SupportedTopologicalRefinement
        ] = SupportedTopologicalRefinement.ALL,
        max_radius_bound: Optional[bool] = False,
    ) -> Tuple[
        bool,
        Dict[Tuple[MotionActivity, MotionConstraint], Any],
        Dict[MotionActivity, Tuple[float, float]],
        Dict[Tuple[MotionActivity, MotionConstraint], List[ConfigurationObject]],
        Dict[Tuple[MotionActivity, MotionConstraint], List[MovableObject]],
        MotionPlanningData,
    ]:

        action_starts = {}
        moving_objs = {}
        start_configs = {}
        goal_configs = {}
        d_max = {}
        obstacles = {}

        # (0 action_start, 1 d_max, 2 movable.object(), 3 starting.object(), 4 waypoint.object(), 5 fixed_obstacles_pos)
        for (ai, mc), value in motion_constraints.items():
            k = constraints_map[(ai, mc)]
            action_starts[k] = value[0]
            d_max[k] = value[1]
            moving_objs[k] = value[2]
            start_configs[k] = value[3]
            goal_configs[k] = value[4]
            obstacles.update(value[5])

        (
            is_valid,
            paths,
            durations,
            unreachable_configurations,
            collision_obstacles,
            planning_data,
        ) = self.check_motion_constraint(
            moving_objects=moving_objs,
            action_starts=action_starts,
            start_configs=start_configs,
            goal_configs=goal_configs,
            obstacles=obstacles,
            problem_objects=problem_objects,
            d_max=d_max,
            planning_time=planning_time,
            interpolate=interpolate,
            simplified=simplified,
            tolerance=tolerance,
            distance=distance,
            motion_planner=motion_planner,
            topological_refinement=topological_refinement,
            max_radius_bound=max_radius_bound,
        )

        remapped_paths = {}
        remapped_durations = {}
        remapped_unreach = {}
        remapped_obs = {}

        # paths = {(ai, mc): path}
        # current_durations = {ai: current_duration}
        for k, v in paths.items():
            (ai, mc) = self.find_key_by_value(constraints_map, k)
            remapped_paths[(ai, mc)] = v

        for k, v in durations.items():
            (ai, _) = self.find_key_by_value(constraints_map, k)
            remapped_durations[ai] = v

        for k, v in unreachable_configurations.items():
            (ai, mc) = self.find_key_by_value(constraints_map, k)
            remapped_unreach[(ai, mc)] = v

        for k, v in collision_obstacles.items():
            (ai, mc) = self.find_key_by_value(constraints_map, k)
            remapped_obs[(ai, mc)] = v

        return (
            is_valid,
            remapped_paths,
            remapped_durations,
            remapped_unreach,
            remapped_obs,
            planning_data,
        )

    def check_motion_constraint(
        self,
        moving_objects: Dict[int, MovableObject],
        start_configs: Dict[int, ConfigurationObject],
        goal_configs: Dict[int, ConfigurationObject],
        obstacles,
        problem_objects,
        *,
        action_starts: Optional[Dict[int, float]] = None,
        d_max: Optional[Dict[int, float]] = None,
        planning_time: Optional[float] = 1.0,
        interpolate: Optional[bool] = True,
        simplified: Optional[bool] = True,
        tolerance: Optional[float] = 0.0,
        distance: Optional[float] = None,
        motion_planner: Optional[SupportedPlanner] = SupportedPlanner.RRT,
        topological_refinement: Optional[
            SupportedTopologicalRefinement
        ] = SupportedTopologicalRefinement.ALL,
        max_radius_bound: Optional[bool] = False,
    ) -> Tuple[
        bool,
        Optional[float],
        Optional[List[Tuple[float, ...]]],
        Optional[List[ConfigurationObject]],
        Optional[List[MovableObject]],
        MotionPlanningData,
    ]:

        motion_model = self.get_motion_model(moving_objects.values())
        control_model = self.get_control_model(moving_objects.values())

        is_valid = False

        is_state_space = False
        if d_max is not None:
            is_state_space = True

        map = self.get_map(problem_objects)

        state_space_map = {}
        i = 0
        for k, v in moving_objects.items():
            state_space_map[i] = k
            i += 1

        if motion_model in {MotionModels.REEDSSHEPP, MotionModels.SE2}:
            cc = CollisionChecker2D(
                moving_objects=moving_objects,
                start_configs=start_configs,
                index_map=state_space_map,
                d_max=list(d_max.values()) if d_max else None,
                map=map,
                movable_obstacles=obstacles,
                topological_refinement=topological_refinement,
                max_radius_bound=max_radius_bound,
            )
        elif motion_model == MotionModels.SE3:
            cc = CollisionChecker3D(
                moving_objects=moving_objects,
                start_configs=start_configs.values,
                index_map=state_space_map,
                d_max=list(d_max.values()) if d_max else None,
                map=map,
                movable_obstacles=obstacles,
                topological_refinement=topological_refinement,
                max_radius_bound=max_radius_bound,
            )
        else:
            raise NotImplementedError("Unsupported motion model:", motion_model)

        motion_problem = self.set_problem(
            map,
            state_space_map,
            moving_objects,
            start_configs,
            goal_configs,
            motion_planner,
            cc,
            d_max=d_max,
            distance=distance,
            action_starts=action_starts,
        )
        # cc.plot_current_state(start_configs, goal_configs)

        if simplified and control_model is not None:
            print(
                f"simplifySolution() method not available in Control Space. Giving back the original solution."
            )
            simplified = False

        solution_path, planner_data, planning_data = self.get_solution(
            motion_problem,
            planning_time=planning_time,
            interpolate=interpolate,
            simplified=simplified,
        )

        unreachable_configurations = {}
        collision_obstacles = {}

        paths = {}
        durations = {}

        remapped_paths = {}
        remapped_durations = {}

        goal_reached = []

        if solution_path:
            if control_model is not None and motion_model in {
                MotionModels.SE2,
                MotionModels.REEDSSHEPP,
            }:

                if is_state_space:
                    paths, durations = self.get_control_path_from_time_space_solution(
                        motion_problem, solution_path, len(moving_objects)
                    )

                    for k, v in paths.items():
                        remapped_paths[state_space_map[k]] = v

                    for k, v in durations.items():
                        remapped_durations[state_space_map[k]] = v

                else:
                    raise NotImplementedError("Pure control setup not implemented.")

                if len(durations) == 1 and list(durations.values())[0][1] <= d_max[0]:

                    real_duration = list(durations.values())[0][1]
                    is_valid = True

                    print(f"New duration: {real_duration}")

                elif len(paths) > 1 and len(durations) > 1:
                    is_valid = True
                else:
                    assert False

            elif motion_model in {MotionModels.REEDSSHEPP, MotionModels.SE2}:
                is_valid = True
                if len(moving_objects) == 1:
                    k = list(moving_objects.keys())[0]
                    for state in solution_path.getStates():
                        if state_space_map[k] not in remapped_paths:
                            remapped_paths[state_space_map[k]] = [
                                (state.getX(), state.getY(), state.getYaw())
                            ]
                        else:
                            remapped_paths[state_space_map[k]].append(
                                (state.getX(), state.getY(), state.getYaw())
                            )
                else:
                    for k, _ in moving_objects.items():
                        for state in solution_path.getStates()[i]:
                            if state_space_map[k] not in remapped_paths:
                                remapped_paths[state_space_map[k]] = [
                                    (state.getX(), state.getY(), state.getYaw())
                                ]
                            else:
                                remapped_paths[state_space_map[k]].append(
                                    (state.getX(), state.getY(), state.getYaw())
                                )

            elif motion_model == MotionModels.SE3:
                is_valid = True
                if len(moving_objects) == 1:
                    k = list(moving_objects.keys())[0]
                    for state in solution_path.getStates():
                        if state_space_map[k] not in remapped_paths:
                            remapped_paths[state_space_map[k]] = [
                                (
                                    state.getX(),
                                    state.getY(),
                                    state.getZ(),
                                    state.rotation().w,
                                    state.rotation().x,
                                    state.rotation().y,
                                    state.rotation().z,
                                )
                            ]
                        else:
                            remapped_paths[state_space_map[k]].append(
                                (
                                    state.getX(),
                                    state.getY(),
                                    state.getZ(),
                                    state.rotation().w,
                                    state.rotation().x,
                                    state.rotation().y,
                                    state.rotation().z,
                                )
                            )
                else:
                    for k, _ in moving_objects.items():
                        for state in solution_path.getStates()[i]:
                            if state_space_map[k] not in remapped_paths:
                                remapped_paths[state_space_map[k]] = [
                                    (
                                        state.getX(),
                                        state.getY(),
                                        state.getZ(),
                                        state.rotation().w,
                                        state.rotation().x,
                                        state.rotation().y,
                                        state.rotation().z,
                                    )
                                ]
                            else:
                                remapped_paths[state_space_map[k]].append(
                                    (
                                        state.getX(),
                                        state.getY(),
                                        state.getZ(),
                                        state.rotation().w,
                                        state.rotation().x,
                                        state.rotation().y,
                                        state.rotation().z,
                                    )
                                )
        else:
            for k, _ in moving_objects.items():
                unreachable_configurations[k] = [goal_configs[k]]
                collision_obstacles[k] = list(obstacles.keys())

            if planner_data:

                goal_equivalent_area = {}
                coverage_area = {}

                num_vertices = planner_data.numVertices()

                reachability_list = []
                goal_vertices = set()
                goal_equivalent_vertices = []

                all_others = set()

                start_equivalent_vertices = []
                start_equivalent_area = {}

                def extract_sampled_state(state, key=None):
                    if is_state_space:
                        state = state[0]
                    if key is not None:
                        state = state[state_space_map[key]]

                    if motion_model in {MotionModels.REEDSSHEPP, MotionModels.SE2}:
                        return (state.getX(), state.getY())
                    elif motion_model == MotionModels.SE3:
                        return (state.getX(), state.getY(), state.getZ())
                    return None

                # Step 1: Collect reachability and coverage area
                for i in range(num_vertices):
                    reachable_data = ob.PlannerData(
                        motion_problem.getSpaceInformation()
                    )
                    planner_data.extractReachable(i, reachable_data)
                    vertex = planner_data.getVertex(i)
                    reachability_list.append((vertex, reachable_data))

                    if planner_data.isGoalVertex(i) and vertex not in goal_vertices:
                        goal_vertices.add(vertex)
                    else:
                        all_others.add(vertex)

                    state = vertex.getState()
                    if len(moving_objects) > 1:
                        for key in moving_objects:
                            sampled_state = extract_sampled_state(state, key)
                            if key not in coverage_area:
                                coverage_area[key] = []
                            if sampled_state not in coverage_area[key]:
                                coverage_area[key].append(sampled_state)
                    else:
                        sampled_state = extract_sampled_state(state)
                        if 0 not in coverage_area:
                            coverage_area[0] = []
                        if sampled_state not in coverage_area[0]:
                            coverage_area[0].append(sampled_state)

                # Step 2: Identify goal-equivalent vertices and extract goal-equivalent areas
                for vertex, reachable in reachability_list:
                    for goal_v in goal_vertices:
                        if goal_v != vertex and reachable.vertexExists(goal_v):
                            goal_equivalent_vertices.append(vertex)

                for vertex in goal_equivalent_vertices:
                    state = vertex.getState()
                    if len(moving_objects) > 1:
                        for key in moving_objects:
                            sampled_state = extract_sampled_state(state, key)
                            if sampled_state is None:
                                continue
                            if key not in goal_equivalent_area:
                                goal_equivalent_area[key] = []
                            if sampled_state not in goal_equivalent_area[key]:
                                goal_equivalent_area[key].append(sampled_state)
                    else:
                        sampled_state = extract_sampled_state(state)
                        if 0 not in goal_equivalent_area:
                            goal_equivalent_area[0] = []
                        if sampled_state not in goal_equivalent_area[0]:
                            goal_equivalent_area[0].append(sampled_state)

                # Step 3: Identify start-equivalent vertices and extract start-equivalent areas
                start_equivalent_vertices = all_others.difference(
                    goal_equivalent_vertices
                )

                for vertex in start_equivalent_vertices:
                    state = vertex.getState()
                    if len(moving_objects) > 1:
                        for key in moving_objects:
                            sampled_state = extract_sampled_state(state, key)
                            if sampled_state is None:
                                continue
                            if key not in start_equivalent_area:
                                start_equivalent_area[key] = []
                            if sampled_state not in start_equivalent_area[key]:
                                start_equivalent_area[key].append(sampled_state)

                hull = {}

                def compute_convex_hulls(area_dict):
                    hulls = {}
                    for k, points in area_dict.items():
                        if len(points) > 2:
                            start_time = time.time()
                            hulls[k] = alphashape.alphashape(np.array(points), 0.1)
                            planning_data.convex_hull_time = time.time() - start_time

                    return hulls

                def is_point_in_hull(point, hull):
                    return hull.contains(Point(*point))

                # Compute convex hulls
                area_source = goal_equivalent_area if is_state_space else coverage_area
                hull = compute_convex_hulls(area_source)

                # Evaluate objects against convex hulls
                for k, convex_h in hull.items():
                    if convex_h is None:
                        continue

                    def to_closed_polygon(coords):
                        """Return a valid closed polygon if possible, else None."""
                        if len(coords) >= 3:
                            if coords[0] != coords[-1]:
                                coords = coords + [coords[0]]  # don't modify in-place
                            return Polygon(coords)
                        return None

                    if k in start_equivalent_area:
                        # Convert to shapely Polygons
                        poly1 = to_closed_polygon(goal_equivalent_area[k])
                        poly2 = to_closed_polygon(start_equivalent_area[k])

                        assert poly1 is not None, f"Invalid goal_equivalent_area[{k}]"

                        if poly2 is not None:
                            # Check if they intersect (overlap)
                            if poly1.intersects(poly2):
                                goal_reached.append(k)
                        else:
                            for p in start_equivalent_area[k]:
                                if poly1.contains(Point(p)):
                                    goal_reached.append(k)
                                    break

                    if topological_refinement not in [
                        SupportedTopologicalRefinement.ALL,
                        SupportedTopologicalRefinement.UNREACH,
                    ]:
                        continue

                    start_type = start_configs[k].type
                    start_name = start_configs[k].name

                    for obj in problem_objects:
                        if (
                            obj.type != start_type
                            or obj.name == start_name
                            or obj == goal_configs[k]
                        ):
                            continue

                        # Convert object configuration to appropriate point
                        if motion_model in {MotionModels.REEDSSHEPP, MotionModels.SE2}:
                            point = (
                                obj.configuration.x / map.resolution,
                                map.image.size[1]
                                - obj.configuration.y / map.resolution,
                            )
                        elif motion_model == MotionModels.SE3:
                            point = (
                                obj.configuration.x,
                                obj.configuration.y,
                                obj.configuration.theta,
                            )
                        else:
                            continue  # Unsupported motion model

                        point_in_hull = is_point_in_hull(point, convex_h)

                        # a point is in the hull if and only if for every equation (describing the facets) the dot product between the point and
                        # the normal vector (eq[:-1]) plus the offset (eq[-1]) is less than or equal to zero. You may want to compare to a small,
                        # positive constant tolerance = 1e-12 rather than to zero because of issues of numerical precision
                        # (otherwise, you may find that a vertex of the convex hull is not in the convex hull)
                        if (not point_in_hull and not is_state_space) or (
                            point_in_hull and is_state_space
                        ):
                            if k not in unreachable_configurations:
                                unreachable_configurations[k] = []
                            if obj not in unreachable_configurations[k]:
                                unreachable_configurations[k].append(obj)

                # from tampest.motion.plotting import plot_reachability_data
                # plot_reachability_data(
                #     map,
                #     motion_model,
                #     hull=hull,
                #     points=coverage_area,  # goal_equivalent_area
                #     start_poses=start_configs,
                #     goal_poses=goal_configs,
                #     obstacles=obstacles,
                #     cc=cc,
                #     unreachable_configurations=unreachable_configurations,
                # )

        if (
            topological_refinement
            in [
                SupportedTopologicalRefinement.ALL,
                SupportedTopologicalRefinement.OBS,
            ]
            and not solution_path
        ):
            collision_obstacles = cc.get_collision_objects()
            for k in goal_reached or []:
                collision_obstacles.pop(k, None)

        if not is_valid:
            print(f"Unreachable configurations: {unreachable_configurations}")
            print(f"Collision obstacles: {collision_obstacles}")

        return (
            is_valid,
            remapped_paths,
            remapped_durations,
            unreachable_configurations,
            collision_obstacles,
            planning_data,
        )
