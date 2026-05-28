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

from functools import partial
from collections import deque
from itertools import combinations
import alphashape
from shapely import Point
from shapely.geometry import Polygon
from ompl import base as ob
from ompl import geometric as og
from ompl import control as oc
from typing import Callable, Optional, Tuple
import numpy as np
import time
from shapely.affinity import *

try:
    import cv2 as _cv2
except ImportError:
    _cv2 = None
from tampest.motion.map import Map, Map2D
from tampest.motion.collision_checker import (
    CollisionChecker,
    CollisionChecker3D,
    CollisionChecker2D,
)
from tampest.motion.motion_sampler import StartAwareSampler
from tampest.motion.motion_validator import SpaceTimeMotionValidator
from unified_planning.shortcuts import *
from tampest.motion.motion_planning_data import (
    MotionPlanningData,
    SupportedPlanner,
    SupportedTopologicalRefinement,
)


def make_sampler_allocator(
    start_timings: Dict[int, float],
    start_configs: Dict[int, ConfigurationObject],
    map: Map2D,
) -> Callable[[ob.StateSpace], ob.StateSampler]:
    def allocator(space: ob.StateSpace) -> ob.StateSampler:
        return StartAwareSampler(space, start_timings, start_configs, map)

    return allocator


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
        if motion_model in (MotionModels.REEDSSHEPP, MotionModels.SE2):
            bounds = ob.RealVectorBounds(2)
            bounds.setLow(0.0)
            bounds.high[0] = map.image.size[0]
            bounds.high[1] = map.image.size[1]

        elif motion_model == MotionModels.SE3:
            env_bounds = map.mesh.bounds.T
            bounds = ob.RealVectorBounds(3)
            bounds.low[0] = env_bounds[0][0]
            bounds.low[1] = env_bounds[1][0]
            bounds.low[2] = env_bounds[2][0]
            bounds.high[0] = env_bounds[0][1]
            bounds.high[1] = env_bounds[1][1]
            bounds.high[2] = env_bounds[2][1]

        else:
            raise NotImplementedError(f"Bounds for {motion_model} not found.")

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
            selected_planner = og.RRT(si)
        elif planner == SupportedPlanner.STRRTstar:
            if control_model is not None and not is_state_space:
                raise Exception("STRRTstar not available for ompl control")
            if is_state_space:
                strrt = og.STRRTstar(si)
                selected_planner = strrt
                # selected_planner.setOptimumApproxFactor(0.001)
            else:
                raise Exception("STRRTstar not available without v_max")
        elif planner == SupportedPlanner.LazyRRT:
            if control_model is not None and not is_state_space:
                raise Exception("LazyRRT not available for ompl control")
            selected_planner = og.LazyRRT(si)
        elif planner == SupportedPlanner.RRTConnect:
            if control_model is not None and not is_state_space:
                raise Exception("RRTConnect not available for ompl control")
            selected_planner = og.RRTConnect(si)
        elif planner == SupportedPlanner.RRTstar:
            if control_model is not None and not is_state_space:
                raise Exception("RRTstar not available for ompl control")
            selected_planner = og.RRTstar(si)
        elif planner == SupportedPlanner.KPIECE1:
            if control_model is not None:
                if is_state_space:
                    selected_planner = oc.KPIECE1(si)
                else:
                    raise Exception("KPIECE1 not available for space time state space")
            else:
                selected_planner = og.KPIECE1(si)
        elif planner == SupportedPlanner.PRM:
            if control_model is not None:
                raise Exception("PRM not available for ompl control")
            selected_planner = og.PRM(si)
        elif planner == SupportedPlanner.LazyPRM:
            if control_model is not None:
                raise Exception("LazyPRM not available for ompl control")
            selected_planner = og.LazyPRM(si)
        elif planner == SupportedPlanner.EST:
            selected_planner = og.EST(si)
        elif planner == SupportedPlanner.SBL:
            if control_model is not None:
                raise Exception("SBL not available for ompl control")
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
        sequential_check: Optional[bool] = True,
    ) -> og.SimpleSetup:

        # set state space space

        motion_model = self.get_motion_model(moving_objects.values())

        if motion_model is None:
            raise Exception("Missing motion model. Unable to set up state space.")

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
                        map,
                        list(moving_objects.values()),
                        action_starts,
                        start_configs,
                        sequential_check,
                    )
                )

            else:
                motion_setup = og.SimpleSetup(space)

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
        if motion_model in {MotionModels.REEDSSHEPP, MotionModels.SE2}:
            motion_setup.setStateValidityChecker(
                ob.StateValidityCheckerFn(
                    partial(cc.isStateValid, motion_setup.getSpaceInformation())
                )
            )
        elif motion_model == MotionModels.SE3:
            motion_setup.setStateValidityChecker(
                ob.StateValidityCheckerFn(partial(cc.isStateValid))
            )
        else:
            NotImplementedError

        if motion_model in {MotionModels.REEDSSHEPP, MotionModels.SE2}:
            motion_setup.getSpaceInformation().setStateValidityCheckingResolution(0.01)

        elif motion_model == MotionModels.SE3:
            motion_setup.getSpaceInformation().setStateValidityCheckingResolution(0.05)
        else:
            NotImplementedError

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

        elif motion_model == MotionModels.SE3:
            if len(moving_objects) > 1:
                for k, _ in moving_objects.items():
                    i = state_space_map[k]
                    self.set_config(
                        start[i],
                        x=start_configs[k].configuration.x,
                        y=start_configs[k].configuration.y,
                        z=start_configs[k].configuration.z,
                        rot=(
                            start_configs[k].configuration.rw,
                            start_configs[k].configuration.rx,
                            start_configs[k].configuration.ry,
                            start_configs[k].configuration.rz,
                        ),
                        is_se3=True,
                    )
                    self.set_config(
                        goal[i],
                        x=goal_configs[k].configuration.x,
                        y=goal_configs[k].configuration.y,
                        z=goal_configs[k].configuration.z,
                        rot=(
                            goal_configs[k].configuration.rw,
                            goal_configs[k].configuration.rx,
                            goal_configs[k].configuration.ry,
                            goal_configs[k].configuration.rz,
                        ),
                        is_se3=True,
                    )
            else:
                start_configs = list(start_configs.values())
                self.set_config(
                    start,
                    x=start_configs[0].configuration.x,
                    y=start_configs[0].configuration.y,
                    z=start_configs[0].configuration.z,
                    rot=(
                        start_configs[0].configuration.rw,
                        start_configs[0].configuration.rx,
                        start_configs[0].configuration.ry,
                        start_configs[0].configuration.rz,
                    ),
                    is_se3=True,
                )
                self.set_config(
                    goal,
                    x=goal_configs[0].configuration.x,
                    y=goal_configs[0].configuration.y,
                    z=goal_configs[0].configuration.z,
                    rot=(
                        goal_configs[0].configuration.rw,
                        goal_configs[0].configuration.rx,
                        goal_configs[0].configuration.ry,
                        goal_configs[0].configuration.rz,
                    ),
                    is_se3=True,
                )
        else:
            NotImplementedError

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

    def get_control_path(
        self,
        motion_setup: og.SimpleSetup,
        solution_path,
        is_time_space: bool,
        action_starts: Dict[int, float],
        start_configs: Dict[int, ConfigurationObject],
        map: Map,
        n_sub: int,
        sequential_check: bool = False,
    ) -> Tuple[
        Dict[int, Tuple[float, float, float, float, float, float]],
        Dict[int, Tuple[float, float]],
    ]:
        """Extract control paths from a time-space solution.

        Superset of get_control_path_from_time_space_solution — also handles the
        sequential_check mode where time is relative to each robot's start offset.
        """

        def _get_state_xy_yaw(state, sub_idx):
            s = state[0] if is_time_space else state
            if n_sub > 1:
                s = s[sub_idx]
            return s.getX(), s.getY(), s.getYaw()

        def _get_config_xy_yaw(cfg):
            x = cfg.configuration.x / map.resolution
            y = map.image.size[1] - cfg.configuration.y / map.resolution
            theta = cfg.configuration.theta
            return x, y, theta

        paths = {}
        starts = {}
        durations = {}
        states = solution_path.getStates()

        for i in range(n_sub):
            started = False
            starts[i] = 0.0
            durations[i] = 0.0

            x0, y0, yaw0 = _get_state_xy_yaw(states[0], i)
            paths[i] = [(x0, y0, yaw0, 0, 0, 0)]
            if sequential_check:
                paths[i].append((x0, y0, yaw0, 0, 0, action_starts[i]))

            tmp_dt = 0

            for j in range(1, solution_path.getStateCount()):

                t0 = motion_setup.getStateSpace().getStateTime(states[j - 1])
                t1 = motion_setup.getStateSpace().getStateTime(states[j])

                if sequential_check:
                    x0, y0, yaw0 = _get_state_xy_yaw(states[j - 1], i)
                    x1, y1, yaw1 = _get_state_xy_yaw(states[j], i)
                else:
                    if is_time_space and t0 > 0 and t0 <= action_starts[i] + 0.1:
                        x0, y0, yaw0 = _get_config_xy_yaw(start_configs[i])
                    else:
                        x0, y0, yaw0 = _get_state_xy_yaw(states[j - 1], i)

                    if is_time_space and t1 > 0 and t1 <= action_starts[i] + 0.1:
                        x1, y1, yaw1 = _get_config_xy_yaw(start_configs[i])
                    else:
                        x1, y1, yaw1 = _get_state_xy_yaw(states[j], i)

                dt = t1 - t0
                dx = x1 - x0
                dy = y1 - y0
                dyaw = yaw1 - yaw0

                v_x = dx / dt
                v_y = dy / dt
                v = np.sqrt(v_x**2 + v_y**2)
                omega = dyaw / dt

                if abs(dx) > 0.00001 or abs(dy) > 0.00001 or abs(dyaw) > 0.00001:
                    if not started:
                        if sequential_check:
                            starts[i] = action_starts[i] + t0
                        else:
                            starts[i] = t0
                    started = True
                    durations[i] += tmp_dt + dt
                    tmp_dt = 0
                elif started:
                    tmp_dt += dt

                paths[i].append((x1, y1, yaw1, v, omega, dt))

        return paths, {i: (v, durations[i]) for i, v in starts.items()}

    def get_map(self, problem_objects: List[Object]):
        map_filename = None

        for o in problem_objects:
            if o.type.is_configuration_type():
                filename = o.type.occupancy_map.filename
                if map_filename is None:
                    map_filename = filename
                else:
                    assert (
                        filename == map_filename
                    ), f"Inconsistent occupancy maps: '{filename}' != '{map_filename}'"
        assert (
            map_filename is not None
        ), "No configuration-type object with a valid occupancy map found."

        return Map.get_from_file(map_filename)

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
        distance: Optional[float] = None,
        motion_planner: Optional[SupportedPlanner] = SupportedPlanner.RRT,
        topological_refinement: Optional[
            SupportedTopologicalRefinement
        ] = SupportedTopologicalRefinement.ALL,
        max_radius_bound: Optional[bool] = False,
        force_sequential_MP_check: Optional[bool] = False,
        hull_enabled: Optional[bool] = False,
        safety_distance: Optional[float] = None,
    ):

        action_starts = {}
        action_durations = {}
        moving_objs = {}
        start_configs = {}
        goal_configs = {}
        d_max = {}
        obstacles = {}

        # (0 action_start, 1 activity_duration, 2 d_max, 3 movable, 4 starting, 5 waypoint, 6 obstacles)
        for (ai, mc), value in motion_constraints.items():
            k = constraints_map[(ai, mc)]
            action_starts[k] = value[0]
            action_durations[k] = value[1]
            d_max[k] = value[2]
            moving_objs[k] = value[3]
            start_configs[k] = value[4]
            goal_configs[k] = value[5]
            obstacles.update(value[6])

        check_fn = self.sequential_check_motion_constraint if force_sequential_MP_check else self.check_motion_constraint

        extra_kwargs = {"safety_distance": safety_distance} if force_sequential_MP_check else {}

        (
            is_valid,
            is_temporal_valid,
            paths,
            durations,
            _reachable_configurations,
            unreachable_configurations,
            collision_obstacles,
            planning_data,
        ) = check_fn(
            moving_objects=moving_objs,
            action_starts=action_starts,
            action_durations=action_durations,
            start_configs=start_configs,
            goal_configs=goal_configs,
            obstacles=obstacles,
            problem_objects=problem_objects,
            d_max=d_max,
            hull_enabled=hull_enabled,
            planning_time=planning_time,
            interpolate=interpolate,
            simplified=simplified,
            distance=distance,
            motion_planner=motion_planner,
            topological_refinement=topological_refinement,
            max_radius_bound=max_radius_bound,
            **extra_kwargs,
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
            is_temporal_valid,
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
        action_durations: Optional[Dict[int, float]] = None,
        d_max: Optional[Dict[int, float]] = None,
        hull_enabled: Optional[bool] = False,
        planning_time: Optional[float] = 1.0,
        interpolate: Optional[bool] = True,
        simplified: Optional[bool] = True,
        distance: Optional[float] = None,
        motion_planner: Optional[SupportedPlanner] = SupportedPlanner.RRT,
        topological_refinement: Optional[
            SupportedTopologicalRefinement
        ] = SupportedTopologicalRefinement.ALL,
        max_radius_bound: Optional[bool] = False,
    ) -> Tuple[
        bool,
        bool,
        Optional[Dict[int, List[Tuple[float, ...]]]],
        Optional[Dict[int, Tuple[float, float]]],
        Optional[Dict[int, List[ConfigurationObject]]],
        Optional[Dict[int, List[ConfigurationObject]]],
        Optional[Dict[int, List[MovableObject]]],
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
                delays=action_starts,
                d_max=list(d_max.values()) if d_max else None,
                map=map,
                movable_obstacles=obstacles,
                topological_refinement=topological_refinement,
                max_radius_bound=max_radius_bound,
            )
        elif motion_model == MotionModels.SE3:
            cc = CollisionChecker3D(
                moving_objects=moving_objects,
                start_configs=start_configs,
                index_map=state_space_map,
                delays=action_starts,
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
        reachable_configurations = {}

        paths = {}
        durations = {}

        remapped_paths = {}
        remapped_durations = {}

        is_temporal_valid = True
        goal_reached = []
        use_bfs = False

        if solution_path:
            if control_model is not None and is_state_space and motion_model in {
                MotionModels.SE2,
                MotionModels.REEDSSHEPP,
            }:
                n = len(moving_objects)
                _as = {i: (action_starts[state_space_map[i]] if action_starts else 0.0) for i in range(n)}
                _sc = {i: start_configs[state_space_map[i]] for i in range(n)}
                paths, durations = self.get_control_path(
                    motion_problem,
                    solution_path,
                    is_state_space,
                    _as,
                    _sc,
                    map,
                    n,
                )

                for k, v in paths.items():
                    remapped_paths[state_space_map[k]] = v

                for k, v in durations.items():
                    remapped_durations[state_space_map[k]] = v

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
            # Record reachable configurations on success
            for k in moving_objects:
                reachable_configurations[k] = [start_configs[k], goal_configs[k]]

            # Check temporal validity if action_durations provided
            if action_durations and remapped_durations:
                for k in moving_objects:
                    if k not in remapped_durations:
                        continue
                    wait = remapped_durations[k][0] - (action_starts[k] if action_starts else 0.0)
                    wait = max(0.0, wait)
                    if remapped_durations[k][1] + wait > action_durations[k]:
                        is_temporal_valid = False
                        break

        else:
            for k, _ in moving_objects.items():
                unreachable_configurations[k] = [goal_configs[k]]
                reachable_configurations[k] = []
                collision_obstacles[k] = list(obstacles.keys())

            use_bfs = (
                (not hull_enabled)
                and action_durations is None
                and motion_model in {MotionModels.SE2, MotionModels.REEDSSHEPP}
            )
            if use_bfs:
                (
                    reachable_configurations,
                    unreachable_configurations,
                    collision_obstacles,
                    goal_reached,
                ) = self._compute_exact_reachability_for_failure(
                    moving_objects,
                    motion_model,
                    start_configs,
                    goal_configs,
                    obstacles,
                    problem_objects,
                    topological_refinement,
                    cc,
                )
                if goal_reached:
                    is_valid = True
            elif planner_data:

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
                            alpha = 0 if not is_state_space and motion_model in {MotionModels.REEDSSHEPP, MotionModels.SE2} else 0.1
                            hulls[k] = alphashape.alphashape(np.array(points), alpha)
                            planning_data.convex_hull_time = time.time() - start_time

                    return hulls

                def is_point_in_hull(point, hull):

                    is_point_in_hull = False

                    if motion_model in {MotionModels.REEDSSHEPP, MotionModels.SE2}:
                        is_point_in_hull = hull.contains(Point(*point))
                    elif motion_model == MotionModels.SE3:
                        is_point_in_hull = hull.contains([point])[0]
                    else:
                        raise NotImplementedError  # Unsupported motion model

                    return is_point_in_hull

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
                                obj.configuration.z,
                            )
                        else:
                            raise NotImplementedError  # Unsupported motion model

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
            and not use_bfs
        ):
            collision_obstacles = cc.get_collision_objects()
            for k in goal_reached or []:
                collision_obstacles.pop(k, None)

        if not is_valid:
            print(f"Unreachable configurations: {unreachable_configurations}")
            print(f"Collision obstacles: {collision_obstacles}")

        return (
            is_valid,
            is_temporal_valid,
            remapped_paths,
            remapped_durations,
            reachable_configurations,
            unreachable_configurations,
            collision_obstacles,
            planning_data,
        )

    def _compute_hull_for_failure(
        self,
        planner_data,
        moving_objects,
        motion_model,
        state_space_map,
        motion_problem,
        is_state_space,
        start_configs,
        goal_configs,
        problem_objects,
        topological_refinement,
        planning_data,
        cc,
    ):
        """Inline hull computation shared by check_motion_constraint and
        sequential_check_motion_constraint failure paths."""

        map = self.get_map(problem_objects)

        reachable_configurations = {}
        unreachable_configurations = {}
        collision_obstacles_out = {}
        goal_reached = []

        for k, _ in moving_objects.items():
            unreachable_configurations[k] = [goal_configs[k]]
            reachable_configurations[k] = []
            collision_obstacles_out[k] = []

        if not planner_data:
            return reachable_configurations, unreachable_configurations, collision_obstacles_out, goal_reached

        goal_equivalent_area = {}
        coverage_area = {}
        num_vertices = planner_data.numVertices()
        reachability_list = []
        goal_vertices = set()
        all_others = set()

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

        for i in range(num_vertices):
            reachable_data = ob.PlannerData(motion_problem.getSpaceInformation())
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
                    coverage_area.setdefault(key, [])
                    if sampled_state not in coverage_area[key]:
                        coverage_area[key].append(sampled_state)
            else:
                sampled_state = extract_sampled_state(state)
                coverage_area.setdefault(0, [])
                if sampled_state not in coverage_area[0]:
                    coverage_area[0].append(sampled_state)

        goal_equivalent_vertices = [
            v for v, rd in reachability_list
            if any(gv != v and rd.vertexExists(gv) for gv in goal_vertices)
        ]

        for vertex in goal_equivalent_vertices:
            state = vertex.getState()
            if len(moving_objects) > 1:
                for key in moving_objects:
                    sampled_state = extract_sampled_state(state, key)
                    if sampled_state is None:
                        continue
                    goal_equivalent_area.setdefault(key, [])
                    if sampled_state not in goal_equivalent_area[key]:
                        goal_equivalent_area[key].append(sampled_state)
            else:
                sampled_state = extract_sampled_state(state)
                goal_equivalent_area.setdefault(0, [])
                if sampled_state not in goal_equivalent_area[0]:
                    goal_equivalent_area[0].append(sampled_state)

        start_equivalent_area = {}
        start_equivalent_vertices = all_others.difference(set(goal_equivalent_vertices))
        for vertex in start_equivalent_vertices:
            state = vertex.getState()
            if len(moving_objects) > 1:
                for key in moving_objects:
                    sampled_state = extract_sampled_state(state, key)
                    if sampled_state is None:
                        continue
                    start_equivalent_area.setdefault(key, [])
                    if sampled_state not in start_equivalent_area[key]:
                        start_equivalent_area[key].append(sampled_state)
            else:
                sampled_state = extract_sampled_state(state)
                start_equivalent_area.setdefault(0, [])
                if sampled_state not in start_equivalent_area[0]:
                    start_equivalent_area[0].append(sampled_state)

        import alphashape as _alphashape
        import time as _time

        def compute_hulls(area_dict):
            hulls = {}
            for kk, points in area_dict.items():
                if len(points) > 2:
                    alpha = 0 if not is_state_space and motion_model in {MotionModels.REEDSSHEPP, MotionModels.SE2} else 0.1
                    hulls[kk] = _alphashape.alphashape(np.array(points), alpha)
            return hulls

        area_source = goal_equivalent_area if is_state_space else coverage_area
        hull = compute_hulls(area_source)

        def to_closed_polygon(coords):
            if len(coords) >= 3:
                if coords[0] != coords[-1]:
                    coords = coords + [coords[0]]
                return Polygon(coords)
            return None

        for k, convex_h in hull.items():
            if convex_h is None:
                continue

            if k in start_equivalent_area and k in goal_equivalent_area:
                poly1 = to_closed_polygon(goal_equivalent_area[k])
                poly2 = to_closed_polygon(start_equivalent_area[k])
                if poly1 is not None:
                    if poly2 is not None and poly1.intersects(poly2):
                        goal_reached.append(k)
                    elif poly2 is None:
                        for p in start_equivalent_area[k]:
                            if poly1.contains(Point(*p)):
                                goal_reached.append(k)
                                break

            if topological_refinement not in [
                SupportedTopologicalRefinement.ALL,
                SupportedTopologicalRefinement.UNREACH,
            ]:
                continue

            orig_k = list(moving_objects.keys())[k] if k == 0 and len(moving_objects) == 1 else k
            start_type = start_configs[orig_k].type
            start_name = start_configs[orig_k].name

            for obj in problem_objects:
                if (
                    obj.type != start_type
                    or obj.name == start_name
                    or obj == goal_configs[orig_k]
                ):
                    continue

                if motion_model in {MotionModels.REEDSSHEPP, MotionModels.SE2}:
                    point = (
                        obj.configuration.x / map.resolution,
                        map.image.size[1] - obj.configuration.y / map.resolution,
                    )
                elif motion_model == MotionModels.SE3:
                    point = (obj.configuration.x, obj.configuration.y, obj.configuration.z)
                else:
                    raise NotImplementedError

                if motion_model in {MotionModels.REEDSSHEPP, MotionModels.SE2}:
                    point_in_hull = convex_h.contains(Point(*point))
                else:
                    point_in_hull = convex_h.contains([point])[0]

                if (not point_in_hull and not is_state_space) or (point_in_hull and is_state_space):
                    unreachable_configurations.setdefault(orig_k, [])
                    if obj not in unreachable_configurations[orig_k]:
                        unreachable_configurations[orig_k].append(obj)

        if topological_refinement in [
            SupportedTopologicalRefinement.ALL,
            SupportedTopologicalRefinement.OBS,
        ]:
            collision_obstacles_out = cc.get_collision_objects()
            for k in goal_reached:
                collision_obstacles_out.pop(k, None)

        return reachable_configurations, unreachable_configurations, collision_obstacles_out, goal_reached

    def _compute_exact_reachability_for_failure(
        self,
        moving_objects,
        motion_model,
        start_configs,
        goal_configs,
        obstacles,
        problem_objects,
        topological_refinement,
        cc,
    ):
        """BFS-based exact reachability on the map image (2D only).

        Alternative to _compute_hull_for_failure for SE2/ReedsShepp spaces.
        Computes connected components from start and goal pixels after eroding
        the map by the robot footprint. Returns the same 4-tuple.
        """
        if _cv2 is None:
            raise ImportError(
                "_compute_exact_reachability_for_failure requires cv2 (pip install opencv-python)"
            )

        map = self.get_map(problem_objects)

        reachable_configurations = {}
        unreachable_configurations = {}
        collision_obstacles_out = {}
        goal_reached = []

        for k, obj in moving_objects.items():
            reachable_configurations[k] = [start_configs[k]]
            unreachable_configurations[k] = [goal_configs[k]]
            collision_obstacles_out[k] = list(obstacles.keys())

            # Build C-space image with obstacles painted on
            open_cv_image = np.array(map.image)
            for o, cfg in obstacles.items():
                obstacle_poly = cc.get_polygon_from_config(o, cfg)
                pts = np.array(obstacle_poly.exterior.coords, dtype=np.int32).reshape((-1, 1, 2))
                _cv2.fillPoly(open_cv_image, [pts], (0, 0, 0))

            gray = _cv2.cvtColor(open_cv_image, _cv2.COLOR_BGR2GRAY)
            binary = np.where(gray > 200, 1, 0).astype(np.uint8)

            # Erode by robot diameter to get C-space free cells
            diam = max(1, int(
                max(
                    np.linalg.norm(np.array(p1) - np.array(p2))
                    for p1, p2 in combinations(obj.footprint, 2)
                )
                / map.resolution
            ))
            kernel = _cv2.getStructuringElement(_cv2.MORPH_ELLIPSE, (diam, diam))
            cspace = _cv2.erode(binary, kernel, iterations=1)

            def _bfs(start_xy):
                h, w = cspace.shape
                sx, sy = int(start_xy[0]), int(start_xy[1])
                reachable = np.zeros_like(cspace, dtype=np.uint8)
                if not (0 <= sx < w and 0 <= sy < h) or cspace[sy, sx] == 0:
                    return reachable
                queue = deque([(sx, sy)])
                visited = {(sx, sy)}
                while queue:
                    x, y = queue.popleft()
                    if not (0 <= x < w and 0 <= y < h) or cspace[y, x] == 0:
                        continue
                    reachable[y, x] = 255
                    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                        nb = (x + dx, y + dy)
                        if nb not in visited:
                            visited.add(nb)
                            queue.append(nb)
                return reachable

            def _cfg_to_px(cfg):
                return (
                    cfg.configuration.x / map.resolution,
                    map.image.size[1] - cfg.configuration.y / map.resolution,
                )

            area_from_start = _bfs(_cfg_to_px(start_configs[k]))
            area_from_goal = _bfs(_cfg_to_px(goal_configs[k]))

            def _in_area(cfg, area):
                px, py = _cfg_to_px(cfg)
                xi, yi = int(px), int(py)
                h, w = area.shape
                return 0 <= xi < w and 0 <= yi < h and area[yi, xi] > 0

            # Check goal reachability from start
            if _in_area(goal_configs[k], area_from_start):
                goal_reached.append(k)
                reachable_configurations[k].append(goal_configs[k])
                unreachable_configurations[k] = []
                continue

            if topological_refinement not in [
                SupportedTopologicalRefinement.ALL,
                SupportedTopologicalRefinement.UNREACH,
            ]:
                continue

            start_type = start_configs[k].type
            start_name = start_configs[k].name
            for candidate in problem_objects:
                if (
                    candidate.type != start_type
                    or candidate.name == start_name
                    or candidate == goal_configs[k]
                ):
                    continue
                if _in_area(candidate, area_from_start):
                    reachable_configurations[k].append(candidate)
                if _in_area(candidate, area_from_goal):
                    unreachable_configurations[k].append(candidate)

        return reachable_configurations, unreachable_configurations, collision_obstacles_out, goal_reached

    def sequential_check_motion_constraint(
        self,
        moving_objects: Dict[int, MovableObject],
        start_configs: Dict[int, ConfigurationObject],
        goal_configs: Dict[int, ConfigurationObject],
        obstacles,
        problem_objects,
        *,
        action_starts: Optional[Dict[int, float]] = None,
        action_durations: Optional[Dict[int, float]] = None,
        d_max: Optional[Dict[int, float]] = None,
        safety_distance: Optional[float] = None,
        planning_time: Optional[float] = 1.0,
        interpolate: Optional[bool] = True,
        simplified: Optional[bool] = True,
        distance: Optional[float] = None,
        hull_enabled: Optional[bool] = False,
        motion_planner: Optional[SupportedPlanner] = SupportedPlanner.RRT,
        topological_refinement: Optional[
            SupportedTopologicalRefinement
        ] = SupportedTopologicalRefinement.ALL,
        max_radius_bound: Optional[bool] = False,
    ):
        """Sequential per-object motion planning with trajectory CC.

        Plans for each moving object one at a time in insertion order.
        Each subsequent robot's CC receives all previously solved trajectories
        so it avoids those paths.  Returns the same 8-tuple as
        check_motion_constraint.
        """

        is_valid = True
        is_temporal_valid = True

        map = self.get_map(problem_objects)
        state_space_map = {i: k for i, k in enumerate(moving_objects)}

        reachable_configurations = {}
        unreachable_configurations = {}
        collision_obstacles = {}
        planning_data_all = {}

        remapped_paths = {}
        remapped_durations = {}

        obj_checked = {}
        idx_checked = []

        for idx, obj in moving_objects.items():

            motion_model = obj.motion_model

            # Build trajectories dict: only those that are still active when idx starts
            paths_to_check = {}
            for prev_idx in idx_checked:
                dur = remapped_durations.get(state_space_map[prev_idx])
                if dur and dur[0] + dur[1] <= (action_starts[idx] if action_starts else 0.0):
                    continue
                if state_space_map[prev_idx] in remapped_paths:
                    paths_to_check[state_space_map[prev_idx]] = remapped_paths[state_space_map[prev_idx]]

            obj_checked[obj] = idx
            idx_checked.append(idx)

            if motion_model in {MotionModels.REEDSSHEPP, MotionModels.SE2}:
                cc = CollisionChecker2D(
                    moving_objects={idx: obj},
                    start_configs={idx: start_configs[idx]},
                    delays={idx: action_starts[idx]} if action_starts else None,
                    index_map=state_space_map,
                    d_max=list(d_max.values()) if d_max else None,
                    map=map,
                    movable_obstacles=obstacles,
                    topological_refinement=topological_refinement,
                    max_radius_bound=max_radius_bound,
                    sequential_check=True,
                    trajectories=paths_to_check,
                    safety_distance=safety_distance,
                    all_movable_objects=moving_objects,
                    all_start_configs=start_configs,
                    all_goal_configs=goal_configs,
                    action_timings=(
                        {k: (action_starts[k], action_durations[k]) for k in action_starts}
                        if action_durations and action_starts
                        else None
                    ),
                )
            elif motion_model == MotionModels.SE3:
                cc = CollisionChecker3D(
                    moving_objects={idx: obj},
                    start_configs={idx: start_configs[idx]},
                    delays={idx: action_starts[idx]} if action_starts else None,
                    index_map=state_space_map,
                    d_max=list(d_max.values()) if d_max else None,
                    map=map,
                    movable_obstacles=obstacles,
                    topological_refinement=topological_refinement,
                    max_radius_bound=max_radius_bound,
                    sequential_check=True,
                    trajectories=paths_to_check,
                    safety_distance=safety_distance,
                    all_movable_objects=moving_objects,
                    all_start_configs=start_configs,
                    all_goal_configs=goal_configs,
                    action_timings=(
                        {k: (action_starts[k], action_durations[k]) for k in action_starts}
                        if action_durations and action_starts
                        else None
                    ),
                )
            else:
                raise Exception(f"Unsupported motion model: {motion_model}")

            motion_problem = self.set_problem(
                map,
                state_space_map,
                {0: obj},
                {0: start_configs[idx]},
                {0: goal_configs[idx]},
                motion_planner,
                cc,
                d_max={0: d_max[idx]} if d_max else None,
                distance=distance,
                action_starts={0: 0},
                sequential_check=True,
            )

            if simplified:
                print("simplifySolution() not available in Control Space. Giving back original solution.")
                simplified = False

            solution_path = None
            abort_current = False
            single_time = planning_time

            for _attempt in range(5):
                if solution_path or abort_current:
                    break

                print(f"{obj} (idx {idx}) to start at {action_starts.get(idx, 0.0)} (d_max {d_max.get(idx) if d_max else None})")

                solution_path, planner_data, obj_planning_data = self.get_solution(
                    motion_problem,
                    planning_time=single_time,
                    interpolate=interpolate,
                    simplified=simplified,
                )
                planning_data_all[idx] = obj_planning_data

                if solution_path:
                    reachable_configurations[idx] = [start_configs[idx]]

                    is_time_space = d_max is not None and d_max.get(idx) is not None

                    if is_time_space and motion_model in {MotionModels.REEDSSHEPP, MotionModels.SE2}:
                        paths, durations = self.get_control_path(
                            motion_problem,
                            solution_path,
                            True,
                            {0: action_starts[idx] if action_starts else 0.0},
                            {0: start_configs[idx]},
                            map,
                            1,
                            sequential_check=True,
                        )
                        remapped_paths[state_space_map[idx]] = paths[0]
                        remapped_durations[state_space_map[idx]] = durations[0]

                        wait = durations[0][0] - (action_starts[idx] if action_starts else 0.0)
                        wait = max(0.0, wait)
                        if action_durations and durations[0][1] + wait > action_durations[idx]:
                            is_temporal_valid = False
                            abort_current = True

                    elif motion_model in {MotionModels.REEDSSHEPP, MotionModels.SE2}:
                        remapped_paths.setdefault(state_space_map[idx], [])
                        for state in solution_path.getStates():
                            remapped_paths[state_space_map[idx]].append(
                                (state.getX(), state.getY(), state.getYaw())
                            )
                    else:
                        raise NotImplementedError()

                single_time *= 2

            if abort_current:
                return (
                    is_valid,
                    is_temporal_valid,
                    remapped_paths,
                    remapped_durations,
                    reachable_configurations,
                    unreachable_configurations,
                    collision_obstacles,
                    planning_data_all,
                )

            if not solution_path:
                is_valid = False
                is_temporal_valid = False

                is_time_space = d_max is not None and d_max.get(idx) is not None
                (
                    reachable_configurations,
                    unreachable_configurations,
                    collision_obstacles,
                    _goal_reached,
                ) = self._compute_hull_for_failure(
                    planner_data,
                    {0: obj},
                    motion_model,
                    state_space_map,
                    motion_problem,
                    is_time_space,
                    {0: start_configs[idx]},
                    {0: goal_configs[idx]},
                    problem_objects,
                    topological_refinement,
                    obj_planning_data,
                    cc,
                )

            if topological_refinement in [
                SupportedTopologicalRefinement.ALL,
                SupportedTopologicalRefinement.OBS,
            ] and not solution_path:
                raw_obs = cc.get_collision_objects()
                collision_obstacles[idx] = raw_obs.get(idx, [])

            if solution_path:
                reachable_configurations[idx].append(goal_configs[idx])

        print("remapped_durations:", remapped_durations)
        return (
            is_valid,
            is_temporal_valid,
            remapped_paths,
            remapped_durations,
            reachable_configurations,
            unreachable_configurations,
            collision_obstacles,
            planning_data_all,
        )
