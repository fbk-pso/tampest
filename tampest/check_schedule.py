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

import unified_planning.model.motion as tamp
from unified_planning.model.motion import (
    MotionActivity,
    MotionConstraint,
    SchedulingMotionProblem,
    ActivityWaypoints,
    MovableObject,
    ConfigurationObject,
)
from unified_planning.plans import Schedule
from unified_planning.model.scheduling import Activity
from unified_planning.model import ExpressionManager, FNode, TimePointInterval, Timing
from tampest.motion.motion_planner import MotionPlanner
from typing import List, Tuple, Dict, Set, Union, Optional
import math
import itertools
from functools import partial
import sys

from tampest.motion.motion_planning_data import (
    SupportedPlanner,
    SupportedTopologicalRefinement,
)


def activity_start_time(schedule: Schedule, activity: Activity) -> int:
    return int(schedule.get(activity.start).constant_value())


def activity_end_time(schedule: Schedule, activity: Activity) -> int:
    return int(schedule.get(activity.end).constant_value())


def get_overlapping_motion_activities(
    activities: List[MotionActivity], schedule: Schedule
) -> List[List[MotionActivity]]:

    def get_next_group(idx: int, activities: List[MotionActivity]) -> int:
        group_end = activity_end_time(schedule, activities[idx])
        next_idx = idx + 1
        while next_idx < len(activities):
            if activity_start_time(schedule, activities[next_idx]) >= group_end:
                break
            group_end = max(
                group_end, activity_end_time(schedule, activities[next_idx])
            )
            next_idx += 1
        return next_idx

    sorted_motion_activities = sorted(
        activities, key=partial(activity_start_time, schedule)
    )
    groups = []
    start_idx = 0
    end_idx = 0
    while end_idx < len(sorted_motion_activities):
        overlapping_activities = []
        end_idx = get_next_group(start_idx, sorted_motion_activities)
        for idx in range(start_idx, end_idx):
            activity = sorted_motion_activities[idx]
            overlapping_activities.append(activity)

        groups.append(overlapping_activities)
        start_idx = end_idx

    return groups


def get_movable_object_conf(
    problem: SchedulingMotionProblem,
    schedule: Schedule,
    activity: MotionActivity,
    movable_object: MovableObject,
) -> tamp.ConfigurationObject:
    motion_activities = []
    for act in schedule.activities:
        if (
            isinstance(act, MotionActivity)
            and movable_object in [mo.object() for mo in act.motion_effects]
            and activity_end_time(schedule, act)
            <= activity_start_time(schedule, activity)
        ):
            motion_activities.append(act)
    sorted_motion_activities = sorted(
        motion_activities, key=partial(activity_end_time, schedule)
    )

    if len(sorted_motion_activities) > 0:
        for mo, conf in sorted_motion_activities[-1].motion_effects.items():
            if mo.object() == movable_object:
                return conf.object()

    for mo, conf in problem.initial_configuration:
        if mo.object() == movable_object:
            return conf.object()

    raise Exception


def get_constraint_params(
    problem: SchedulingMotionProblem,
    schedule: Schedule,
    overlapping_activities: List[MotionActivity],
    activity: MotionActivity,
    motion_constraint: ActivityWaypoints,
    use_dynamic_obstacles: bool = True,
) -> Tuple[
    tamp.MovableObject,
    tamp.ConfigurationObject,
    tamp.ConfigurationObject,
    Dict[tamp.MovableObject, tamp.ConfigurationObject],
]:
    movable = motion_constraint.movable
    starting = motion_constraint.starting
    if len(motion_constraint.waypoints) != 1:
        raise NotImplementedError
    waypoint = motion_constraint.waypoints[0]
    static_obstacles_conf: Dict[tamp.MovableObject, tamp.ConfigurationObject] = {}

    if motion_constraint.static_obstacles is not None:
        for movable_object in motion_constraint.static_obstacles:
            if movable.object() != movable_object:
                static_obstacles_conf[movable_object] = get_movable_object_conf(
                    problem, schedule, activity, movable_object
                )
                assert isinstance(
                    static_obstacles_conf[movable_object], tamp.ConfigurationObject
                )
            else:
                raise Exception

    group_movable_objects = set()
    for act in overlapping_activities:
        for mo in act.motion_effects:
            group_movable_objects.add(mo.object())
        for mc in act.motion_constraints:
            assert isinstance(mc, ActivityWaypoints)
            group_movable_objects.add(mc.movable.object())

    if (
        use_dynamic_obstacles
        and motion_constraint.dynamic_obstacles_at_start is not None
    ):
        for movable_object in motion_constraint.dynamic_obstacles_at_start:
            if movable.object() != movable_object:
                if movable_object in group_movable_objects:
                    # another activity is moving the object
                    continue
                static_obstacles_conf[movable_object] = get_movable_object_conf(
                    problem, schedule, activity, movable_object
                )
                assert isinstance(
                    static_obstacles_conf[movable_object], tamp.ConfigurationObject
                )
            else:
                raise Exception

    return (
        movable.object(),
        starting.object(),
        waypoint.object(),
        static_obstacles_conf,
    )


def get_obstacles_with_configuration(
    problem: SchedulingMotionProblem,
    schedule: Schedule,
    activity: MotionActivity,
    activity_group: List[MotionActivity],
    use_dynamic_obstacles: bool,
) -> Dict[tamp.MovableObject, Tuple[Optional[FNode], ConfigurationObject]]:
    group_movable_objects = []
    for act in activity_group:
        for motion_constraint in act.motion_constraints:
            group_movable_objects.append(motion_constraint.movable.object())

    assert len(activity.motion_constraints) == 1
    motion_constraint = activity.motion_constraints[0]
    assert isinstance(motion_constraint, tamp.ActivityWaypoints)

    obstacles = {}
    for static_dynamic_obstacles in [
        motion_constraint.static_obstacles,
        motion_constraint.dynamic_obstacles_at_start if use_dynamic_obstacles else [],
    ]:
        if static_dynamic_obstacles is None:
            continue

        for obstacle in static_dynamic_obstacles:
            if obstacle in group_movable_objects:
                continue

            obstacle_conf = get_movable_object_conf(
                problem, schedule, activity, obstacle
            )
            if isinstance(static_dynamic_obstacles, Dict):
                obstacles[obstacle] = (
                    static_dynamic_obstacles[obstacle],
                    obstacle_conf,
                )
            else:
                obstacles[obstacle] = (None, obstacle_conf)

    return obstacles


def get_activity_max_duration(activity: Activity) -> int:
    assert activity.duration.upper.is_constant()
    return activity.duration.upper.constant_value()


def check_schedule(
    problem: SchedulingMotionProblem,
    schedule: Schedule,
    motion_planning_time: float,
    interpolate: bool,
    simplified: bool,
    distance: float,
    motion_planner: SupportedPlanner,
    topological_refinement: SupportedTopologicalRefinement,
    max_radius_bound: bool,
    topological_cache: Set[Tuple],
    topological_temporal_cache: Dict[Tuple, List[Tuple[float, ...]]],
    topological_refinements_cache: Set[Tuple],
    motion_paths: Dict[
        Tuple[MotionActivity, MotionConstraint], List[Tuple[float, ...]]
    ],
    activities_conditions: Dict[
        Tuple[MotionActivity, MovableObject, ConfigurationObject], Activity
    ] = {},
    enable_reachable_generalization: bool = True,
    enable_movable_generalization: bool = True,
    force_sequential_MP_check: bool = False,
    safety_distance: Optional[float] = None,
) -> Tuple[bool, List[FNode], bool]:

    assert isinstance(problem, SchedulingMotionProblem)
    mp = MotionPlanner()

    activities = [problem.get_activity(a.name) for a in schedule.activities]
    motion_activities = list(
        filter(
            lambda activity: isinstance(activity, MotionActivity)
            and len(activity.motion_constraints) > 0,
            activities,
        )
    )
    # motion_activities.sort(key=lambda act: schedule.get(act.start).constant_value())

    for activity in motion_activities:
        is_valid, refinements = topological_check_single_activity(
            activity,
            problem,
            schedule,
            mp,
            # motion_planning_time=motion_planning_time / 2.0,
            motion_planning_time=(
                motion_planning_time * 10.0
                if force_sequential_MP_check == True
                else motion_planning_time / 2.0
            ),
            interpolate=interpolate,
            simplified=simplified,
            topological_refinement=topological_refinement,
            cache=topological_cache,
            refinements_cache=topological_refinements_cache,
            activities_conditions=activities_conditions,
            enable_reachable_generalization=enable_reachable_generalization,
            enable_movable_generalization=enable_movable_generalization,
        )
        if not is_valid:
            return False, refinements, False

    for activity in motion_activities:
        is_valid, refinements, is_temporal_refinement = topological_temporal_check(
            [activity],
            problem,
            schedule,
            mp,
            True,
            motion_planning_time,
            interpolate,
            simplified,
            distance,
            motion_planner,
            topological_refinement,
            max_radius_bound,
            motion_paths,
            topological_temporal_cache,
            activities_conditions,
            enable_reachable_generalization=enable_reachable_generalization,
            enable_movable_generalization=enable_movable_generalization,
            force_sequential_MP_check=force_sequential_MP_check,
            safety_distance=safety_distance,
        )
        if not is_valid:
            return False, refinements, is_temporal_refinement

    # group overlapping motion activities
    overlapping_activities = get_overlapping_motion_activities(
        motion_activities, schedule
    )
    # check each group of activities
    for group in overlapping_activities:
        is_valid, refinements, is_temporal_refinement = topological_temporal_check(
            group,
            problem,
            schedule,
            mp,
            False,
            # motion_planning_time * len(group),
            (
                motion_planning_time * 2.0 + 5.0
                if force_sequential_MP_check == True
                else motion_planning_time * len(group)
            ),
            interpolate,
            simplified,
            distance,
            motion_planner,
            topological_refinement,
            max_radius_bound,
            motion_paths,
            topological_temporal_cache,
            activities_conditions,
            enable_reachable_generalization=enable_reachable_generalization,
            enable_movable_generalization=enable_movable_generalization,
            force_sequential_MP_check=force_sequential_MP_check,
            safety_distance=safety_distance,
        )
        if not is_valid:
            return False, refinements, is_temporal_refinement

    return True, [], False


def filter_motion_paths(
    schedule: Schedule,
    motion_paths: Dict[
        Tuple[MotionActivity, MotionConstraint], List[Tuple[float, ...]]
    ],
) -> Dict[Tuple[MotionActivity, MotionConstraint], List[Tuple[float, ...]]]:
    motion_activities = set(
        filter(
            lambda activity: isinstance(activity, MotionActivity)
            and len(activity.motion_constraints) > 0,
            schedule.activities,
        )
    )

    # Filter motion_paths to only include activities present in motion_activities
    return {
        (activity, motion_constraint): path
        for (activity, motion_constraint), path in motion_paths.items()
        if activity in motion_activities
    }


def topological_check_single_activity(
    activity: MotionActivity,
    problem: SchedulingMotionProblem,
    schedule: Schedule,
    mp: MotionPlanner,
    motion_planning_time: float,
    interpolate: bool,
    simplified: bool,
    topological_refinement: SupportedTopologicalRefinement,
    cache: Set[Tuple],
    refinements_cache: Set[Tuple],
    activities_conditions: Dict[
        Tuple[MotionActivity, MovableObject, ConfigurationObject], Activity
    ] = {},
    enable_reachable_generalization: bool = True,
    enable_movable_generalization: bool = True,
) -> Tuple[bool, List[FNode]]:
    for motion_constraint in activity.motion_constraints:
        assert isinstance(motion_constraint, ActivityWaypoints)
        movable, starting, waypoint, obstacles = get_constraint_params(
            problem,
            schedule,
            [activity],
            activity,
            motion_constraint,
            use_dynamic_obstacles=False,
        )
        sorted_obstacles = tuple(sorted(obstacles.items(), key=lambda o: o[0].name))

        print(f"Performing topological check for activity '{activity.name}'")
        if activity_key(activity, sorted_obstacles) in cache:
            print("cache hit: VALID")
            continue

        # topological check
        (
            is_valid,
            _,
            _,
            _,
            reachable_configurations,
            unreachable_goals,
            static_obstacles,
            _,
        ) = mp.check_motion_constraint(
            {0: movable},
            {0: starting},
            {0: waypoint},
            obstacles,
            problem.all_objects,
            planning_time=motion_planning_time,
            interpolate=interpolate,
            simplified=simplified,
            distance=None,
            motion_planner=SupportedPlanner.RRT,
            topological_refinement=topological_refinement,
            max_radius_bound=False,
        )

        for k in reachable_configurations:
            if k in unreachable_goals:
                assert set(reachable_configurations).isdisjoint(unreachable_goals[k])

        # TODO: unreachable_goals should be computed by check_motion_constraint
        unreachable_goals[0] = list(
            filter(
                lambda l: l not in reachable_configurations[0],
                problem.objects(motion_constraint.starting.type),
            )
        )

        if not enable_reachable_generalization:
            reachable_configurations = {0: [starting]}

        if enable_movable_generalization:
            movable_objects = equivalent_movable_objects(problem, movable)
        else:
            movable_objects = [movable]

        for i in range(len(reachable_configurations[0])):
            for j in range(len(reachable_configurations[0])):
                if i == j:
                    continue

                for movable in movable_objects:
                    for act in filter_motion_activities(
                        problem,
                        movable,
                        reachable_configurations[0][i],
                        reachable_configurations[0][j],
                    ):
                        cache.add(activity_key(act, sorted_obstacles))

        if not is_valid:
            print("Add topological refinement")
            configurations = [
                (conf1, conf2)
                for conf1 in reachable_configurations[0]
                for conf2 in unreachable_goals[0]
            ]
            if enable_reachable_generalization:
                configurations += [(goal, start) for start, goal in configurations]
            refinements = []
            for movable in movable_objects:
                for start, goal in configurations:
                    for act in filter_motion_activities(problem, movable, start, goal):
                        motion_constraint = act.motion_constraints[0]
                        obstacles_configuration_fluent = None
                        if isinstance(motion_constraint.static_obstacles, dict):
                            obstacles_configuration_fluent = (
                                motion_constraint.static_obstacles
                            )

                        if activity_key(act, sorted_obstacles) not in refinements_cache:
                            refinements.append(
                                get_topological_refinement(
                                    problem,
                                    schedule,
                                    overlapping_activities=[(act, activity)],
                                    unreachable_locations={
                                        (act, motion_constraint): [goal]
                                    },
                                    blocking_obstacles={
                                        (act, motion_constraint): (
                                            static_obstacles[0]
                                            if len(static_obstacles) > 0
                                            else []
                                        )
                                    },
                                    is_single_activity_layer=True,
                                    obstacles_configuration_fluent=obstacles_configuration_fluent,
                                    activities_conditions=activities_conditions,
                                )
                            )
                            refinements_cache.add(activity_key(act, sorted_obstacles))
            assert len(refinements) > 0
            return False, refinements

        else:
            print("VALID")

    return True, []


def topological_temporal_check(
    activity_group: List[MotionActivity],
    problem: SchedulingMotionProblem,
    schedule: Schedule,
    mp: MotionPlanner,
    is_single_activity_layer: bool,
    motion_planning_time: float,
    interpolate: bool,
    simplified: bool,
    distance: float,
    motion_planner: SupportedPlanner,
    topological_refinement: SupportedTopologicalRefinement,
    max_radius_bound: bool,
    motion_paths: Dict[
        Tuple[MotionActivity, MotionConstraint], List[Tuple[float, ...]]
    ],
    cache: Dict[Tuple, List[Tuple[float, ...]]],
    activities_conditions: Dict[
        Tuple[MotionActivity, MovableObject, ConfigurationObject], Activity
    ],
    enable_reachable_generalization: bool = True,
    enable_movable_generalization: bool = True,
    force_sequential_MP_check: bool = False,
    safety_distance: Optional[float] = None,
) -> Tuple[bool, List[FNode], bool]:
    movable_objects = {}
    for activity in activity_group:
        if len(activity.motion_constraints) > 1:
            raise NotImplementedError("Multiple motion constraints are not supported")
        if len(activity.motion_effects) > 1:
            raise NotImplementedError("Multiple motion effects are not supported")
        movable = list(activity.motion_effects.keys())[0].object()

        if movable in movable_objects:
            current_start_time = activity_start_time(schedule, activity)
            last_end_time = activity_end_time(schedule, movable_objects[movable])
            if (
                current_start_time < last_end_time
            ):  # assumption: activity in group ordered by start time
                raise Exception("Overlapping activities moving the same object")

            mc = activity.motion_constraints[0]
            assert isinstance(mc, ActivityWaypoints)

            current_start_pose = mc.starting
            last_end_pose = movable_objects[movable].motion_constraints[0].waypoints[0]
            if current_start_pose != last_end_pose:
                raise Exception("Discontinuous motion of the same object")

        movable_objects[movable] = activity

    activity_start_duration = {}
    constraints_map = {}
    motion_constraints = {}
    activity_keys = []
    start_zero = activity_start_time(schedule, activity_group[0])
    for activity in activity_group:
        activity_start = activity_start_time(schedule, activity)
        activity_end = activity_end_time(schedule, activity)
        activity_duration = activity_end - activity_start
        activity_start_duration[activity] = (activity_start, activity_duration)
        d_max = get_activity_max_duration(activity)

        obstacle_configurations: List[Union[int, str]] = []
        for motion_constraint in activity.motion_constraints:
            assert isinstance(motion_constraint, ActivityWaypoints)
            movable, starting, waypoint, obstacles = get_constraint_params(
                problem,
                schedule,
                activity_group,
                activity,
                motion_constraint,
                use_dynamic_obstacles=not is_single_activity_layer,
            )
            obstacle_configurations.append(len(obstacles))
            for o in sorted(obstacles.keys(), key=lambda o: o.name):
                obstacle_configurations += [o.name, obstacles[o].name]

            constraints_map[(activity, motion_constraint)] = len(constraints_map)
            motion_constraints[(activity, motion_constraint)] = (
                activity_start - start_zero,
                activity_duration,
                d_max,
                movable,
                starting,
                waypoint,
                obstacles,
            )

        activity_keys.append(
            (
                activity.name,
                activity_start - start_zero,
                activity_duration,
                tuple(obstacle_configurations),
            )
        )

    if len(motion_constraints) == 0:
        # no motion constraints
        return True, [], False

    print(
        f"Performing topological-temporal check for activities",
        [a.name for a in activity_group],
    )

    group_key = tuple(activity_keys)
    if group_key in cache:
        print("cache hit: VALID")
        motion_paths.update(cache[group_key])
        return True, [], False

    (
        is_valid,
        is_temporal_valid,
        paths,
        current_durations,
        unreachable_goals,
        static_obstacles,
        mc_planning_data,
    ) = mp.check_motion_constraints(
        motion_constraints,
        constraints_map,
        problem.all_objects,
        planning_time=motion_planning_time,
        interpolate=interpolate,
        simplified=simplified,
        distance=distance,
        motion_planner=motion_planner,
        topological_refinement=topological_refinement,
        max_radius_bound=max_radius_bound,
        hull_enabled=False,
        force_sequential_MP_check=force_sequential_MP_check,
        safety_distance=safety_distance,
    )

    # if is_valid:
    #     # check if the overlapping activities are temporally valid
    #     for activity, (
    #         activity_start,
    #         activity_duration,
    #     ) in activity_start_duration.items():
    #         if activity not in current_durations:
    #             continue

    #         wait = current_durations[activity][0] - (
    #             float(activity_start) - float(start_zero)
    #         )
    #         assert wait > -0.0000001
    #         wait = max(0, wait)
    #         if current_durations[activity][1] + wait > activity_duration:
    #             # not temporally valid
    #             is_valid = False
    #             break

    if is_valid:
        if is_temporal_valid:
            cache[group_key] = paths
            if is_single_activity_layer:
                movable, starting, waypoint, _ = get_constraint_params(
                    problem,
                    schedule,
                    activity_group,
                    activity_group[0],
                    activity_group[0].motion_constraints[0],
                    use_dynamic_obstacles=False,
                )
                if enable_reachable_generalization:
                    configurations = [(starting, waypoint), (waypoint, starting)]
                else:
                    configurations = [(starting, waypoint)]
                if enable_movable_generalization:
                    movable_objects = equivalent_movable_objects(problem, movable)
                else:
                    movable_objects = [movable]
                assert len(paths) == 1
                for movable in movable_objects:
                    for i, (start, goal) in enumerate(configurations):
                        if i == 0:
                            path = list(paths.values())[0]
                        else:
                            # use reversed motion path for the (waypoint, starting) direction
                            path = list(paths.values())[0][::-1]

                        for act in filter_motion_activities(
                            problem, movable, start, goal
                        ):
                            key = ((act.name,) + group_key[0][1:],)
                            if not key in cache:
                                cache[key] = {(act, act.motion_constraints[0]): path}

            motion_paths.update(paths)
            print("VALID")
            return True, [], False
        else:

            assert current_durations
            print(
                "Computed durations",
                {a.name: current_durations[a] for a in current_durations},
            )
            print("Add temporal refinement")
            # not temporally valid (at least one activity has duration greater than the expected)
            if is_single_activity_layer:
                refinements = []
                movable, starting, waypoint, _ = get_constraint_params(
                    problem,
                    schedule,
                    activity_group,
                    activity_group[0],
                    activity_group[0].motion_constraints[0],
                    use_dynamic_obstacles=False,
                )
                if enable_reachable_generalization:
                    configurations = [(starting, waypoint), (waypoint, starting)]
                else:
                    configurations = [(starting, waypoint)]
                if enable_movable_generalization:
                    movable_objects = equivalent_movable_objects(problem, movable)
                else:
                    movable_objects = [movable]
                for movable in movable_objects:
                    for i, (start, goal) in enumerate(configurations):
                        if i == 0:
                            path = list(paths.values())[0]
                        else:
                            # use reversed motion path for the (waypoint, starting) direction
                            path = list(paths.values())[0][::-1]

                        for act in filter_motion_activities(
                            problem, movable, start, goal
                        ):
                            refinements.append(
                                get_temporal_refinement(
                                    problem,
                                    schedule,
                                    overlapping_activities=[(act, activity_group[0])],
                                    motion_planner_durations=current_durations,
                                    is_single_activity_layer=True,
                                    activities_conditions=activities_conditions,
                                )
                            )
                            key = (
                                (
                                    act.name,
                                    0,
                                    math.ceil(current_durations[activity_group[0]][1]),
                                    group_key[0][3],
                                ),
                            )
                            if not key in cache:
                                cache[key] = {(act, act.motion_constraints[0]): path}
                return False, refinements, True
            else:
                return (
                    False,
                    [
                        get_temporal_refinement(
                            problem,
                            schedule,
                            overlapping_activities=activity_group,
                            motion_planner_durations=current_durations,
                            is_single_activity_layer=False,
                            activities_conditions=activities_conditions,
                        )
                    ],
                    True,
                )

    else:
        # unreachable goals
        print("Add topological refinement")
        obstacles_configuration_fluent = None
        if all(
            isinstance(motion_constraint.static_obstacles, dict)
            and isinstance(motion_constraint.dynamic_obstacles_at_start, dict)
            for _, motion_constraint in motion_constraints
        ):
            obstacles_configuration_fluent = {}
            for _, motion_constraint in motion_constraints:
                obstacles_configuration_fluent.update(
                    motion_constraint.static_obstacles
                )
                obstacles_configuration_fluent.update(
                    motion_constraint.dynamic_obstacles_at_start
                )

        return (
            False,
            [
                get_topological_refinement(
                    problem,
                    schedule,
                    overlapping_activities=activity_group,
                    unreachable_locations=unreachable_goals,
                    blocking_obstacles=static_obstacles,
                    is_single_activity_layer=is_single_activity_layer,
                    obstacles_configuration_fluent=obstacles_configuration_fluent,
                    activities_conditions=activities_conditions,
                )
            ],
            False,
        )


def activity_key(
    activity: MotionActivity,
    obstacles_configuration: Tuple[Tuple[MovableObject, ConfigurationObject], ...],
) -> Tuple:
    key = (activity.name, obstacles_configuration)
    return key


def filter_motion_activities(
    problem: SchedulingMotionProblem,
    movable: MovableObject,
    starting: ConfigurationObject,
    goal: ConfigurationObject,
) -> List[MotionActivity]:
    activities = []
    for activity in problem.motion_activities:
        if len(activity.motion_constraints) == 0:
            continue

        motion_constraint = activity.motion_constraints[0]
        assert isinstance(motion_constraint, ActivityWaypoints)
        if (
            movable == motion_constraint.movable.object()
            and starting == motion_constraint.starting.object()
            and goal == motion_constraint.waypoints[0].object()
        ):
            activities.append(activity)

    return activities


def equivalent_movable_objects(
    problem: SchedulingMotionProblem, movable: MovableObject
) -> List[MovableObject]:
    return list(problem.objects(movable.type))


def no_overlap(
    activity1: MotionActivity, activity2: MotionActivity, em: ExpressionManager
) -> FNode:
    return em.Or(
        em.LE(activity1.end, activity2.start), em.GE(activity1.start, activity2.end)
    )


def overlap(
    activity1: MotionActivity, activity2: MotionActivity, em: ExpressionManager
) -> FNode:
    # return em.Not(no_overlap(activity1, activity2, em))
    return em.And(
        em.LT(activity1.start, activity2.end), em.GT(activity1.end, activity2.start)
    )


def get_topological_refinement(
    problem: SchedulingMotionProblem,
    schedule: Schedule,
    overlapping_activities: Union[
        List[MotionActivity], List[Tuple[MotionActivity, MotionActivity]]
    ],
    unreachable_locations: Dict[
        Tuple[MotionActivity, MotionConstraint], List[tamp.ConfigurationObject]
    ],
    blocking_obstacles: Dict[
        Tuple[MotionActivity, MotionConstraint], List[tamp.MovableObject]
    ],
    is_single_activity_layer: bool,
    obstacles_configuration_fluent: Optional[Dict[tamp.MovableObject, FNode]] = None,
    activities_conditions: Dict[
        Tuple[MotionActivity, MovableObject, ConfigurationObject], Activity
    ] = {},
) -> FNode:
    em: ExpressionManager = problem.environment.expression_manager
    if isinstance(overlapping_activities[0], tuple):
        assert is_single_activity_layer
        scheduled_activities = [act[1] for act in overlapping_activities]
        overlapping_activities = [act[0] for act in overlapping_activities]
    else:
        scheduled_activities = overlapping_activities
    cc0 = []
    for eq_group in equivalent_groups(
        problem, overlapping_activities, unreachable_locations
    ):
        premise = refinement_premise(problem, eq_group, is_single_activity_layer)
        cc1 = []
        for i, activity_a in enumerate(eq_group):
            act_constr_key = (
                overlapping_activities[i],
                overlapping_activities[i].motion_constraints[0],
            )
            if act_constr_key not in blocking_obstacles:
                # overlapping_activities[i] reached the goal
                continue

            cc2 = []
            for obstacle in blocking_obstacles[act_constr_key]:
                obstacle_conf = get_movable_object_conf(
                    problem, schedule, scheduled_activities[i], obstacle
                )
                conf_fluent = (
                    None
                    if obstacles_configuration_fluent is None
                    else obstacles_configuration_fluent[obstacle]
                )
                cc2.append(
                    obstacle_changes_configuration_constraint(
                        problem,
                        activity_a,
                        obstacle,
                        obstacle_conf,
                        conf_fluent,
                        activities_conditions,
                    )
                )

            if len(cc2) > 0:
                cc1.append(em.Or(cc2))

        # print("premise", premise)
        if len(cc1) > 0:
            learned_constraint = em.Or(cc1)
            cc0.append(em.Implies(premise, learned_constraint))
            # print("learned", learned_constraint)
        else:
            cc0.append(em.Not(premise))

    return em.And(cc0)


def refinement_premise(
    problem: SchedulingMotionProblem,
    activity_group: List[MotionActivity],
    is_single_activity_layer: bool,
) -> FNode:
    em: ExpressionManager = problem.environment.expression_manager
    cc1 = [em.And(a.present for a in activity_group)]
    if not is_single_activity_layer:
        movable_activities_map = {}
        for activity in activity_group:
            motion_constraint: ActivityWaypoints = activity.motion_constraints[0]
            movable = motion_constraint.movable
            if movable not in movable_activities_map:
                movable_activities_map[movable] = []
            movable_activities_map[movable].append(activity)
        for activities in movable_activities_map.values():
            if len(activities) > 1:
                for activity in activities[1:]:
                    cc1.append(em.LE(activities[0].end, activity.start))

        for activity_r in rival_activities(problem, activity_group):
            before = []
            after = []
            for activity_a in activity_group:
                before.append(em.LE(activity_r.end, activity_a.start))
                after.append(em.GE(activity_r.start, activity_a.end))
            cc1.append(
                em.Implies(activity_r.present, em.Or(em.And(before), em.And(after)))
            )
    premise = em.And(cc1)
    return premise


def obstacle_changes_configuration_constraint(
    problem: SchedulingMotionProblem,
    activity: MotionActivity,
    obstacle: MovableObject,
    obstacle_configuration: ConfigurationObject,
    obstacle_configuration_fluent: Optional[FNode],
    activities_conditions: Dict[
        Tuple[MotionActivity, MovableObject, ConfigurationObject], Activity
    ] = {},
) -> FNode:
    em: ExpressionManager = problem.environment.expression_manager
    if obstacle_configuration_fluent is not None:
        activity_condition_key = (activity, obstacle, obstacle_configuration)
        if activity_condition_key not in activities_conditions:
            activity_condition = problem.add_activity(
                f"{activity.name}_condition{len(activities_conditions)}",
                optional=True,
            )
            # TODO: avoid using protected method _set_duration_constraint
            activity_condition._set_duration_constraint(activity.duration)
            problem.add_constraint(em.Equals(activity_condition.start, activity.start))
            problem.add_constraint(em.Equals(activity_condition.end, activity.end))

            activity_condition.add_condition(
                TimePointInterval(Timing(0, activity_condition.start)),
                em.Not(
                    em.Equals(obstacle_configuration_fluent, obstacle_configuration)
                ),
            )
            activity_condition.add_constraint(activity.present)

            # print(activity_condition)
            activities_conditions[activity_condition_key] = activity_condition

        activity_condition = activities_conditions[activity_condition_key]
        return activity_condition.present

    else:
        del_activities = deleter_activities(problem, obstacle, obstacle_configuration)
        cc3 = []
        for activity_h in helper_activities(problem, obstacle, obstacle_configuration):
            cc4 = []
            for activity_x in del_activities:
                cc4.append(
                    em.Implies(
                        activity_x.present,
                        em.Or(
                            em.LE(activity_x.end, activity_h.start),
                            em.GE(activity_x.start, activity.end),
                        ),
                    )
                )
            c4 = em.And(cc4) if len(cc4) > 0 else True

            cc3.append(
                em.And(
                    filter(
                        lambda e: not isinstance(e, bool),
                        [
                            activity_h.present,
                            em.LE(activity_h.end, activity.start),
                            c4,
                        ],
                    )
                )
            )

        if (
            get_movable_initial_configuration(problem, obstacle)
            != obstacle_configuration
            and len(del_activities) > 0
        ):
            cc3.append(
                em.And(
                    em.Implies(
                        activity_x.present, em.GE(activity_x.start, activity.end)
                    )
                    for activity_x in del_activities
                )
            )
        return em.Or(cc3)


def get_movable_initial_configuration(
    problem: SchedulingMotionProblem, movable: MovableObject
) -> ConfigurationObject:
    for movable_exp, config_exp in problem.initial_configuration:
        if movable_exp.object() == movable:
            return config_exp.object()
    raise AssertionError(f"Movable {movable.name} has not an initial configuration.")


def first_activity_condition(
    activity: MotionActivity, activities: List[MotionActivity], em: ExpressionManager
) -> FNode:
    cc = []
    for act in activities:
        if act.name == activity.name:
            continue
        cc.append(em.LE(activity.start, act.start))

    return em.And(cc)


def delay(
    activity: MotionActivity, group: List[MotionActivity], schedule: Schedule
) -> int:
    """Calculates the delay of the activity with respect to the first activity in the group."""
    first_activity = min(group, key=lambda a: activity_start_time(schedule, a))
    return activity_start_time(schedule, activity) - activity_start_time(
        schedule, first_activity
    )


def duration(activity: MotionActivity, schedule: Schedule) -> int:
    """Calculates the duration of the activity."""
    return activity_end_time(schedule, activity) - activity_start_time(
        schedule, activity
    )


def get_temporal_refinement(
    problem: SchedulingMotionProblem,
    schedule: Schedule,
    overlapping_activities: Union[
        List[MotionActivity], List[Tuple[MotionActivity, MotionActivity]]
    ],
    motion_planner_durations: Dict[MotionActivity, Tuple[float, float]],
    is_single_activity_layer: bool,
    activities_conditions: Dict[
        Tuple[MotionActivity, MovableObject, ConfigurationObject], Activity
    ] = {},
) -> FNode:
    em: ExpressionManager = problem.environment.expression_manager

    if isinstance(overlapping_activities[0], tuple):
        assert is_single_activity_layer
        scheduled_activities = [act[1] for act in overlapping_activities]
        overlapping_activities = [act[0] for act in overlapping_activities]
    else:
        scheduled_activities = overlapping_activities

    premise = refinement_premise(
        problem, overlapping_activities, is_single_activity_layer
    )

    learned_constraint = []
    if len(overlapping_activities) == 1:
        activity_a = overlapping_activities[0]
        delta_a_bar = motion_planner_durations[scheduled_activities[0]][0]
        w_a_bar = delta_a_bar  # delta_a = 0 if len(overlapping_activities) == 1
        duration_a_bar = motion_planner_durations[scheduled_activities[0]][1]
        learned_constraint.append(
            em.GE(
                em.Minus(activity_a.end, activity_a.start),
                math.ceil(duration_a_bar + w_a_bar),
            )
        )
    else:
        cc0 = []
        for first_activity in overlapping_activities:
            is_first_activity = first_activity_condition(
                first_activity, overlapping_activities, em
            )

            cc1 = []
            for activity_a in motion_planner_durations:
                if activity_a == first_activity:
                    continue
                delta_a = delay(activity_a, overlapping_activities, schedule)
                cc1.append(
                    em.LT(em.Minus(activity_a.start, first_activity.start), delta_a)
                )
            c1 = em.Or(cc1)

            cc1 = []
            for activity_a in motion_planner_durations:
                delta_a = delay(activity_a, overlapping_activities, schedule)
                duration_a = duration(activity_a, schedule)
                delta_a_bar = motion_planner_durations[activity_a][0]
                duration_a_bar = motion_planner_durations[activity_a][1]
                if delta_a + duration_a >= math.ceil(delta_a_bar + duration_a_bar):
                    continue
                cc1.append(
                    em.GE(
                        em.Minus(activity_a.end, first_activity.start),
                        math.ceil(duration_a_bar + delta_a_bar),
                    )
                )
            c2 = em.Or(cc1)
            cc0.append(em.Implies(is_first_activity, em.Or(c1, c2)))
        learned_constraint.append(em.And(cc0))

    for i, activity_a in enumerate(scheduled_activities):
        obstacles_with_configuration = get_obstacles_with_configuration(
            problem,
            schedule,
            activity_a,
            scheduled_activities,
            use_dynamic_obstacles=not is_single_activity_layer,
        )
        for obstacle, (
            conf_fluent,
            obstacle_conf,
        ) in obstacles_with_configuration.items():
            learned_constraint.append(
                obstacle_changes_configuration_constraint(
                    problem,
                    overlapping_activities[i],
                    obstacle,
                    obstacle_conf,
                    conf_fluent,
                    activities_conditions,
                )
            )

    learned_constraint = em.Or(learned_constraint)
    # print("premise", premise)
    # print("learned'", learned_constraint)
    return em.Implies(premise, learned_constraint)


def equivalent_groups(
    problem: SchedulingMotionProblem,
    group: List[MotionActivity],
    unreachable_locations: Dict[
        Tuple[MotionActivity, MotionConstraint], List[tamp.ConfigurationObject]
    ],
) -> List[List[MotionActivity]]:
    eq_activities_group = []

    for activity in group:
        key = (activity, activity.motion_constraints[0])

        # Check both that the dict is not empty and that the key exists
        if unreachable_locations and key in unreachable_locations:
            u_locs = unreachable_locations[key]
        else:
            # Fallback: empty list (or whatever default makes sense)
            u_locs = []

        eq_activities_group.append(
            equivalent_activities(
                problem,
                activity,
                u_locs,
            )
        )

    eq_groups = list(itertools.product(*eq_activities_group))
    return eq_groups


def equivalent_activities(
    problem: SchedulingMotionProblem,
    activity: MotionActivity,
    unreachable_locations: List[tamp.ConfigurationObject],
) -> List[MotionActivity]:

    movable = list(activity.motion_effects.keys())[0].object()
    starting = activity.motion_constraints[0].starting.object()
    activities = []

    for act in problem.motion_activities:
        assert (
            len(act.motion_effects) == 1
        ), "Exactly one motion effect should be defined"
        assert (
            len(act.motion_constraints) <= 1
        ), "At most one motion constraint should be defined"
        if len(act.motion_constraints) == 0:
            continue
        movable2, conf = list(act.motion_effects.items())[0]
        movable2 = movable2.object()
        starting2 = act.motion_constraints[0].starting.object()
        conf = conf.object()

        if (
            movable2 == movable
            and conf in unreachable_locations
            and starting2 == starting
        ):
            activities.append(act)

    # print(f"equivalent_activities({activity.name})", list(a.name for a in activities))
    if (
        not activity in activities
    ):  # unreachable_locations is empty because the motion planner did not find a solution even if it exists
        activities.append(activity)
    return activities


def rival_activities(
    problem: SchedulingMotionProblem, overlapping_activities: List[MotionActivity]
) -> List[MotionActivity]:
    """
    Activities potentially involving the movement of an obstacle present in one of the activities
    of `overlapping_activities`, either as a movable object or as an obstacle of the associated
    motion constraint.
    """

    obstacles = set()
    for act in overlapping_activities:
        for movable in act.motion_effects:
            obstacles.add(movable.object())

        for motion_constraint in act.motion_constraints:
            assert isinstance(motion_constraint, ActivityWaypoints)
            obstacles.add(motion_constraint.movable.object())
            oo = []
            if motion_constraint.static_obstacles is not None:
                oo += [obstacle for obstacle in motion_constraint.static_obstacles]
            if motion_constraint.dynamic_obstacles_at_start is not None:
                oo += [
                    obstacle
                    for obstacle in motion_constraint.dynamic_obstacles_at_start
                ]
            for obstacle in oo:
                obstacles.add(obstacle)

    activities = []
    for act in problem.motion_activities:
        if act in overlapping_activities:
            continue

        is_rival = False
        for movable in act.motion_effects:
            if movable.object() in obstacles:
                is_rival = True
                break

        if not is_rival:
            for motion_constraint in act.motion_constraints:
                assert isinstance(motion_constraint, ActivityWaypoints)
                if motion_constraint.movable.object() in obstacles:
                    is_rival = True
                    break

        if is_rival:
            activities.append(act)

    # print(
    #     f"rival_activities({list(a.name for a in overlapping_activities)}) = ",
    #     list(a.name for a in activities),
    # )
    return activities


def helper_activities(
    problem: SchedulingMotionProblem,
    obstacle: MovableObject,
    obstacle_conf: tamp.ConfigurationObject,
) -> List[MotionActivity]:

    activities = []
    for activity in problem.motion_activities:
        for movable, config in activity.motion_effects.items():
            if movable.object() == obstacle and config.object() != obstacle_conf:
                activities.append(activity)

    # print(
    #     f"helper_activities({obstacle.name}, {obstacle_conf.name})",
    #     list(a.name for a in activities),
    # )
    return activities


def deleter_activities(
    problem: SchedulingMotionProblem,
    obstacle: MovableObject,
    obstacle_conf: tamp.ConfigurationObject,
) -> List[MotionActivity]:

    activities = []
    for activity in problem.motion_activities:
        for movable, config in activity.motion_effects.items():
            if movable.object() == obstacle and config.object() == obstacle_conf:
                activities.append(activity)

    # print(
    #     f"deleter_activities({obstacle.name}, {obstacle_conf.name})",
    #     list(a.name for a in activities),
    # )
    return activities
