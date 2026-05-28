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

from unified_planning.engines import UPSequentialSimulator
from unified_planning.engines import TimeTriggeredPlanValidator
from unified_planning.model.motion.action import (
    InstantaneousMotionAction,
    DurativeMotionAction,
)
from unified_planning.plans import PlanKind
from tampest.motion.motion_planner import MotionPlanner


def getOverlappingActions(timed_actions):
    grouped_actions = []
    current_group = []
    current_end = None

    sorted_actions = timed_actions
    sorted_actions.sort(key=lambda x: x[0])

    for ai in sorted_actions:
        if isinstance(ai[1].action, DurativeMotionAction):
            if not current_group:
                current_group.append(ai)
                current_end = ai[0] + ai[2]
            else:
                if current_end > ai[0]:
                    current_end = max(current_end, ai[0] + ai[2])
                    current_group.append(ai)
                else:
                    grouped_actions.append(current_group)
                    current_group = [ai]
                    current_end = ai[0] + ai[2]

    if current_group:
        grouped_actions.append(current_group)

    return grouped_actions


def getConstraintParams(state, action, mc):

    params = {k: v for k, v in zip(action.action.parameters, action.actual_parameters)}

    movable = mc.movable
    if mc.movable.is_parameter_exp():
        movable = params[mc.movable.parameter()]
    elif mc.movable.is_fluent_exp():
        movable = state.get_value(mc.movable)
    starting = mc.starting
    if mc.starting.is_parameter_exp():
        starting = params[mc.starting.parameter()]
    elif mc.starting.is_fluent_exp():
        starting = state.get_value(mc.starting)
    if len(mc.waypoints) != 1:
        raise NotImplementedError
    waypoint = mc.waypoints[0]
    if mc.waypoints[0].is_parameter_exp():
        waypoint = params[mc.waypoints[0].parameter()]
    elif mc.waypoints[0].is_fluent_exp():
        waypoint = state.get_value(mc.waypoints[0])
    static_obstacles_pos = {}

    if mc.static_obstacles:
        for o, fe in mc.static_obstacles.items():
            if movable.object() != o:
                static_obstacles_pos[o] = state.get_value(fe).object()

    return (
        movable.object(),
        starting.object(),
        waypoint.object(),
        static_obstacles_pos,
    )


def getActionMaxDuration(state, action_instance):
    action = action_instance.action
    upper_duration = action.duration.upper

    d_max = None

    if upper_duration.is_fluent_exp():
        params = {
            k: v for k, v in zip(action.parameters, action_instance.actual_parameters)
        }
        fluent_params = []
        for arg in upper_duration.args:
            if arg.is_parameter_exp():
                fluent_params.append(params[arg.parameter()])
            elif arg.is_fluent_exp():
                fluent_params.append(state.get_value(arg))
            else:
                fluent_params.append(arg)
        d_max = state.get_value(upper_duration.fluent()(*fluent_params))
    else:
        d_max = upper_duration

    if d_max.type.is_int_type():
        d_max = d_max.int_constant_value()
    elif d_max.type.is_real_type():
        d_max = d_max.real_constant_value()
    else:
        raise TypeError(f"{d_max.type} not supported")

    return d_max


def check_plan(
    cache,
    problem,
    plan,
    motion_planning_time,
    interpolate,
    simplified,
    distance,
    motion_planner,
    topological_refinement,
    max_radius_bound,
    motion_planning_data,
):
    print("Plan to check:")
    print(plan)

    mp = MotionPlanner()

    n_instance = len(motion_planning_data)

    if plan.kind == PlanKind.TIME_TRIGGERED_PLAN:
        validator = TimeTriggeredPlanValidator()
        validator.skip_checks = True
        res = validator.validate(problem, plan)
        em = problem.environment.expression_manager

        # group overlapping motion actions
        overlapping_actions = getOverlappingActions(plan.timed_actions)

        # check each group of actions
        for group in overlapping_actions:

            action_keys = []

            constraints_map = {}
            n = 0
            ai_to_start_dur = {}

            start_zero = group[0][0]
            motion_constraints = {}
            for ai in group:
                ai_to_start_dur[ai[1]] = ai[0], ai[2]

                action_start = ai[0]
                a = ai[1].action
                ai[1]._motion_paths = {}

                if isinstance(a, DurativeMotionAction):

                    closest_time = max(
                        [value for value in res.trace.keys() if value <= action_start]
                    )
                    state = res.trace[closest_time]

                    d_max = getActionMaxDuration(state, ai[1])

                    motion_constraints_key = []
                    for mc in a.motion_constraints:
                        movable, starting, waypoint, obstacles = (
                            getConstraintParams(state, ai[1], mc)
                        )
                        static_obstacles_pos_list = []
                        if obstacles:
                            static_obstacles_pos_list = [
                                obstacles[k]
                                for k in sorted(obstacles.keys(), key=str)
                            ]
                        motion_constraints_key.append(
                            (
                                movable,
                                starting,
                                waypoint,
                                tuple(static_obstacles_pos_list),
                            )
                        )
                        constraints_map[(ai[1], mc)] = n
                        motion_constraints[(ai[1], mc)] = (
                            action_start - start_zero,
                            ai[2],  # activity_duration (actual plan duration)
                            d_max,
                            *getConstraintParams(state, ai[1], mc),
                        )
                        n += 1

                    action_keys.append(
                        (
                            a.name,
                            action_start - start_zero,
                            ai[2],
                            *ai[1].actual_parameters,
                            tuple(motion_constraints_key),
                        )
                    )

            key = tuple(action_keys)

            if key in cache:
                is_valid = True
                for k, v in cache[key].items():
                    for ai in group:
                        if (
                            ai[1].actual_parameters == k[0].actual_parameters
                            and ai[1].action.name == k[0].action.name
                        ):
                            paths = {(ai[1], k[1]): v}
                # paths = cache[key]
            else:

                # motion_constraints = {mc: (action_start, d_max, movable.object(), starting.object(), waypoint.object(), static_obstacles_pos)}
                # constraints_map = {(ai, mc): 0} ------------ used to assigned an id (name) to each motion planning problem

                print("Check motion constraints for overlapping actions")
                print(group)

                # paths = {(ai, mc): path}
                # current_durations = {ai: current_duration}
                (
                    is_valid,
                    _is_temporal_valid,
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
                    hull_enabled=True,
                )

                print("is_valid:", is_valid)
                print(current_durations)

                motion_planning_data[n_instance] = mc_planning_data
                n_instance += 1

                if is_valid:
                    is_temporally_valid = True
                    for ai, (start, dur) in ai_to_start_dur.items():
                        wait = current_durations[ai][0] - (
                            float(start) - float(start_zero)
                        )
                        assert wait > -0.0000001
                        wait = max(0, wait)
                        if current_durations[ai][1] + wait > dur:
                            is_temporally_valid = False
                            break
                    if not is_temporally_valid:
                        is_valid = False
                    else:
                        cache[key] = paths

            if is_valid:
                for k, v in paths.items():
                    k[0].motion_paths[k[1]] = v
            elif (
                current_durations
            ):  # at least one ai has a duration > than the expected
                new_durations = {}
                for ai, (start, dur) in ai_to_start_dur.items():
                    wait = current_durations[ai][0] - (float(start) - float(start_zero))
                    assert wait > -0.0000001
                    wait = max(0, wait)
                    new_durations[ai] = (start, dur, wait, current_durations[ai][1])
                return False, new_durations, None
            else:
                new_conds = []
                for (ai, mc), ug in unreachable_goals.items():
                    obs = (
                        static_obstacles[(ai, mc)]
                        if (ai, mc) in static_obstacles
                        else []
                    )
                    print(ai, ug, obs)
                    a = ai.action
                    action_start, _ = ai_to_start_dur[ai]
                    closest_time = max(
                        [value for value in res.trace.keys() if value <= action_start]
                    )
                    state = res.trace[closest_time]
                    movable, starting, _, _ = getConstraintParams(state, ai, mc)
                    conds = []
                    conds.append(em.Equals(mc.movable, movable))
                    conds.append(em.Equals(mc.starting, starting))
                    for o in obs:
                        fe = mc.static_obstacles[o]
                        conds.append(em.Equals(fe, state.get_value(fe)))
                    unreachable_conds = []
                    for wp in mc.waypoints:
                        for u in ug:
                            # unreachable_conds.append(em.Not(em.Equals(wp, u)))
                            unreachable_conds.append(em.Equals(wp, u))
                    # new_conds.append((a, em.Implies(em.And(conds), em.And(unreachable_conds))))
                    new_conds.append(
                        (a, em.Not(em.And(em.And(conds), em.Or(unreachable_conds))))
                    )
                return False, None, new_conds

    if plan.kind == PlanKind.SEQUENTIAL_PLAN:
        simulator = UPSequentialSimulator(problem, error_on_failed_checks=False)

        state = simulator.get_initial_state()
        em = problem.environment.expression_manager

        for ai in plan.actions:

            a = ai.action
            if isinstance(a, InstantaneousMotionAction):

                ai._motion_paths = {}

                for mc in a.motion_constraints:

                    static_obstacles_pos_list = []
                    movable, starting, waypoint, obstacles = getConstraintParams(
                        state, ai, mc
                    )

                    if obstacles:
                        static_obstacles_pos_list = [
                            obstacles[k] for k in sorted(obstacles.keys(), key=str)
                        ]

                    key = (
                        a.name,
                        ai.actual_parameters,
                        movable,
                        starting,
                        waypoint,
                        tuple(static_obstacles_pos_list),
                    )

                    if key in cache:
                        is_valid = True
                        path = cache[key]
                    else:
                        print("Check motion constraint", ai)

                        (
                            is_valid,
                            _is_temporal_valid,
                            path,
                            _,
                            _reachable,
                            unreachable_goals,
                            static_obstacles,
                            mc_planning_data,
                        ) = mp.check_motion_constraint(
                            {0: movable},
                            {0: starting},
                            {0: waypoint},
                            obstacles,
                            problem.all_objects,
                            planning_time=motion_planning_time,
                            interpolate=interpolate,
                            simplified=simplified,
                            distance=distance,
                            motion_planner=motion_planner,
                            topological_refinement=topological_refinement,
                            max_radius_bound=max_radius_bound,
                            hull_enabled=True,
                        )

                        motion_planning_data[mc] = mc_planning_data
                    if is_valid:
                        cache[key] = path
                        ai.motion_paths[mc] = path
                    else:

                        conds = []
                        conds.append(em.Equals(mc.movable, movable))
                        conds.append(em.Equals(mc.starting, starting))
                        if 0 in static_obstacles:
                            for o in static_obstacles[0]:
                                fe = mc.static_obstacles[o]
                                conds.append(em.Equals(fe, state.get_value(fe)))
                        unreachable_conds = []
                        for wp in mc.waypoints:
                            if 0 in unreachable_goals:
                                for u in unreachable_goals[0]:
                                    # unreachable_conds.append(em.Not(em.Equals(wp, u)))
                                    unreachable_conds.append(em.Equals(wp, u))
                        # new_conds.append((a, em.Implies(em.And(conds), em.And(unreachable_conds))))
                        new_cond = (
                            a,
                            em.Not(em.And(em.And(conds), em.Or(unreachable_conds))),
                        )
                        return False, None, [new_cond]

            state = simulator.apply(state, a, ai.actual_parameters)

    return True, None, None
