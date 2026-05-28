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

import time
import warnings
import unified_planning as up
from unified_planning.model import ProblemKind, ExpressionManager
from unified_planning.engines import PlanGenerationResultStatus, PlanGenerationResult
from unified_planning.model.motion import (
    SchedulingMotionProblem,
    MotionActivity,
    MotionConstraint,
)
from unified_planning.model.scheduling import SchedulingProblem
from unified_planning.plans import Schedule
from unified_planning.shortcuts import (
    BoolType,
    TimePointInterval,
    Timing,
    ClosedTimeInterval,
)
from typing import IO, Callable, Optional, Type, Dict, Tuple, List
from tampest.check_schedule import check_schedule, filter_motion_paths
from tampest.motion.motion_planning_data import (
    SupportedTopologicalRefinement,
    SupportedPlanner,
    resolve_motion_params,
)
from unified_planning.plans import MotionSchedule


class SampMetaEngine(up.engines.MetaEngine, up.engines.mixins.OneshotPlannerMixin):
    def __init__(
        self,
        motion_planning_time: float = 3.0,
        interpolate: bool = False,
        simplified: bool = False,
        distance: Optional[float] = None,
        motion_planner: SupportedPlanner = SupportedPlanner.STRRTstar,
        topological_refinement: SupportedTopologicalRefinement = SupportedTopologicalRefinement.ALL,
        max_radius_bound: bool = False,
        use_fluents: bool = False,
        enable_reachable_generalization: bool = True,
        enable_movable_generalization: bool = True,
        force_sequential_MP_check: bool = False,
        safety_distance: Optional[float] = None,
    ):
        """Initialise the SAMP meta-engine.

        Standard motion-planning parameters (shared with the other TAMPEST engines):
          motion_planning_time: per-query OMPL timeout in seconds.
          interpolate: densify the returned path by interpolating between waypoints.
          simplified: apply OMPL's path simplifier after planning.
          distance: uniform inflation radius applied to all obstacles.
          motion_planner: OMPL algorithm to use (STRRTstar, RRT, LazyRRT, RRTConnect).
          topological_refinement: filter unreachable goals / irrelevant obstacles before
            calling the motion planner (NONE, UNREACH, OBS, ALL).
          max_radius_bound: cap the STRRTstar radius bound to the initial sample radius.

        Scheduling+motion parameters (samp-specific):
          use_fluents: model robot-configuration occupancy as a fluent updated by motion
            activities, rather than as a cumulative resource on the scheduler side.
            Required when the back-end scheduler does not support cumulative resources.
          enable_reachable_generalization: generalise unreachability conclusions across
            activities that share the same start/goal regions, so a single failed motion
            query rules out an entire class of scheduling assignments at once.
          enable_movable_generalization: extend the same generalisation across movable
            objects when they occupy the same configuration space.
          force_sequential_MP_check: disable concurrent motion-planning queries; run one
            activity at a time. Useful for deterministic reproducibility comparisons.
          safety_distance: minimum clearance enforced between simultaneously moving
            robots. None uses the engine default (no extra inflation beyond `distance`).
        """
        up.engines.MetaEngine.__init__(self)
        up.engines.mixins.OneshotPlannerMixin.__init__(self)
        self._motion_planning_time = motion_planning_time
        self._interpolate = interpolate
        self._simplified = simplified
        self._distance = distance
        self._motion_planner = motion_planner
        self._topological_refinement = topological_refinement
        self._max_radius_bound = max_radius_bound
        self._use_fluents = use_fluents
        self._enable_reachable_generalization = enable_reachable_generalization
        self._enable_movable_generalization = enable_movable_generalization
        self._force_sequential_MP_check = force_sequential_MP_check
        self._safety_distance = safety_distance

    @property
    def name(self) -> str:
        return f"SampMetaEngine[{self.engine.name}]"

    @staticmethod
    def is_compatible_engine(engine: Type[up.engines.Engine]) -> bool:
        if not engine.is_oneshot_planner():
            return False
        needed_kind = ProblemKind(version=2)
        needed_kind.set_problem_class("SCHEDULING")
        # needed_kind.set_scheduling("OPTIONAL_ACTIVITIES")
        # needed_kind.set_scheduling("SCOPED_CONSTRAINTS")
        return engine.supports(needed_kind)

    @staticmethod
    def _supported_kind(engine: Type[up.engines.Engine]) -> ProblemKind:
        # TODO
        supported_kind = engine.supported_kind()
        supported_kind.set_problem_class("SAMP")
        supported_kind.unset_time("CONTINUOUS_TIME")
        return supported_kind

    @staticmethod
    def _supports(problem_kind: ProblemKind, engine: Type[up.engines.Engine]) -> bool:
        return problem_kind <= SampMetaEngine._supported_kind(engine)

    @staticmethod
    def satisfies(optimality_guarantee: "up.engines.OptimalityGuarantee") -> bool:
        return False

    @staticmethod
    def get_credits(**kwargs) -> Optional["up.engines.Credits"]:
        return None  # TODO: return proper engine credits

    def _solve(
        self,
        problem: "up.model.AbstractProblem",
        heuristic: Optional[Callable[["up.model.state.State"], Optional[float]]] = None,
        timeout: Optional[float] = None,
        memout: Optional[float] = None,
        output_stream: Optional[IO[str]] = None,
    ) -> "up.engines.results.PlanGenerationResult":
        assert isinstance(problem, SchedulingMotionProblem)
        problem: SchedulingMotionProblem

        if heuristic is not None:
            warnings.warn(
                f"{self.name} does not support custom heuristics.", UserWarning
            )
        if memout is not None:
            warnings.warn(f"{self.name} does not support memout.", UserWarning)

        # avoid exception when passing a SchedulingMotionProblem to the engine
        self.engine.error_on_failed_checks = False

        distance = resolve_motion_params(
            self._motion_planner, self._distance, requires_spacetime=True
        )

        deadline = None if timeout is None else time.time() + timeout
        motion_planning_time = self._motion_planning_time
        orig_problem = problem
        problem = orig_problem.clone()
        self._prepare_problem(problem)
        topological_cache = set()
        topological_temporal_cache = dict()
        topological_refinements_cache = set()
        activities_conditions = {}
        motion_paths: Dict[
            Tuple[MotionActivity, MotionConstraint], List[Tuple[float, ...]]
        ] = {}

        first_iteration = True
        while deadline is None or deadline > time.time():
            timeout = None if deadline is None else deadline - time.time()

            pure_sched = SchedulingProblem.clone(problem)

            res = self.engine.solve(
                pure_sched, timeout=timeout, output_stream=output_stream
            )

            if res.status in [
                PlanGenerationResultStatus.UNSOLVABLE_PROVEN,
                PlanGenerationResultStatus.UNSOLVABLE_INCOMPLETELY,
            ]:
                if first_iteration:
                    # TODO: can UNSOLVABLE_PROVEN be returned here?
                    break

                motion_planning_time *= 2
                print("Restart with time budget", motion_planning_time)
                problem = orig_problem.clone()
                self._prepare_problem(problem)
                activities_conditions = {}
                # TODO: cache and reuse the initial plan to avoid recomputing it
            elif res.status in [
                PlanGenerationResultStatus.SOLVED_SATISFICING,
                PlanGenerationResultStatus.SOLVED_OPTIMALLY,
            ]:
                schedule: Schedule = res.plan
                print("Plan found")
                print(schedule)
                is_valid, refinements, is_temporal_refinement = check_schedule(
                    problem,
                    schedule,
                    motion_planning_time,
                    self._interpolate,
                    self._simplified,
                    distance,
                    self._motion_planner,
                    self._topological_refinement,
                    self._max_radius_bound,
                    topological_cache,
                    topological_temporal_cache,
                    topological_refinements_cache,
                    motion_paths,
                    activities_conditions,
                    self._enable_reachable_generalization,
                    self._enable_movable_generalization,
                    self._force_sequential_MP_check,
                    self._safety_distance,
                )
                if is_valid:
                    motion_paths = filter_motion_paths(schedule, motion_paths)
                    activities_conditions_set = set(activities_conditions.values())
                    activities = list(
                        filter(
                            lambda act: act not in activities_conditions_set,
                            schedule.activities,
                        )
                    )
                    return PlanGenerationResult(
                        PlanGenerationResultStatus.SOLVED_SATISFICING,
                        MotionSchedule(
                            activities,
                            schedule.assignment,
                            motion_paths,
                            schedule.environment,
                        ),
                        self.name,
                        # metrics=metrics_from_planning_data(motion_planning_data),
                    )

                for c in refinements:
                    problem.add_constraint(c)

            else:
                print("Unexpected status:", res.status)
                raise NotImplementedError

            first_iteration = False

        return PlanGenerationResult(
            PlanGenerationResultStatus.UNSOLVABLE_INCOMPLETELY, None, self.name
        )

    def _prepare_problem(self, problem: SchedulingMotionProblem):
        em: ExpressionManager = problem.environment.expression_manager
        for i in range(len(problem.motion_activities) - 1):
            activity_a = problem.motion_activities[i]
            for j in range(i + 1, len(problem.motion_activities)):
                activity_b = problem.motion_activities[j]
                if self._in_conflict(activity_a, activity_b):
                    problem.add_constraint(
                        em.Or(
                            em.LE(activity_a.end, activity_b.start),
                            em.GE(activity_a.start, activity_b.end),
                        ),
                        scope=[activity_a.present, activity_b.present],
                    )

        initial_configurations = {
            conf_exp.object() for _, conf_exp in problem.initial_configuration
        }
        if self._use_fluents:
            is_free_fluents = {}
            for activity in problem.motion_activities:
                if len(activity.motion_constraints) == 0:
                    continue

                assert len(activity.motion_constraints) == 1
                starting_conf = activity.motion_constraints[0].starting.object()
                ending_conf = activity.motion_constraints[0].waypoints[0].object()
                for conf in [starting_conf, ending_conf]:
                    if conf not in is_free_fluents:
                        is_free_fluents[conf] = problem.add_fluent(
                            f"{conf.name}_is_free",
                            BoolType(),
                            default_initial_value=(conf not in initial_configurations),
                        )
                activity.add_condition(
                    TimePointInterval(Timing(-1, activity.end)),
                    is_free_fluents[ending_conf],
                )
                activity.add_effect(
                    Timing(0, activity.start), is_free_fluents[starting_conf], True
                )
                activity.add_effect(
                    Timing(-1, activity.end), is_free_fluents[ending_conf], False
                )
        else:
            for activity_a in problem.motion_activities:
                if len(activity_a.motion_constraints) == 0:
                    continue

                assert len(activity_a.motion_constraints) == 1
                ending_conf_a = activity_a.motion_constraints[0].waypoints[0].object()

                cc = []
                for activity_b in problem.motion_activities:
                    if (
                        activity_a == activity_b
                        or len(activity_b.motion_constraints) == 0
                    ):
                        continue

                    movable_b = activity_b.motion_constraints[0].movable.object()
                    ending_conf_b = (
                        activity_b.motion_constraints[0].waypoints[0].object()
                    )
                    if ending_conf_b != ending_conf_a:
                        continue

                    cc1 = []
                    for activity_c in problem.motion_activities:
                        if (
                            activity_a == activity_c
                            or activity_b == activity_c
                            or len(activity_c.motion_constraints) == 0
                        ):
                            continue

                        movable_c = activity_c.motion_constraints[0].movable.object()
                        starting_conf_c = activity_c.motion_constraints[
                            0
                        ].starting.object()
                        ending_conf_c = (
                            activity_c.motion_constraints[0].waypoints[0].object()
                        )
                        if not (
                            movable_b == movable_c
                            and starting_conf_c == ending_conf_b
                            and ending_conf_c != ending_conf_b
                        ):
                            continue

                        cc1.append(
                            em.And(
                                activity_c.present,
                                em.LT(activity_b.end, activity_c.end),
                                em.LT(activity_c.start, activity_a.end),
                            )
                        )

                    if len(cc1) > 0:
                        cc.append(
                            em.Implies(
                                em.And(
                                    activity_b.present,
                                    em.LE(activity_b.end, activity_a.end),
                                ),
                                em.Or(cc1),
                            )
                        )
                    else:
                        cc.append(
                            em.Not(
                                em.And(
                                    activity_b.present,
                                    em.LE(activity_b.end, activity_a.end),
                                )
                            )
                        )
                activity_a.add_constraint(em.And(cc))

                if ending_conf_a in initial_configurations:
                    other_activity_before = []
                    for activity_b in problem.motion_activities:
                        if (
                            activity_a == activity_b
                            or len(activity_b.motion_constraints) == 0
                        ):
                            continue

                        starting_conf_b = activity_b.motion_constraints[
                            0
                        ].starting.object()
                        if starting_conf_b != ending_conf_a:
                            continue

                        other_activity_before.append(
                            em.And(
                                activity_b.present,
                                em.LT(activity_b.start, activity_a.end),
                            )
                        )
                    activity_a.add_constraint(em.Or(other_activity_before))

    def _in_conflict(
        self, activity_a: MotionActivity, activity_b: MotionActivity
    ) -> bool:
        assert len(activity_a.motion_effects) == 1
        assert len(activity_b.motion_effects) == 1
        movable_a = list(activity_a.motion_effects.keys())[0].object()
        movable_b = list(activity_b.motion_effects.keys())[0].object()
        if movable_a == movable_b:
            return True
        if (
            len(activity_a.motion_constraints) > 0
            and activity_a.motion_constraints[0].static_obstacles is not None
            and movable_b in activity_a.motion_constraints[0].static_obstacles
        ):
            return True
        if (
            len(activity_b.motion_constraints) > 0
            and activity_b.motion_constraints[0].static_obstacles is not None
            and movable_a in activity_b.motion_constraints[0].static_obstacles
        ):
            return True
        return False
