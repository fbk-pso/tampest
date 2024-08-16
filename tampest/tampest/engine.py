import pysmt
import warnings
import unified_planning as up
from unified_planning.model import ProblemKind
from unified_planning.engines import PlanGenerationResultStatus
from unified_planning.engines import PlanGenerationResult, Credits
from unified_planning.engines import UPSequentialSimulator
from unified_planning.engines.results import correct_plan_generation_result
from unified_planning.engines.compilers.utils import replace_action
from unified_planning.model.tamp.action import InstantaneousMotionAction
from unified_planning.plans import TimeTriggeredPlan
from typing import IO, Callable, Optional
from pysmt.shortcuts import Solver
from tempest.encoder import ProblemEncoder
from tampest.motion_planner import MotionPlanner
from tampest.motion_planning_data import SupportedTopologicalRefinement, SupportedPlanner
from functools import partial


class TampestEngine(up.engines.Engine, up.engines.mixins.OneshotPlannerMixin):
    """Implementation of the TAMPEST Engine."""

    def __init__(
            self, incremental=True, step_horizon=None, motion_planning_time=3.0,
            interpolate=False, simplified=False, tolerance=0.0,
            distance=None, motion_planner=SupportedPlanner.RRT,
            topological_refinement=SupportedTopologicalRefinement.ALL
    ):
        up.engines.Engine.__init__(self)
        up.engines.mixins.OneshotPlannerMixin.__init__(self)
        self._step_horizon = step_horizon
        self._incremental = incremental
        self._motion_planning_time = motion_planning_time
        self._interpolate = interpolate
        self._simplified = simplified
        self._tolerance = tolerance
        self._distance = distance
        self._motion_planner = motion_planner
        self._topological_refinement = topological_refinement

    @property
    def name(self) -> str:
        return "TAMPEST"


    @staticmethod
    def supported_kind() -> ProblemKind:
        supported_kind = ProblemKind()
        supported_kind.set_problem_class("TAMP")
        supported_kind.set_problem_class("ACTION_BASED")
        supported_kind.set_problem_type("SIMPLE_NUMERIC_PLANNING")
        supported_kind.set_problem_type("GENERAL_NUMERIC_PLANNING")
        supported_kind.set_numbers("CONTINUOUS_NUMBERS")
        supported_kind.set_numbers("DISCRETE_NUMBERS")
        supported_kind.set_numbers("BOUNDED_TYPES")
        supported_kind.set_conditions_kind("NEGATIVE_CONDITIONS")
        supported_kind.set_conditions_kind("DISJUNCTIVE_CONDITIONS")
        supported_kind.set_conditions_kind("EQUALITIES")
        supported_kind.set_effects_kind("CONDITIONAL_EFFECTS")
        supported_kind.set_effects_kind("INCREASE_EFFECTS")
        supported_kind.set_effects_kind("DECREASE_EFFECTS")
        supported_kind.set_effects_kind("STATIC_FLUENTS_IN_BOOLEAN_ASSIGNMENTS")
        supported_kind.set_effects_kind("STATIC_FLUENTS_IN_NUMERIC_ASSIGNMENTS")
        supported_kind.set_effects_kind("STATIC_FLUENTS_IN_OBJECT_ASSIGNMENTS")
        supported_kind.set_effects_kind("FLUENTS_IN_BOOLEAN_ASSIGNMENTS")
        supported_kind.set_effects_kind("FLUENTS_IN_NUMERIC_ASSIGNMENTS")
        supported_kind.set_effects_kind("FLUENTS_IN_OBJECT_ASSIGNMENTS")
        supported_kind.set_typing("FLAT_TYPING")
        supported_kind.set_parameters("BOOL_FLUENT_PARAMETERS")
        supported_kind.set_parameters("BOUNDED_INT_FLUENT_PARAMETERS")
        supported_kind.set_parameters("BOOL_ACTION_PARAMETERS")
        supported_kind.set_parameters("BOUNDED_INT_ACTION_PARAMETERS")
        supported_kind.set_parameters("UNBOUNDED_INT_ACTION_PARAMETERS")
        supported_kind.set_parameters("REAL_ACTION_PARAMETERS")
        supported_kind.set_fluents_type("NUMERIC_FLUENTS")
        supported_kind.set_fluents_type("OBJECT_FLUENTS")
        return supported_kind

    @staticmethod
    def supports(problem_kind: "up.model.ProblemKind") -> bool:
        return problem_kind <= TampestEngine.supported_kind()

    def _solve(
        self,
        problem: "up.model.AbstractProblem",
        heuristic: Optional[Callable[["up.model.state.State"], Optional[float]]] = None,
        timeout: Optional[float] = None,
        memout: Optional[float] = None,
        output_stream: Optional[IO[str]] = None,
    ) -> "up.engines.results.PlanGenerationResult":
        assert isinstance(problem, up.model.Problem)
        if timeout is not None:
            warnings.warn("TemPEST does not support timeout.", UserWarning)
        if memout is not None:
            warnings.warn("TemPEST does not support memout.", UserWarning)
        if output_stream is not None:
            warnings.warn("TemPEST does not support output stream.", UserWarning)
        if heuristic is not None:
            warnings.warn("TemPEST does not support custom heuristics.", UserWarning)

        orig_problem = problem
        em = problem.environment.expression_manager
        pysmt_env = pysmt.shortcuts.get_env()
        motion_planning_time = self._motion_planning_time
        cache = {}

        while True:
            problem = problem.clone()

            new_to_old = {}
            for a in problem.actions:
                orig_a = orig_problem.action(a.name)
                if isinstance(a, InstantaneousMotionAction):
                    for mc in a.motion_constraints:
                        if mc.obstacles:
                            for mo, fe in mc.obstacles.items():
                                if mo.type == mc.movable.type:
                                    a.add_precondition(em.Implies(em.Equals(mc.movable, mo), em.Equals(mc.starting, fe)))
                new_to_old[a] = orig_a

            enc = ProblemEncoder(problem, pysmt_env=pysmt_env)

            if self._incremental:
                with Solver(logic="QF_LRA") as smt:
                    smt.add_assertion(enc.incremental_step_zero())
                    h = 2
                    old_conds = []
                    while self._step_horizon is None or h <= self._step_horizon:
                        formula, temp_formula = enc.incremental_step(h-1)
                        smt.add_assertion(formula)
                        for a, c in old_conds:
                            smt_a = enc.a(a, h - 1)
                            smt_c = enc.to_smt(c, h - 2, h - 1, scope=a)
                            f = enc.mgr.Implies(smt_a, smt_c)
                            smt.add_assertion(f)
                        smt.push()
                        smt.add_assertion(temp_formula)
                        while smt.solve():
                            plan = enc.extract_plan(smt.get_model(), h)
                            is_valid, new_conds, planning_data = check_plan(cache, problem, plan, motion_planning_time, self._interpolate,
                                                                            self._simplified, self._tolerance, self._distance,
                                                                            self._motion_planner, self._topological_refinement)
                            if is_valid:
                                return PlanGenerationResult(
                                    PlanGenerationResultStatus.SOLVED_SATISFICING,
                                    plan.replace_action_instances(partial(replace_action, map=new_to_old)),
                                    self.name,
                                    metrics=metrics_from_planning_data(planning_data),
                                )
                            else:
                                smt.pop()
                                for a, c in new_conds:
                                    print(a.name, c)
                                    for i in range(1, h):
                                        smt_c = enc.to_smt(c, i - 1, i, scope=a)
                                        f = enc.mgr.Implies(enc.a(a, i), smt_c)
                                        smt.add_assertion(f)
                                old_conds.extend(new_conds)
                                smt.push()
                                smt.add_assertion(temp_formula)
                        smt.pop()
                        print(f"No solution with bound {h}")
                        h += 1
            else:
                h = 2
                old_conds = []
                while self._step_horizon is None or h <= self._step_horizon:
                    formula = enc.monolithic_bounded_planning(h)
                    with Solver(logic="QF_LRA") as smt:
                        smt.add_assertion(formula)
                        for a, c in old_conds:
                            for i in range(1, h):
                                smt_c = enc.to_smt(c, i - 1, i, scope=a)
                                f = enc.mgr.Implies(enc.a(a, i), smt_c)
                                smt.add_assertion(f)
                        while smt.solve():
                            plan = enc.extract_plan(smt.get_model(), h)
                            is_valid, new_conds, planning_data = check_plan(cache, problem, plan, motion_planning_time,
                                                                            self._interpolate, self._simplified,
                                                                            self._tolerance, self._distance,
                                                                            self._motion_planner, self._topological_refinement)
                            if is_valid:
                                return PlanGenerationResult(
                                    PlanGenerationResultStatus.SOLVED_SATISFICING,
                                    plan.replace_action_instances(partial(replace_action, map=new_to_old)),
                                    self.name,
                                    metrics=metrics_from_planning_data(planning_data),
                                )
                            else:
                                for a, c in new_conds:
                                    print(a.name, c)
                                    for i in range(1, h):
                                        smt_c = enc.to_smt(c, i - 1, i, scope=a)
                                        f = enc.mgr.Implies(enc.a(a, i), smt_c)
                                        smt.add_assertion(f)
                                old_conds.extend(new_conds)
                        print(f"No solution with bound {h}")
                        h += 1

            motion_planning_time *= 2

        return PlanGenerationResult(
            PlanGenerationResultStatus.UNSOLVABLE_INCOMPLETELY,
            None,
            self.name,
        )


def metrics_from_planning_data(planning_data):
    pt = 0
    it = 0
    fcht = 0
    nw = 0
    nwai = 0
    pl = 0
    for d in planning_data.values():
        if d.planning_time is not None:
            pt += d.planning_time
        if d.interpolation_time is not None:
            it += d.interpolation_time
        if d.convex_hull_time is not None:
            fcht += d.convex_hull_time
        if d.n_waypoints is not None:
            nw += d.n_waypoints
        if d.n_waypoints_after_interpolation is not None:
            nwai += d.n_waypoints_after_interpolation
        if d.path_length is not None:
            pl += d.path_length
    metrics = {}
    metrics["motion_planning_time"] = str(pt)
    metrics["interpolation_time"] = str(it)
    metrics["convex_hull_time"] = str(fcht)
    metrics["n_waypoints"] = str(nw)
    metrics["n_waypoints_after_interpolation"] = str(nwai)
    metrics["path_length"] = str(pl)
    return metrics


def check_plan(cache, problem, plan, motion_planning_time, interpolate, simplified, tolerance, distance, motion_planner, topological_refinement):
    is_valid = True
    simulator = UPSequentialSimulator(problem, error_on_failed_checks=False)
    state = simulator.get_initial_state()
    em = problem.environment.expression_manager
    new_conds = []
    planning_data = {}
    print("Plan to check:")
    print(plan)
    for ai in plan.actions:
        a = ai.action
        if isinstance(a, InstantaneousMotionAction):
            params = {k: v for k, v in zip(a.parameters, ai.actual_parameters)}
            ai._motion_paths = {}
            mp = MotionPlanner()
            for mc in a.motion_constraints:
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
                obstacles_pos = {}
                obstacles_pos_list = []
                if mc.obstacles:
                    for o, fe in mc.obstacles.items():
                        if movable.object() != o:
                            obstacles_pos[o] = state.get_value(fe).object()
                    obstacles_pos_list = [obstacles_pos[k] for k in sorted(obstacles_pos.keys(), key=str)]
                key = (a.name, ai.actual_parameters, movable, starting, waypoint, tuple(obstacles_pos_list))
                if key in cache:
                    is_valid = True
                    path = cache[key]
                else:
                    print("Check motion constraints", ai)
                    is_valid, path, unreachable_goals, obstacles, mc_planning_data = mp.check_motion_constraint(movable.object(),
                                                                                                                starting.object(),
                                                                                                                waypoint.object(),
                                                                                                                obstacles_pos,
                                                                                                                problem.all_objects,
                                                                                                                planning_time=motion_planning_time,
                                                                                                                interpolate=interpolate,
                                                                                                                simplified=simplified,
                                                                                                                tolerance=tolerance,
                                                                                                                distance=distance,
                                                                                                                motion_planner=motion_planner,
                                                                                                                topological_refinement=topological_refinement)
                    planning_data[mc] = mc_planning_data
                if is_valid:
                    cache[key] = path
                    ai.motion_paths[mc] = path
                else:
                    conds = []
                    conds.append(em.Equals(mc.movable, movable))
                    conds.append(em.Equals(mc.starting, starting))
                    for o in obstacles:
                        fe = mc.obstacles[o]
                        conds.append(em.Equals(fe, state.get_value(fe)))
                    unreachable_conds = []
                    for wp in mc.waypoints:
                        for u in unreachable_goals:
                            #unreachable_conds.append(em.Not(em.Equals(wp, u)))
                            unreachable_conds.append(em.Equals(wp, u))
                    #new_conds.append((a, em.Implies(em.And(conds), em.And(unreachable_conds))))
                    new_conds.append((a, em.Not(em.And(em.And(conds), em.Or(unreachable_conds)))))
                    return is_valid, new_conds, planning_data #break
            if not is_valid:
                break
        state = simulator.apply(state, a, ai.actual_parameters)
    return is_valid, new_conds, planning_data
