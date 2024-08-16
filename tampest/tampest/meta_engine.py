import time
import warnings
import unified_planning as up
from unified_planning.model import ProblemKind
from unified_planning.engines import PlanGenerationResultStatus, PlanGenerationResult
from unified_planning.model.tamp import InstantaneousMotionAction
from unified_planning.engines.compilers.usertype_fluents_remover import UsertypeFluentsRemover
from unified_planning.plans import ActionInstance
from typing import IO, Callable, Optional, Type
from tampest.engine import check_plan, metrics_from_planning_data
from tampest.motion_planning_data import SupportedTopologicalRefinement, SupportedPlanner
from functools import partial


class TampMetaEngine(up.engines.MetaEngine, up.engines.mixins.OneshotPlannerMixin):
    def __init__(self, motion_planning_time=3.0,
                 interpolate=False, simplified=False,
                 tolerance=0.0, distance=None,
                 motion_planner=SupportedPlanner.RRT,
                 topological_refinement=SupportedTopologicalRefinement.ALL):
        up.engines.MetaEngine.__init__(self)
        up.engines.mixins.OneshotPlannerMixin.__init__(self)
        self._motion_planning_time = motion_planning_time
        self._interpolate = interpolate
        self._simplified = simplified
        self._tolerance = tolerance
        self._distance = distance
        self._motion_planner = motion_planner
        self._topological_refinement = topological_refinement

    @property
    def name(self):
        return f"TampMetaEngine[{self.engine.name}]"

    @staticmethod
    def is_compatible_engine(engine: Type[up.engines.Engine]) -> bool:
        if not engine.is_oneshot_planner():
            return False
        needed_kind = ProblemKind(version=2)
        needed_kind.set_problem_class("ACTION_BASED")
        if not engine.supports(needed_kind):
            return False
        return True

    @staticmethod
    def _supported_kind(engine: Type[up.engines.Engine]) -> "ProblemKind":
        supported_kind = engine.supported_kind()
        supported_kind.set_problem_class("TAMP")
        supported_kind.unset_time("CONTINUOUS_TIME")
        supported_kind.unset_time("DISCRETE_TIME")
        return supported_kind

    @staticmethod
    def _supports(problem_kind: "ProblemKind", engine: Type[up.engines.Engine]) -> bool:
        return problem_kind <= TampMetaEngine._supported_kind(engine)

    @staticmethod
    def satisfies(optimality_guarantee: "up.engines.OptimalityGuarantee") -> bool:
        return False

    @staticmethod
    def get_credits(**kwargs) -> Optional["up.engines.Credits"]:
        return None # TODO

    def _solve(
        self,
        problem: "up.model.AbstractProblem",
        heuristic: Optional[Callable[["up.model.state.State"], Optional[float]]] = None,
        timeout: Optional[float] = None,
        memout: Optional[float] = None,
        output_stream: Optional[IO[str]] = None,
    ) -> "up.engines.results.PlanGenerationResult":
        assert isinstance(problem, up.model.Problem)
        if heuristic is not None:
            warnings.warn(f"{self.name} does not support custom heuristics.", UserWarning)
        if memout is not None:
            warnings.warn(f"{self.name} does not support memout.", UserWarning)
        if timeout is None:
            deadline = None
        else:
            deadline = time.time() + timeout

        orig_problem = problem

        problem = problem.clone()
        motion_planning_time = self._motion_planning_time
        mc_map = prepare_problem(problem, orig_problem)
        cache = {}

        while True:
            for a in mc_map.keys():
                a._motion_constraints = []
            timeout = None if deadline is None else deadline-time.time()
            if timeout is not None and timeout < 0:
                break
            if self.engine.supported_kind().has_object_fluents():
                res = self.engine.solve(problem, timeout=timeout, output_stream=output_stream)
                plan = res.plan
            else:
                ufr = UsertypeFluentsRemover()
                comp_result = ufr.compile(problem)
                comp_problem = comp_result.problem
                res = self.engine.solve(comp_problem, timeout=timeout, output_stream=output_stream)
                plan = None if res.plan is None else res.plan.replace_action_instances(comp_result.map_back_action_instance)
            if plan is None:
                motion_planning_time *= 2
                print("Restart with time budget", motion_planning_time)
                problem = orig_problem.clone()
                mc_map = prepare_problem(problem, orig_problem)
            else:
                for a, mc in mc_map.items():
                    a._motion_constraints = mc
                is_valid, new_conds, planning_data = check_plan(cache, problem, plan, motion_planning_time, self._interpolate, self._simplified,
                                                                self._tolerance, self._distance,
                                                                self._motion_planner, self._topological_refinement)
                if is_valid:
                    return PlanGenerationResult(
                        PlanGenerationResultStatus.SOLVED_SATISFICING,
                        plan.replace_action_instances(partial(replace_action, orig_problem=orig_problem)),
                        self.name,
                        metrics=metrics_from_planning_data(planning_data),
                    )
                else:
                    for a, c in new_conds:
                        print(a.name, c)
                        a.add_precondition(c)

        return PlanGenerationResult(
            PlanGenerationResultStatus.UNSOLVABLE_INCOMPLETELY, None, self.name
        )


def replace_action(action_instance, orig_problem):
    return ActionInstance(orig_problem.action(action_instance.action.name), action_instance.actual_parameters)


def prepare_problem(problem, orig_problem):
    em = problem.environment.expression_manager
    mc_map = {}
    for a in problem.actions:
        if isinstance(a, InstantaneousMotionAction):
            mc_map[a] = a.motion_constraints
            for mc in a.motion_constraints:
                if mc.obstacles:
                    for mo, fe in mc.obstacles.items():
                        if mo.type == mc.movable.type:
                            a.add_precondition(em.Implies(em.Equals(mc.movable, mo), em.Equals(mc.starting, fe)))
    return mc_map
