import pysmt
import warnings
import unified_planning as up

from unified_planning.model import ProblemKind
from unified_planning.engines import (
    PlanGenerationResultStatus,
    PlanGenerationResult,
    Credits,
)
from unified_planning.engines.results import correct_plan_generation_result
from unified_planning.plans import TimeTriggeredPlan
from typing import IO, Callable, Optional
from pysmt.shortcuts import Solver
from tempest.encoder import ProblemEncoder


credits = Credits(
    "TemPEST",
    "FBK PSO Unit",
    "tamer@fbk.eu",
    "https://tamer.fbk.eu",
    "Free for Educational Use",
    "Temporal Planning Encoding into Satisfiability Testing",
    "Temporal Planning Encoding into Satisfiability Testing",
)


class TempestEngine(up.engines.Engine, up.engines.mixins.OneshotPlannerMixin):
    """Implementation of the TemPEST Engine."""

    def __init__(self, incremental=True, horizon=None):
        up.engines.Engine.__init__(self)
        up.engines.mixins.OneshotPlannerMixin.__init__(self)
        self.horizon = horizon
        self.incremental = incremental

    @property
    def name(self) -> str:
        return "TemPEST"

    @staticmethod
    def supported_kind() -> ProblemKind:
        supported_kind = ProblemKind()
        supported_kind.set_problem_class("ACTION_BASED")
        supported_kind.set_problem_type("SIMPLE_NUMERIC_PLANNING")
        supported_kind.set_problem_type("GENERAL_NUMERIC_PLANNING")
        supported_kind.set_time("CONTINUOUS_TIME")
        supported_kind.set_time("INTERMEDIATE_CONDITIONS_AND_EFFECTS")
        supported_kind.set_time("TIMED_EFFECTS")
        supported_kind.set_time("TIMED_GOALS")
        supported_kind.set_time("DURATION_INEQUALITIES")
        supported_kind.set_time("SELF_OVERLAPPING")
        supported_kind.set_expression_duration("STATIC_FLUENTS_IN_DURATIONS")
        supported_kind.set_expression_duration("FLUENTS_IN_DURATIONS")
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
        return problem_kind <= TempestEngine.supported_kind()

    @staticmethod
    def satisfies(optimality_guarantee: "up.engines.OptimalityGuarantee") -> bool:
        return False

    @staticmethod
    def get_credits(**kwargs) -> Optional["up.engines.Credits"]:
        return credits

    def _solve(
        self,
        problem: "up.model.AbstractProblem",
        heuristic: Optional[Callable[["up.model.state.State"], Optional[float]]] = None,
        timeout: Optional[float] = None,
        output_stream: Optional[IO[str]] = None,
    ) -> "up.engines.results.PlanGenerationResult":
        assert isinstance(problem, up.model.Problem)
        if timeout is not None:
            warnings.warn("TemPEST does not support timeout.", UserWarning)
        if output_stream is not None:
            warnings.warn("TemPEST does not support output stream.", UserWarning)
        if heuristic is not None:
            warnings.warn("TemPEST does not support custom heuristics.", UserWarning)

        pysmt_env = pysmt.shortcuts.get_env()

        enc = ProblemEncoder(problem, pysmt_env=pysmt_env)

        if self.incremental:
            with Solver(logic="QF_LRA") as smt:
                smt.add_assertion(enc.incremental_step_zero())
                h = 2
                while self.horizon is None or h <= self.horizon:
                    formula, temp_formula = enc.incremental_step(h-1)
                    smt.add_assertion(formula)
                    smt.push()
                    smt.add_assertion(temp_formula)
                    if smt.solve():
                        plan = enc.extract_plan(smt.get_model(), h)
                        res = PlanGenerationResult(
                            PlanGenerationResultStatus.SOLVED_SATISFICING,
                            plan,
                            self.name,
                        )
                        if isinstance(plan, TimeTriggeredPlan):
                            return correct_plan_generation_result(res, problem, None)
                        else:
                            return res
                    else:
                        smt.pop()
                        print(f"No solution with bound {h}")
                        h += 1
        else:
            h = 2
            while self.horizon is None or h <= self.horizon:
                formula = enc.monolithic_bounded_planning(h)
                with Solver(logic="QF_LRA") as smt:
                    smt.add_assertion(formula)
                    if smt.solve():
                        plan = enc.extract_plan(smt.get_model(), h)
                        res = PlanGenerationResult(
                            PlanGenerationResultStatus.SOLVED_SATISFICING,
                            plan,
                            self.name,
                        )
                        if isinstance(plan, TimeTriggeredPlan):
                            return correct_plan_generation_result(res, problem, None)
                        else:
                            return res
                    else:
                        print(f"No solution with bound {h}")
                        h += 1

        return PlanGenerationResult(
            PlanGenerationResultStatus.UNSOLVABLE_INCOMPLETELY, None, self.name
        )
