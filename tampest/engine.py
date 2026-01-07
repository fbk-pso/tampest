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

import pysmt
import warnings
import unified_planning as up

from fractions import Fraction
from functools import partial
from typing import IO, Callable, Optional

from unified_planning.model import ProblemKind
from unified_planning.engines import PlanGenerationResultStatus
from unified_planning.engines import PlanGenerationResult
from unified_planning.engines.compilers.utils import replace_action
from unified_planning.model.tamp.action import (
    InstantaneousMotionAction,
    DurativeMotionAction,
)

from tempest.encoders import IncrementalEncoder, MonolithicEncoder
from tempest.engine import TempestEngine

from tampest.motion.motion_planning_data import (
    SupportedTopologicalRefinement,
    SupportedPlanner,
)
from tampest.utils import metrics_from_planning_data
from tampest.check_plan import check_plan


EPSILON = 0.1
TOLERANCE = 0.1


class TampestEngine(up.engines.Engine, up.engines.mixins.OneshotPlannerMixin):
    """Implementation of the TAMPEST Engine."""

    def __init__(
        self,
        incremental=True,
        step_horizon=None,
        motion_planning_time=3.0,
        interpolate=False,
        simplified=False,
        tolerance=0.0,
        distance=None,
        motion_planner=SupportedPlanner.RRT,
        topological_refinement=SupportedTopologicalRefinement.ALL,
        max_radius_bound=False,
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
        self._max_radius_bound = max_radius_bound

    @property
    def name(self) -> str:
        return "TAMPEST"

    @staticmethod
    def supported_kind() -> ProblemKind:
        supported_kind = TempestEngine.supported_kind()
        supported_kind.set_problem_class("TAMP")
        return supported_kind

    @staticmethod
    def supports(problem_kind: "up.model.ProblemKind") -> bool:
        return problem_kind <= TampestEngine.supported_kind()

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
        if heuristic is not None:
            warnings.warn("TemPEST does not support custom heuristics.", UserWarning)

        orig_problem = problem
        em = problem.environment.expression_manager
        pysmt_env = pysmt.environment.Environment()
        motion_planning_time = self._motion_planning_time
        cache = {}

        motion_planning_data = {}

        while True:
            problem = orig_problem.clone()
            new_to_old = {}
            for a in problem.actions:
                orig_a = orig_problem.action(a.name)
                if isinstance(a, InstantaneousMotionAction):
                    for mc in a.motion_constraints:
                        if mc.static_obstacles:
                            for mo, fe in mc.static_obstacles.items():
                                if mo.type == mc.movable.type:
                                    a.add_precondition(
                                        em.Implies(
                                            em.Equals(mc.movable, mo),
                                            em.Equals(mc.starting, fe),
                                        )
                                    )

                if isinstance(a, DurativeMotionAction):
                    for t, mcl in a.timed_motion_constraints.items():
                        for mc in mcl:
                            if mc.static_obstacles:
                                for mo, fe in mc.static_obstacles.items():
                                    if mo.type == mc.movable.type:
                                        a.add_condition(
                                            t,
                                            em.Implies(
                                                em.Equals(mc.movable, mo),
                                                em.Equals(mc.starting, fe),
                                            ),
                                        )

                new_to_old[a] = orig_a

            modify_horizon = lambda x: x
            if self._incremental:
                encoder = IncrementalEncoder(
                    problem, pysmt_env=pysmt_env, epsilon=EPSILON
                )
                modify_horizon = lambda x: x - 1
            else:
                encoder = MonolithicEncoder(
                    problem, pysmt_env=pysmt_env, epsilon=EPSILON
                )

            with pysmt_env.factory.Solver(logic="QF_LRA") as smt:
                step_zero = encoder.encode_step_zero()
                if step_zero is not None:
                    smt.add_assertion(step_zero)
                h = 2
                old_conds = []
                old_updated_durations = []
                while self._step_horizon is None or h <= self._step_horizon:
                    formula, assumptions = encoder.encode_step(modify_horizon(h))
                    if formula is not None:
                        smt.add_assertion(formula)
                    for a, c in old_conds:
                        i = h - 1
                        for i in range(h - 1 if self._incremental else 1, h):
                            smt_a = encoder.a(a, i)
                            smt_c = encoder.to_smt(c, i - 1, i, scope=a)
                            f = encoder.mgr.Implies(smt_a, smt_c)
                            smt.add_assertion(f)
                    for updated_durations in old_updated_durations:
                        smt.add_assertion(
                            _encode_temporal_refinement(encoder, updated_durations, h)
                        )
                    while smt.solve(assumptions):
                        plan = encoder.extract_plan(smt.get_model(), h)
                        is_valid, updated_durations, new_conds = check_plan(
                            cache,
                            problem,
                            plan,
                            motion_planning_time,
                            self._interpolate,
                            self._simplified,
                            self._tolerance,
                            self._distance,
                            self._motion_planner,
                            self._topological_refinement,
                            self._max_radius_bound,
                            motion_planning_data,
                        )
                        print(f"Updated durations: {updated_durations}")
                        if is_valid:
                            return PlanGenerationResult(
                                PlanGenerationResultStatus.SOLVED_SATISFICING,
                                plan.replace_action_instances(
                                    partial(replace_action, map=new_to_old)
                                ),
                                self.name,
                                metrics=metrics_from_planning_data(
                                    motion_planning_data
                                ),
                            )
                        else:
                            if updated_durations:
                                old_updated_durations.append(updated_durations)
                                smt.add_assertion(
                                    _encode_temporal_refinement(
                                        encoder, updated_durations, h
                                    )
                                )
                            else:
                                for a, c in new_conds:
                                    if output_stream is not None:
                                        output_stream.write(f"{a.name} {c}\n")
                                    for i in range(1, h):
                                        smt_c = encoder.to_smt(c, i - 1, i, scope=a)
                                        f = encoder.mgr.Implies(encoder.a(a, i), smt_c)
                                        smt.add_assertion(f)
                                old_conds.extend(new_conds)
                    if output_stream is not None:
                        output_stream.write(f"No solution with bound {h}\n")
                    h += 1
            motion_planning_time *= 2


def _encode_temporal_refinement(enc, updated_durations, h):
    ev_all = []
    ev_start = []
    for ai, (start, dur, _, _) in updated_durations.items():
        ev_all.append((start, ai, True))
        ev_start.append((start, ai))
        ev_all.append((start + dur, ai, False))
    ev_all.sort(key=lambda x: x[0])
    ev_start.sort(key=lambda x: x[0])
    start_zero = ev_all[0][0]
    formulas = []
    for t in _nested_tuples(len(ev_start), h):
        f = []
        f2 = []
        f3 = []
        step = {}
        for i, j in enumerate(t):
            _, ai = ev_start[i]
            step[ai] = j
            start, dur, wait, new_dur = updated_durations[ai]
            if new_dur + wait > dur:
                f3.append(
                    enc.mgr.GE(
                        enc.dur(ai.action, j),
                        enc.mgr.Real(Fraction(new_dur + wait + TOLERANCE)),
                    )
                )
            if j != t[0]:
                f2.append(
                    enc.mgr.GE(
                        enc.mgr.Minus(enc.t(j), enc.t(t[0])),
                        enc.mgr.Real(Fraction(start - start_zero - EPSILON)),
                    )
                )
            params_f = enc.em.And(
                [
                    enc.em.Equals(p, ap)
                    for p, ap in zip(ai.actual_parameters, ai.action.parameters)
                ]
            )
            enc_ai_j = enc.mgr.And(
                enc.a(ai.action, j), enc.to_smt(params_f, j, j, scope=ai.action)
            )
            f.append(enc_ai_j)
        ord = []
        for _, ai, is_start in ev_all:
            if is_start:
                ord.append(enc.t(step[ai]))
            else:
                ord.append(enc.mgr.Plus(enc.t(step[ai]), enc.dur(ai.action, step[ai])))
        for i in range(len(ord) - 1):
            f2.append(enc.mgr.LE(ord[i], ord[i + 1]))
        formulas.append(
            enc.mgr.Implies(
                enc.mgr.And(f), enc.mgr.Implies(enc.mgr.And(f2), enc.mgr.And(f3))
            )
        )
    return enc.mgr.And(formulas)


def _generate_tuples(current_level, max_level, start_values, result, upper_limit):
    if current_level == max_level:
        result.append(tuple(start_values))
        return
    start = start_values[-1] if start_values else 1
    for i in range(start, upper_limit):
        _generate_tuples(
            current_level + 1, max_level, start_values + [i], result, upper_limit
        )


def _nested_tuples(n, upper_limit):
    result = []
    _generate_tuples(0, n, [], result, upper_limit)
    return result
