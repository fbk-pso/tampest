import unified_planning as up
import unified_planning.engines as engines
import unified_planning.model.walkers as walkers
from unified_planning.engines.mixins.compiler import CompilationKind, CompilerMixin
from unified_planning.engines.results import CompilerResult
from unified_planning.model.operators import OperatorKind
from unified_planning.model import Problem, ProblemKind, Fluent, Parameter, Object, InstantaneousAction
from unified_planning.model.walkers.identitydag import IdentityDagWalker
from typing import OrderedDict, Optional, Dict, List


operator_set = frozenset(op for op in OperatorKind if op != OperatorKind.FLUENT_EXP)

class Substituter(IdentityDagWalker):
    def __init__(
        self,
        fluents: Dict["Fluent", "Fluent"],
        subs: Dict["up.model.FNode", "up.model.FNode"],
        environment: "up.environment.Environment",
    ):
        IdentityDagWalker.__init__(self, environment, True)
        self.environment = environment
        self._fluents = fluents
        self._subs = subs

    def substitute(self, expression: "up.model.FNode") -> "up.model.FNode":
        return self.walk(expression)

    def walk_fluent_exp(
        self,
        expression: "up.model.FNode",
        args: List["up.model.FNode"],
        **kwargs,
    ) -> "up.model.FNode":
        fluent = self._fluents.get(expression.fluent(), expression.fluent())
        return self.manager.FluentExp(fluent, args)

    @walkers.handles(operator_set)
    def walk_replace_or_identity(
        self,
        expression: "up.model.FNode",
        args: List["up.model.FNode"],
        **kwargs,
    ) -> "up.model.FNode":
        res = self._subs.get(expression, None)
        if res is not None:
            return res
        else:
            return IdentityDagWalker.super(self, expression, args, **kwargs)


class UserTypesRemover(engines.engine.Engine, CompilerMixin):

    def __init__(self):
        engines.engine.Engine.__init__(self)
        CompilerMixin.__init__(self, CompilationKind.USERTYPE_FLUENTS_REMOVING) # TO FIX

    @property
    def name(self):
        return "types-remover"

    @staticmethod
    def supported_kind() -> ProblemKind:
        supported_kind = ProblemKind(version=2)
        supported_kind.set_problem_class("TAMP")
        supported_kind.set_problem_class("ACTION_BASED")
        supported_kind.set_typing("FLAT_TYPING")
        supported_kind.set_problem_type("SIMPLE_NUMERIC_PLANNING")
        supported_kind.set_problem_type("GENERAL_NUMERIC_PLANNING")
        supported_kind.set_fluents_type("INT_FLUENTS")
        supported_kind.set_fluents_type("REAL_FLUENTS")
        supported_kind.set_numbers("BOUNDED_TYPES")
        supported_kind.set_conditions_kind("NEGATIVE_CONDITIONS")
        supported_kind.set_conditions_kind("DISJUNCTIVE_CONDITIONS")
        supported_kind.set_conditions_kind("EQUALITIES")
        # supported_kind.set_conditions_kind("EXISTENTIAL_CONDITIONS")
        # supported_kind.set_conditions_kind("UNIVERSAL_CONDITIONS")
        supported_kind.set_effects_kind("CONDITIONAL_EFFECTS")
        supported_kind.set_effects_kind("INCREASE_EFFECTS")
        supported_kind.set_effects_kind("DECREASE_EFFECTS")
        supported_kind.set_effects_kind("STATIC_FLUENTS_IN_BOOLEAN_ASSIGNMENTS")
        supported_kind.set_effects_kind("STATIC_FLUENTS_IN_NUMERIC_ASSIGNMENTS")
        supported_kind.set_effects_kind("FLUENTS_IN_BOOLEAN_ASSIGNMENTS")
        supported_kind.set_effects_kind("FLUENTS_IN_NUMERIC_ASSIGNMENTS")
        # supported_kind.set_effects_kind("FORALL_EFFECTS")
        return supported_kind

    @staticmethod
    def supports(problem_kind):
        return problem_kind <= UserTypesRemover.supported_kind()

    @staticmethod
    def supports_compilation(compilation_kind: CompilationKind) -> bool:
        return True # TO FIX

    @staticmethod
    def resulting_problem_kind(
        problem_kind: ProblemKind, compilation_kind: Optional[CompilationKind] = None
    ) -> ProblemKind:
        raise NotImplementedError

    def _compile(
        self,
        problem: "up.model.AbstractProblem",
        compilation_kind: "up.engines.CompilationKind",
    ) -> CompilerResult:
        assert isinstance(problem, Problem)
        env = problem.environment
        em = env.expression_manager
        tm = env.type_manager
        new_problem = Problem(problem.name)

        ObjectType = tm.UserType("object")

        subs = {}
        new_fluents = {}
        for ut in problem.user_types:
            new_f = Fluent(f"is_{ut.name}", p=ObjectType)
            new_fluents[ut] = new_f
            new_problem.add_fluent(new_f, default_initial_value=False)
            for o in problem.objects(ut):
                new_o = Object(o.name, ObjectType)
                subs[em.ObjectExp(o)] = em.ObjectExp(new_o)
                new_problem.add_object(new_o)
                new_problem.set_initial_value(new_f(new_o), True)

        fluents_subs = {}
        for f in problem.fluents:
            params = []
            for p in f.signature:
                params.append(Parameter(p.name, ObjectType))
            new_f = Fluent(f.name, f.type, params)
            fluents_subs[f] = new_f
            new_problem.add_fluent(new_f, default_initial_value=problem.fluents_defaults.get(f, None))

        for a in problem.actions:
            assert isinstance(a, InstantaneousAction)
            params = OrderedDict()
            for p in a.parameters:
                params[p.name] = ObjectType
            new_a = InstantaneousAction(a.name, params)
            a_subs = dict(subs)
            for p in a.parameters:
                new_p = new_a.parameter(p.name)
                a_subs[em.ParameterExp(p)] = em.ParameterExp(new_p)
                new_a.add_precondition(new_fluents[p.type](new_p))
            substituter = Substituter(fluents_subs, a_subs, env)
            for c in a.preconditions:
                new_a.add_precondition(substituter.substitute(c))
            for e in a.effects:
                ef = substituter.substitute(e.fluent)
                ev = substituter.substitute(e.value)
                ec = substituter.substitute(e.condition)
                if e.is_increase():
                    new_a.add_increase_effect(ef, ev, ec)
                elif e.is_decrease():
                    new_a.add_decrease_effect(ef, ev, ec)
                else:
                    new_a.add_effect(ef, ev, ec)
            new_problem.add_action(new_a)

        substituter = Substituter(fluents_subs, subs, env)
        for k, v in problem.explicit_initial_values.items():
            new_problem.set_initial_value(substituter.substitute(k), substituter.substitute(v))

        for g in problem.goals:
            new_problem.add_goal(substituter.substitute(g))

        def replace_action(
            action_instance: up.plans.ActionInstance,
        ) -> Optional[up.plans.ActionInstance]:
            new_a = new_problem.action(action_instance.action.name)
            params = []
            for p in action_instance.actual_parameters:
                params.append(substituter.substitute(p))
            return up.plans.ActionInstance(new_a, tuple(params))

        return CompilerResult(
            new_problem, replace_action, self.name
        )
