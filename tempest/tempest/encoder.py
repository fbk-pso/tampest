import pysmt
import unified_planning as up

from functools import lru_cache
from unified_planning.plans import SequentialPlan, TimeTriggeredPlan
from unified_planning.model.fluent import get_all_fluent_exp
from tempest.converter import SMTConverter


class SymbolEncoder:
    def __init__(self, objects, pysmt_env=None):
        self.pysmt_env = pysmt_env if pysmt_env else pysmt.shortcuts.get_env()
        self.mgr = self.pysmt_env.formula_manager
        self.objects = objects
        self.type_constraints = {}
        self.c = 0

    @lru_cache(maxsize=None)
    def t(self, i):
        return self.mgr.Symbol(f"t@{i}", pysmt.shortcuts.REAL)

    @lru_cache(maxsize=None)
    def t_last(self):
        return self.mgr.Symbol(f"t@last", pysmt.shortcuts.REAL)

    @lru_cache(maxsize=None)
    def action(self, action, i):
        return self.mgr.Symbol(f"act_{action.name}@{i}")

    @lru_cache(maxsize=None)
    def chain_var(self, action, e, i, w):
        self.c += 1
        if action is None:
            return self.mgr.Symbol(f"problem_{self.c}@{i}-{w}")
        else:
            return self.mgr.Symbol(f"act_{action.name}_{self.c}@{i}-{w}")

    @lru_cache(maxsize=None)
    def duration(self, action, i):
        return self.mgr.Symbol(f"dur_{action.name}@{i}", pysmt.shortcuts.REAL)

    @lru_cache(maxsize=None)
    def fluent(self, fluent, args, i):
        smt_type, lb, ub = self.type_to_smt(fluent.type)
        args_str = f"_{'_'.join([str(a) for a in args])}"
        res = self.mgr.Symbol(f"fluent_{fluent.name}{args_str}@{i}", smt_type)
        self.add_type_constraints(res, fluent.type, lb, ub, i)
        return res

    @lru_cache(maxsize=None)
    def parameter(self, action, parameter, i):
        smt_type, lb, ub = self.type_to_smt(parameter.type)
        res = self.mgr.Symbol(f"parameter_{action.name}_{parameter.name}@{i}", smt_type)
        self.add_type_constraints(res, parameter.type, lb, ub, i)
        return res

    def add_type_constraints(self, symbol, type, lb, ub, i):
        self.type_constraints.setdefault(i, set())
        if type.is_user_type():
            l = []
            for p in range(lb, ub+1):
                l.append(self.mgr.Equals(symbol, self.mgr.Real(p)))
            self.type_constraints[i].add(self.mgr.Or(l))
        else:
            if lb is not None:
                self.type_constraints[i].add(self.mgr.GE(symbol, self.mgr.Real(lb)))
            if ub is not None:
                self.type_constraints[i].add(self.mgr.LE(symbol, self.mgr.Real(ub)))

    def type_to_smt(self, type):
        if type.is_bool_type():
            return pysmt.shortcuts.BOOL, None, None
        elif type.is_int_type():
            lb, ub = type.lower_bound, type.upper_bound
            return pysmt.shortcuts.REAL, lb, ub
        elif type.is_real_type():
            lb, ub = type.lower_bound, type.upper_bound
            return pysmt.shortcuts.REAL, lb, ub
        elif type.is_user_type():
            lb = None
            for obj, i in self.objects.items():
                if obj.type == type:
                    if lb is None:
                        lb = i
                    ub = i
            return pysmt.shortcuts.REAL, lb, ub
        else:
            raise NotImplementedError(f"Unknown type in conversion: {type}")


class ProblemEncoder:
    def __init__(self, problem, pysmt_env=None):
        self.problem = problem
        self.em = self.problem.environment.expression_manager
        self.pysmt_env = pysmt_env if pysmt_env else pysmt.shortcuts.get_env()
        self.mgr = self.pysmt_env.formula_manager
        self.converters = {}

        self.objects = {}
        self.map_back_objects = {}
        i = 0
        for ut in problem.user_types:
            for obj in problem.objects(ut):
                self.objects[obj] = i
                self.map_back_objects[i] = obj
                i += 1

        self.static_fluents = problem.get_static_fluents()

        self.symenc = SymbolEncoder(self.objects, self.pysmt_env)

        self.touchers = {}
        for a in self.problem.actions:
            if isinstance(a, up.model.InstantaneousAction):
                for e in a.effects:
                    self.touchers.setdefault(e.fluent.fluent(), []).append((a, None, e))
            elif isinstance(a, up.model.DurativeAction):
                for t, le in a.effects.items():
                    for e in le:
                        self.touchers.setdefault(e.fluent.fluent(), []).append(
                            (a, t, e)
                        )
        for t, le in self.problem.timed_effects.items():
            for e in le:
                self.touchers.setdefault(e.fluent.fluent(), []).append((None, t, e))

        self.mutexes = {}
        for a in self.problem.actions:
            if not isinstance(a, up.model.InstantaneousAction):
                continue
            mutex_a = []
            self.mutexes[a] = mutex_a
            for b in self.problem.actions:
                if not isinstance(b, up.model.InstantaneousAction):
                    continue
                if a != b and self.is_mutex(a, b):
                    mutex_a.append(b)

    def is_mutex(self, a, b):
        fve = self.problem.environment.free_vars_extractor

        a_p = set(x.fluent() for p in a.preconditions for x in fve.get(p))
        b_p = set(x.fluent() for p in b.preconditions for x in fve.get(p))
        a_e = set(e.fluent.fluent() for e in a.effects)
        b_e = set(e.fluent.fluent() for e in b.effects)

        return not (a_p.isdisjoint(b_e) and b_p.isdisjoint(a_e) and a_e.isdisjoint(b_e))

    def converter(self, i, w, scope):
        key = (None, i, w) if scope is None else (scope.name, i, w)
        try:
            return self.converters[key]
        except KeyError:
            c = SMTConverter(
                i=i,
                w=w,
                symenc=self.symenc,
                pysmt_env=self.pysmt_env,
                problem=self.problem,
                objects=self.objects,
                static_fluents=self.static_fluents,
                scope=scope,
            )
            self.converters[key] = c
            return c

    def to_smt(self, phi, i, w=None, scope=None):
        if w is None:
            w = i
        return self.converter(i, w, scope).to_smt(phi)

    def t(self, i):
        if i == 0:
            return self.mgr.Real(0)
        return self.symenc.t(i)

    def t_last(self):
        return self.symenc.t_last()

    def a(self, action, i):
        return self.symenc.action(action, i)

    def dur(self, action, i):
        return self.symenc.duration(action, i)

    def chain_var(self, action, e, i, w):
        return self.symenc.chain_var(action, e, i, w)

    def encode_tp(self, action, t, i):
        smt_t = self.t(i)
        smt_dur = self.dur(action, i)
        if t.is_from_start():
            if t.delay != 0:
                tp = self.mgr.Plus(smt_t, self.mgr.Real(t.delay))
            else:
                tp = smt_t
        else:
            if t.delay != 0:
                tp = self.mgr.Plus(smt_t, smt_dur, self.mgr.Real(t.delay))
            else:
                tp = self.mgr.Plus(smt_t, smt_dur)
        return tp

    def encode_problem_tp(self, t, h=None):
        if t.is_from_start():
            return self.mgr.Real(t.delay)
        else:
            assert t.delay == 0
            return self.t_last() if h is None else self.t(h - 1)

    def encode_effect(self, action, e, i, w):
        # The fluent is assigned in the current state (i)
        smt_f = self.to_smt(e.fluent, i, w, scope=action)
        # The value expression is evaluated in the previous state (i-1)
        smt_v = self.to_smt(e.value, i - 1, w, scope=action)
        # The condition is evaluated in the previous state
        smt_c = self.to_smt(e.condition, i - 1, w, scope=action)

        if e.is_assignment():
            pass
        elif e.is_increase():
            smt_v = self.mgr.Plus(self.to_smt(e.fluent, i - 1, w, scope=action), smt_v)
        elif e.is_decrease():
            smt_v = self.mgr.Minus(self.to_smt(e.fluent, i - 1, w, scope=action), smt_v)
        else:
            raise NotImplementedError

        return self.mgr.Implies(smt_c, self.mgr.EqualsOrIff(smt_f, smt_v))

    def encode_effects(self, action, t, le, i, w):
        if action is None:
            assert t.is_from_start()
            smt_tp = self.encode_problem_tp(t)
        else:
            smt_tp = self.encode_tp(action, t, w)
        eff = [self.mgr.Equals(self.t(i), smt_tp)]
        for e in le:
            eff.append(self.encode_effect(action, e, i, w))
        return self.mgr.And(eff)

    def encode_condition_or_goal(self, action, it, c, i, w, h=None):
        if action is None:
            smt_tp_1 = self.encode_problem_tp(it.lower, h)
            smt_tp_2 = self.encode_problem_tp(it.upper, h)
        else:
            smt_tp_1 = self.encode_tp(action, it.lower, w)
            smt_tp_2 = self.encode_tp(action, it.upper, w)
        if it.lower == it.upper:
            smt_tp = smt_tp_1
            cond = self.mgr.And(self.mgr.LT(self.t(i - 1), smt_tp), self.mgr.GE(self.t(i), smt_tp))
            formula = self.mgr.Implies(cond, self.to_smt(c, i - 1, w, scope=action))
        elif it.is_left_open():  # open and left open are the same
            cond_1 = self.mgr.And(self.mgr.LE(self.t(i - 1), smt_tp_1), self.mgr.GT(self.t(i), smt_tp_1))
            formula_1 = self.mgr.Implies(cond_1, self.to_smt(c, i - 1, w, scope=action))
            cond_2 = self.mgr.And(self.mgr.LT(self.t(i), smt_tp_2), self.mgr.GT(self.t(i), smt_tp_1))
            formula_2 = self.mgr.Implies(cond_2, self.to_smt(c, i - 1, w, scope=action))
            formula = self.mgr.And(formula_1, formula_2)
        else:  # closed and right open are the same
            cond_1 = self.mgr.And(self.mgr.LT(self.t(i - 1), smt_tp_1), self.mgr.GE(self.t(i), smt_tp_1))
            formula_1 = self.mgr.Implies(cond_1, self.to_smt(c, i - 1, w, scope=action))
            cond_2 = self.mgr.And(self.mgr.LT(self.t(i), smt_tp_2), self.mgr.GE(self.t(i), smt_tp_1))
            formula_2 = self.mgr.Implies(cond_2, self.to_smt(c, i - 1, w, scope=action))
            formula = self.mgr.And(formula_1, formula_2)
        return formula

    def encode_action_duration(self, action, i):
        smt_dur = self.dur(action, i)
        duration = action.duration
        smt_lower = self.to_smt(duration.lower, i - 1, i, scope=action)
        smt_upper = self.to_smt(duration.upper, i - 1, i, scope=action)
        l = []
        if duration.is_left_open():
            l.append(self.mgr.GT(smt_dur, smt_lower))
        else:
            l.append(self.mgr.GE(smt_dur, smt_lower))
        if duration.is_right_open():
            l.append(self.mgr.LT(smt_dur, smt_upper))
        else:
            l.append(self.mgr.LE(smt_dur, smt_upper))
        return self.mgr.And(l)

    def encode_incremental_durative_action(self, action, i):
        l = []
        temp_l = []

        conjunction = []
        conjunction.append(self.encode_action_duration(action, i))
        conjunction.append(
            self.mgr.LE(self.mgr.Plus(self.t(i), self.dur(action, i)), self.t_last())
        )

        # Action conditions
        for it, lc in action.conditions.items():
            c = self.em.And(lc)
            ev = (it, c)
            conjunction.append(self.chain_var(action, ev, i, i))
            for w in range(1, i + 1):
                temp_l.append(
                    self.mgr.Implies(
                        self.chain_var(action, ev, i + 1, w), self.mgr.Bool(True)
                    )
                )
                rhs = self.encode_tp(action, it.upper, w)
                l.append(
                    self.mgr.Implies(self.a(action, w), self.mgr.LE(rhs, self.t_last()))
                )
                formula = self.encode_condition_or_goal(action, it, c, i, w)
                l.append(
                    self.mgr.Implies(
                        self.chain_var(action, ev, i, w),
                        self.mgr.And(formula, self.chain_var(action, ev, i + 1, w)),
                    )
                )

        # Action effects
        for t, le in action.effects.items():
            ev = (t, tuple(le))
            conjunction.append(self.chain_var(action, ev, i, i))
            for w in range(1, i + 1):
                temp_l.append(
                    self.mgr.Implies(
                        self.chain_var(action, ev, i + 1, w), self.mgr.Bool(False)
                    )
                )
                formula = self.encode_effects(action, t, le, i, w)
                l.append(
                    self.mgr.Implies(
                        self.chain_var(action, ev, i, w),
                        self.mgr.Or(formula, self.chain_var(action, ev, i + 1, w)),
                    )
                )

        l.append(self.mgr.Implies(self.a(action, i), self.mgr.And(conjunction)))

        return self.mgr.And(l), self.mgr.And(temp_l)

    def encode_durative_action(self, action, i, h):
        l = []

        # Action duration constraints
        l.append(self.encode_action_duration(action, i))
        l.append(self.mgr.LE(self.mgr.Plus(self.t(i), self.dur(action, i)), self.t(h - 1)))

        # Action conditions
        for it, lc in action.conditions.items():
            c = self.em.And(lc)
            for k in range(i, h):
                l.append(self.encode_condition_or_goal(action, it, c, k, i))

        # Action effects
        for t, le in action.effects.items():
            if t.is_from_start() and t.delay == 0:
                l.append(self.encode_effects(action, t, le, i, i))
            else:
                l2 = []
                for k in range(i, h):
                    l2.append(self.encode_effects(action, t, le, k, i))
                l.append(self.mgr.Or(l2))

        return self.mgr.Implies(self.a(action, i), self.mgr.And(l))

    def encode_instantaneous_action(self, action, i):
        l = []
        smt_a = self.a(action, i)

        # Encode preconditions
        for p in action.preconditions:
            l.append(self.to_smt(p, i - 1, i, scope=action))

        # Encode effects
        for e in action.effects:
            l.append(self.encode_effect(action, e, i, i))

        return self.mgr.Implies(smt_a, self.mgr.And(l))

    def encode_frame_axiom(self, i):
        res = []
        for f in self.problem.fluents:
            if f in self.static_fluents:
                continue
            for f_exp in get_all_fluent_exp(self.problem, f):
                smt_f0 = self.symenc.fluent(f_exp.fluent(), f_exp.args, i - 1)
                smt_f1 = self.symenc.fluent(f_exp.fluent(), f_exp.args, i)
                eq = self.mgr.EqualsOrIff(smt_f0, smt_f1)
                cond = self.mgr.Not(
                    eq
                )  # The fluent changes its value between step i-1 and i
                disjunctions = []  # List of possible events that can change the value
                for a, t, e in self.touchers.get(f, []):
                    if a is None:  # Timed effect
                        conj = [self.mgr.Equals(self.t(i), self.encode_problem_tp(t))]
                        conj.append(self.to_smt(e.condition, i-1))
                        trivially_false = False
                        for z, p in enumerate(e.fluent.args):
                            p = self.to_smt(p, i, scope=a)
                            va = self.to_smt(f_exp.arg(z), 0)
                            if p.is_constant() and va.is_constant():
                                if p != va:
                                    trivially_false = True
                                    break
                                # Do not add trivially true conjuncts
                            else:
                                conj.append(self.mgr.Equals(p, va))
                        if not trivially_false:
                            disjunctions.append(self.mgr.And(conj))
                    elif t is None or (
                        t.is_from_start() and t.delay == 0
                    ):  # Effect at start
                        conj = [self.a(a, i)]
                        conj.append(self.to_smt(e.condition, i-1, i, scope=a))
                        trivially_false = False
                        for z, p in enumerate(e.fluent.args):
                            p = self.to_smt(p, i, scope=a)
                            va = self.to_smt(f_exp.arg(z), 0, scope=a)
                            if p.is_constant() and va.is_constant():
                                if p != va:
                                    trivially_false = True
                                    break
                                # Do not add trivially true conjuncts
                            else:
                                conj.append(self.mgr.Equals(p, va))
                        if not trivially_false:
                            disjunctions.append(self.mgr.And(conj))
                    else:
                        for w in range(1, i + 1):
                            conj = [self.a(a, w)]
                            conj.append(self.to_smt(e.condition, i-1, w, scope=a))
                            conj.append(
                                self.mgr.Equals(self.t(i), self.encode_tp(a, t, w))
                            )
                            trivially_false = False
                            for z, p in enumerate(e.fluent.args):
                                p = self.to_smt(p, w, scope=a)
                                va = self.to_smt(f_exp.arg(z), 0, scope=a)
                                if p.is_constant() and va.is_constant():
                                    if p != va:
                                        trivially_false = True
                                        break
                                    # Do not add trivially true conjuncts
                                else:
                                    conj.append(self.mgr.Equals(p, va))
                            if not trivially_false:
                                disjunctions.append(self.mgr.And(conj))
                if len(disjunctions) == 0:  # No events => no changes
                    res.append(eq)
                elif len(disjunctions) == 1:
                    res.append(self.mgr.Implies(cond, disjunctions[0]))
                else:
                    res.append(self.mgr.Implies(cond, self.mgr.Or(disjunctions)))
                    for x in range(len(disjunctions)):  # Only one is possible
                        for y in range(x + 1, len(disjunctions)):
                            res.append(
                                self.mgr.Not(
                                    self.mgr.And(disjunctions[x], disjunctions[y])
                                )
                            )
        return self.mgr.And(res)

    def encode_mutex_constraints(self, i):
        res = []
        if not self.problem.kind.has_continuous_time():
            for a, muts in self.mutexes.items():
                if muts:
                    res.append(
                        self.mgr.Implies(
                            self.a(a, i),
                            self.mgr.And(self.mgr.Not(self.a(b, i)) for b in muts),
                        )
                    )
        return self.mgr.And(res)

    def initial_state(self):
        res = []
        for f, v in self.problem.initial_values.items():
            smt_f = self.to_smt(f, 0)
            smt_v = self.to_smt(v, 0)
            res.append(self.mgr.EqualsOrIff(smt_f, smt_v))
        for c in self.symenc.type_constraints[0]:
            res.append(c)
        return self.mgr.And(res)

    def monolithic_bounded_planning(self, h):
        res = []

        # Encode fluents initial values
        res.append(self.initial_state())

        # Timed effects
        for t, le in self.problem.timed_effects.items():
            l = []
            for i in range(1, h):
                l.append(self.encode_effects(None, t, le, i, 0))
            res.append(self.mgr.Or(l))

        for i in range(1, h):
            # Time is non-negative and strictly increasing
            res.append(self.mgr.LT(self.t(i - 1), self.t(i)))

            # Encode actions
            for a in self.problem.actions:
                if isinstance(a, up.model.InstantaneousAction):
                    res.append(self.encode_instantaneous_action(a, i))
                elif isinstance(a, up.model.DurativeAction):
                    res.append(self.encode_durative_action(a, i, h))

            # Frame axiom
            res.append(self.encode_frame_axiom(i))

            # Mutex exclusions in non-temporal problems
            res.append(self.encode_mutex_constraints(i))

            # Add type constraints
            for c in self.symenc.type_constraints[i]:
                res.append(c)

        # Timed goals
        for it, lg in self.problem.timed_goals.items():
            g = self.em.And(lg)
            if it.lower == it.upper and it.lower.is_from_end() and it.lower.delay == 0:
                res.append(self.to_smt(g, h - 1))
            else:
                if it.lower.is_from_start() and it.lower.delay == 0:
                    res.append(self.to_smt(g, 0))
                for i in range(1, h):
                    res.append(self.encode_condition_or_goal(None, it, g, i, 0, h))
            res.append(self.mgr.LE(self.encode_problem_tp(it.upper), self.t(h - 1)))

        # Goals
        for g in self.problem.goals:
            res.append(self.to_smt(g, h - 1))

        return self.mgr.And(res)

    def incremental_step_zero(self):
        res = []

        # Encode fluents initial values
        res.append(self.initial_state())

        # Timed effects
        for t, le in self.problem.timed_effects.items():
            ev = (t, tuple(le))
            res.append(self.chain_var(None, ev, 1, 0))

        # Timed goals
        for it, lg in self.problem.timed_goals.items():
            g = self.em.And(lg)
            if it.lower.is_from_start() and it.lower.delay == 0:
                res.append(self.to_smt(g, 0, 0, scope=None))

        return self.mgr.And(res)

    def incremental_step(self, i):
        res = []
        temp_res = []

        # Time is non-negative and strictly increasing
        res.append(self.mgr.LT(self.t(i - 1), self.t(i)))

        temp_res.append(self.mgr.Equals(self.t_last(), self.t(i)))

        # Timed effect
        for t, le in self.problem.timed_effects.items():
            assert t.is_from_start()
            ev = (t, tuple(le))
            formula = self.encode_effects(None, t, le, i, 0)
            res.append(
                self.mgr.Implies(
                    self.chain_var(None, ev, i, 0),
                    self.mgr.Or(formula, self.chain_var(None, ev, i + 1, 0)),
                )
            )
            temp_res.append(
                self.mgr.Implies(
                    self.chain_var(None, ev, i + 1, 0), self.mgr.Bool(False)
                )
            )

        # Actions
        for a in self.problem.actions:
            if isinstance(a, up.model.InstantaneousAction):
                res.append(self.encode_instantaneous_action(a, i))
            elif isinstance(a, up.model.DurativeAction):
                f, tf = self.encode_incremental_durative_action(a, i)
                res.append(f)
                temp_res.append(tf)

        # Frame axiom
        res.append(self.encode_frame_axiom(i))

        # Mutex exclusions in non-temporal problems
        res.append(self.encode_mutex_constraints(i))

        # Add type constraints
        for c in self.symenc.type_constraints[i]:
            res.append(c)

        # Timed goals
        for it, lg in self.problem.timed_goals.items():
            g = self.em.And(lg)
            if it.lower == it.upper and it.lower.is_from_end() and it.lower.delay == 0:
                temp_res.append(self.to_smt(g, i, 0))
            else:
                res.append(self.encode_condition_or_goal(None, it, g, i, 0))
                res.append(self.mgr.LE(self.encode_problem_tp(it.upper), self.t_last()))

        # Goals
        temp_res.append(
            self.to_smt(self.em.And(self.problem.goals), i, 0, scope=None)
        )

        return self.mgr.And(res), self.mgr.And(temp_res)

    def extract_plan(self, model, h):
        res = []
        for i in range(h):
            t = model.get_py_value(self.t(i))
            for a in self.problem.actions:
                if model.get_py_value(self.a(a, i)):
                    d = (
                        model.get_py_value(self.dur(a, i))
                        if isinstance(a, up.model.DurativeAction)
                        else None
                    )
                    params = []
                    for p in a.parameters:
                        pv = model.get_py_value(self.symenc.parameter(a, p, i))
                        if p.type.is_user_type():
                            assert pv.denominator == 1
                            pv = self.map_back_objects[pv.numerator]
                        elif p.type.is_int_type():
                            assert pv.denominator == 1
                            pv = pv.numerator
                        params.append(pv)
                    res.append((t, a(*params), d))
        if self.problem.kind.has_continuous_time():
            return TimeTriggeredPlan(res)
        else:
            return SequentialPlan([ai for _, ai, _ in res])
