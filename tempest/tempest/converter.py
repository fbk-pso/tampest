from typing import List
import unified_planning as up
import unified_planning.model.walkers as walkers
from unified_planning.model.fnode import FNode
from unified_planning.model.fluent import get_all_fluent_exp


class SMTConverter(walkers.dag.DagWalker):
    def __init__(self, i, w, symenc, pysmt_env, problem, objects, static_fluents, scope):
        walkers.dag.DagWalker.__init__(self)
        self.pysmt_env = pysmt_env
        self.i = i
        self.w = w
        self.symenc = symenc
        self.manager = pysmt_env.formula_manager
        self.problem = problem
        self.static_fluents = static_fluents
        self.objects = objects
        self.scope = scope

    def to_smt(self, expression):
        return self.walk(expression)

    def constant_to_smt(self, expression):
        if expression.is_object_exp():
            va = self.manager.Real(self.objects[expression.object()])
        elif expression.is_bool_constant():
            va = self.manager.Bool(expression.bool_constant_value())
        elif (
                expression.is_real_constant()
                or expression.is_int_constant()
        ):
            va = self.manager.Real(expression.constant_value())
        else:
            raise NotImplementedError
        return va

    def walk_fluent_exp(self, expression, args):
        f = expression.fluent()
        is_static = False
        if f in self.static_fluents:
            is_static = True
        if len(args) == 0 and is_static:
            return self.constant_to_smt(self.problem.initial_value(expression))
        elif len(args) == 0 and not is_static:
            return self.symenc.fluent(expression.fluent(), tuple(), self.i)
        else:
            res = None
            for f_exp in get_all_fluent_exp(self.problem, expression.fluent()):
                if is_static:
                    then = self.constant_to_smt(self.problem.initial_value(f_exp))
                else:
                    then = self.symenc.fluent(f_exp.fluent(), f_exp.args, self.i)
                if res is None:
                    res = then
                else:
                    trivially_false = False
                    trivially_true = True
                    conds = []
                    for j, p in enumerate(args):
                        va = self.constant_to_smt(f_exp.arg(j))
                        if p.is_constant() and va.is_constant():
                            if p != va:
                                trivially_false = True
                                trivially_true = False
                                break
                        else:
                            trivially_true = False
                            conds.append(self.manager.Equals(p, va))
                    if trivially_true:
                        return then
                    if not trivially_false:
                        res = self.manager.Ite(self.manager.And(conds), then, res)
            return res

    def walk_bool_constant(self, expression, args, **kwargs):
        return self.manager.Bool(expression.bool_constant_value())

    def walk_int_constant(self, expression, args, **kwargs):
        return self.manager.Real(expression.int_constant_value())

    def walk_real_constant(self, expression, args, **kwargs):
        return self.manager.Real(expression.real_constant_value())

    def walk_param_exp(self, expression, args, **kwargs):
        assert self.scope is not None
        return self.symenc.parameter(self.scope, expression.parameter(), self.w)

    def walk_object_exp(self, expression, args, **kwargs):
        return self.manager.Real(self.objects[expression.object()])

    def walk_and(self, expression, args):
        return self.manager.And(args)

    def walk_or(self, expression, args):
        return self.manager.Or(args)

    def walk_not(self, expression, args):
        return self.manager.Not(args[0])

    def walk_iff(self, expression, args):
        return self.manager.Iff(*args)

    def walk_implies(self, expression, args):
        return self.manager.Implies(*args)

    def walk_equals(self, expression, args):
        return self.manager.Equals(*args)

    def walk_le(self, expression, args):
        return self.manager.LE(*args)

    def walk_lt(self, expression, args):
        return self.manager.LT(*args)

    def walk_plus(self, expression, args):
        return self.manager.Plus(args)

    def walk_minus(self, expression, args):
        return self.manager.Minus(*args)

    def walk_times(self, expression, args):
        return self.manager.Times(args)

    def walk_div(self, expression, args):
        return self.manager.Div(*args)
