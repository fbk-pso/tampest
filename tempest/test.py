from unified_planning.shortcuts import *
from unified_planning.test.examples import get_example_problems

env = get_environment()
env.factory.add_engine("tempest", "tempest.engine", "TempestEngine")
env.credits_stream = None

pbs = get_example_problems()
problem = pbs["matchcellar"].problem
# print(problem)

with OneshotPlanner(name="tempest", params={'incremental': False}) as p:
    res = p.solve(problem)
    print(res)

    if res.plan:
        with PlanValidator(problem_kind=problem.kind) as v:
            check = v.validate(problem, res.plan)
            print(check)
