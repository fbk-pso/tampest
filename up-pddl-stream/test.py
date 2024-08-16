from unified_planning.shortcuts import *
from up_pddl_stream.engine import SupportedAlgorithm
from tampest.motion_planning_data import SupportedPlanner
from benchmarks import get_problem
import argparse

env = get_environment()
env.factory.add_engine("pddlstream", "up_pddl_stream.engine", "PDDLStreamEngine")
env.credits_stream = None

def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--domain', required=True, type=str)
    parser.add_argument('--dim', required=True, type=str)
    parser.add_argument('--d', required=True, type=int)
    parser.add_argument('--c', required=True, type=int)
    parser.add_argument('--tp', required=True, type=str)
    parser.add_argument('--mp', required=True, type=str)
    parser.add_argument('--mp_time', required=False, type=float)
    
    args, _ = parser.parse_known_args()
    
    domain = args.domain

    if domain == "delivery":
        print("Unable to solve problems with numeric fluents.")
        return

    dim = args.dim
    d = args.d
    c = args.c

    if args.tp == 'incremental':
        tp = SupportedAlgorithm.INCREMENTAL
    elif args.tp =='focused':
        tp = SupportedAlgorithm.FOCUSED
    elif args.tp =='binding':
        tp = SupportedAlgorithm.BINDING
    elif args.tp =='adaptive':
        tp = SupportedAlgorithm.ADAPTIVE
    else:
        raise NotImplementedError
    
    if args.mp == 'RRT':
        mp = SupportedPlanner.RRT
    elif args.mp =='LazyRRT':
        mp = SupportedPlanner.LazyRRT
    else:
        raise NotImplementedError

    interpolate=False 
    simplified=False 
    tolerance=0.0
    distance=None 

    if args.mp_time:
        mp_time = args.mp_time
    else:
        mp_time = 5.0
 
    problem = get_problem(domain, dim, d, c)
    print(problem)

    with OneshotPlanner(name="pddlstream", 
                        params={'pddl_stream_algorithm': tp, 
                                'motion_planning_time': mp_time,
                                'interpolate': interpolate,
                                'simplified': simplified,
                                'tolerance': tolerance,
                                'distance': distance,
                                'motion_planner': mp}) as planner:
        
        res = planner.solve(problem)

    if res and res.plan:
        print("SOLUTION FOUND!")
        print(res)
    else:
        print("NO SOLUTION FOUND!")
               
if __name__ == '__main__':
    main()