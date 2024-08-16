from unified_planning.shortcuts import *
from tampest.motion_planning_data import SupportedTopologicalRefinement, SupportedPlanner
from benchmarks import get_problem
import argparse

env = get_environment()
env.factory.add_engine("tampest", "tampest.engine", "TampestEngine")
env.factory.add_meta_engine("tamp", "tampest.meta_engine", "TampMetaEngine")
env.credits_stream = None

def main():

    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--domain', required=True, type=str)
    parser.add_argument('--dim', required=True, type=str)
    parser.add_argument('--d', required=True, type=int)
    parser.add_argument('--c', required=True, type=int)
    parser.add_argument('--tp', required=True, type=str)
    parser.add_argument('--mp', required=True, type=str)
    parser.add_argument('--tr', required=True, type=str)
    parser.add_argument('--capacity', required=False, type=int)

    args, _ = parser.parse_known_args()

    domain = args.domain
    dim = args.dim
    d = args.d
    c = args.c
    tp = args.tp # ['fast-downward' 'enhsp' 'tamer' 'tampest']
    
    if args.mp == 'RRT':
        mp = SupportedPlanner.RRT
    elif args.mp =='LazyRRT':
        mp = SupportedPlanner.LazyRRT
    else:
        raise NotImplementedError
    
    if args.tr == 'none': # ['none' 'unreach' 'obs' 'all']
        tr = SupportedTopologicalRefinement.NONE
    elif args.tr =='unreach':
        tr = SupportedTopologicalRefinement.UNREACH
    elif args.tr =='obs':
        tr = SupportedTopologicalRefinement.OBS
    elif args.tr =='all':
        tr = SupportedTopologicalRefinement.ALL
    else:
        raise NotImplementedError

    robot_capacity = None
    if args.capacity:
        robot_capacity = args.capacity

    incremental = True
    step_horizon = 100
    interpolate=False 
    simplified=True 
    tolerance=0.0
    distance=None 
    
    mp_time = 5.0

    problem = get_problem(domain, dim, d, c, robot_capacity)

    if tp == "tampest":
        with OneshotPlanner(name="tampest", 
                            params={'incremental': incremental, 'step_horizon': step_horizon, 
                                    'motion_planning_time': mp_time, 'interpolate': interpolate, 
                                    'simplified': simplified, 'tolerance': tolerance, 'distance': distance, 
                                    'motion_planner': mp, 'topological_refinement': tr}) as planner:

            res = planner.solve(problem) 

    else:
        with OneshotPlanner(name=f"tamp[{tp}]",
                            params={
                                    'motion_planning_time': mp_time,
                                    'interpolate': interpolate, 'simplified': simplified,
                                    'tolerance': tolerance, 'distance': distance,
                                    'motion_planner': mp,
                                    'topological_refinement': tr}) as planner:  

            res = planner.solve(problem) 
            
    if res and res.plan:
        print("SOLUTION FOUND!")
        print(res)
    else:
        print("NO SOLUTION FOUND!")
        
               
if __name__ == '__main__':
    main()