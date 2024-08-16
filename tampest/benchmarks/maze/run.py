import argparse
from unified_planning.shortcuts import *
import yaml
from tampest.benchmarks.maze.problem import Maze
from tampest.motion_planning_data import SupportedPlanner, SupportedTopologicalRefinement
from tampest.utils import plot_plan

env = get_environment()
env.factory.add_engine("tampest", "tampest.engine", "TampestEngine")
env.factory.add_meta_engine("tamp", "tampest.meta_engine", "TampMetaEngine")
env.credits_stream = None

def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--dim', required=True, type=str)
    parser.add_argument('--domain', required=False, type=str)
    parser.add_argument('--problem', required=False, type=str)
    parser.add_argument('--planner', required=True, type=str)
    parser.add_argument('--d', required=False, type=int)
    parser.add_argument('--c', required=False, type=int)
    args, _ = parser.parse_known_args()

    dim = args.dim

    domain_file= None 
    problem_file= None
    d= None
    c = None

    if args.domain:
        domain_file = args.domain

    if args.problem:
        problem_file = args.problem

    if args.d >=0:
        d = args.d 
    
    if args.c >=0:
        c = args.c
  
    with open(args.planner, "r") as stream:
        try:
            planner_file = yaml.safe_load(stream)
            tp = planner_file['task_planner'] # ['fast-downward' 'enhsp' 'tamer' 'tampest']

            if planner_file['motion_planner'] == 'RRT':
                mp = SupportedPlanner.RRT
            elif planner_file['motion_planner'] =='LazyRRT':
                mp = SupportedPlanner.LazyRRT
            else:
                raise NotImplementedError
    
            if planner_file['topological_refinement'] == 'none': # ['none' 'unreach' 'obs' 'all']
                tr = SupportedTopologicalRefinement.NONE
            elif planner_file['topological_refinement'] =='unreach':
                tr = SupportedTopologicalRefinement.UNREACH
            elif planner_file['topological_refinement'] =='obs':
                tr = SupportedTopologicalRefinement.OBS
            elif planner_file['topological_refinement'] =='all':
                tr = SupportedTopologicalRefinement.ALL
            else:
                raise NotImplementedError

            incremental = planner_file['incremental']
            step_horizon = planner_file['step_horizon']
            interpolate = planner_file['interpolate'] 
            simplified = planner_file['simplified'] 
            tolerance = planner_file['tolerance']
            distance = planner_file['distance'] 
            mp_time = planner_file['motion_planning_time']

        except yaml.YAMLError as exc:
                print(exc)


    if domain_file is not None and problem_file is not None:
        problem = Maze().get_problem_from_config(dim, domain_file, problem_file)
    elif d>=0 and c>=0:
        problem = Maze().get_problem(dim, d, c)
        print(problem)
    else:
        raise ValueError

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
        plot_plan(problem.all_objects, res)

    else:
        print("NO SOLUTION FOUND!")


if __name__ == '__main__':
    main()