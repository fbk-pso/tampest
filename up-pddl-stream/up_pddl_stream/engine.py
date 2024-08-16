from enum import Enum
import sys
import os

PATH = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(os.path.dirname(PATH), "pddlstream"))

from pddlstream.utils import INF
from up_pddl_stream.ut_compiler import UserTypesRemover

import warnings
import unified_planning as up
from unified_planning.model import ProblemKind, Fluent, Parameter
from unified_planning.engines import PlanGenerationResultStatus
from unified_planning.engines import PlanGenerationResult
from unified_planning.model.tamp.action import InstantaneousMotionAction, InstantaneousAction, MotionConstraint, Waypoints
from unified_planning.model.tamp.path import Path, ReedsSheppPath, SE2Path, SE3Path
from unified_planning.shortcuts import BoolType, MotionConstraint, FluentExp, Equals, Not, UserType, ObjectExp, MotionModels, MovableObject
from typing import IO, Callable, List, Optional, OrderedDict, Dict

from pddlstream.algorithms.meta import solve
from pddlstream.language.generator import from_fn
from pddlstream.language.constants import Action, And, PDDLProblem, print_solution

from unified_planning.io import PDDLWriter
from unified_planning.plans import SequentialPlan
from unified_planning.engines.compilers.usertype_fluents_remover import UsertypeFluentsRemover

from tampest.motion_planner import MotionPlanner
from tampest.motion_planning_data import SupportedPlanner

import time

# FOCUSED_ALGORITHMS = ['focused', 'binding', 'adaptive']
# ALGORITHMS = ['incremental'] + FOCUSED_ALGORITHMS
# DEFAULT_ALGORITHM = 'adaptive'

class SupportedAlgorithm(str, Enum):
    INCREMENTAL = 'incremental'
    FOCUSED = 'focused'
    BINDING = 'binding'
    ADAPTIVE = 'adaptive'


class PDDLStreamEngine(up.engines.Engine, up.engines.mixins.OneshotPlannerMixin):
    """Implementation of the PDDLStream Engine."""

    def __init__(
            self, 
            pddl_stream_algorithm=SupportedAlgorithm.INCREMENTAL, pddl_stream_unit_costs=True, pddl_stream_debug=False, pddl_stream_visualize=False,
            motion_planning_time=3.0,
            interpolate=False, simplified=False, tolerance=0.0,
            distance=None, motion_planner=SupportedPlanner.RRT, memout = INF
    ):
        up.engines.Engine.__init__(self)
        up.engines.mixins.OneshotPlannerMixin.__init__(self)
        self._pddl_stream_algorithm=pddl_stream_algorithm 
        self._pddl_stream_unit_costs = pddl_stream_unit_costs 
        self._pddl_stream_debug=pddl_stream_debug 
        self._pddl_stream_visualize=pddl_stream_visualize
        self._motion_planning_time = motion_planning_time
        self._interpolate = interpolate
        self._simplified = simplified
        self._tolerance = tolerance
        self._distance = distance
        self._motion_planner = motion_planner
        self._memout = memout


    @property
    def name(self) -> str:
        return "pddlstream"

    @staticmethod
    def supported_kind() -> ProblemKind:
        supported_kind = ProblemKind()
        supported_kind.set_problem_class("TAMP")
        supported_kind.set_fluents_type("OBJECT_FLUENTS")
        return supported_kind

    @staticmethod
    def supports(problem_kind: "up.model.ProblemKind") -> bool:
        return problem_kind <= PDDLStreamEngine.supported_kind()

    def _mc_to_bool_fluent(self, name:str, mc: MotionConstraint, problem, mov_objs_fluents):
        params = []
        params.append(mc.movable.parameter())
        params.append(mc.starting.parameter())
        waypoint = mc.waypoints[0]
        params.append(waypoint.parameter())
        new_preconditions = []
        added = []
        mc_fluents = {}
        compatible_configs = []
        if mc.obstacles:
            for obs, config in mc.obstacles.items(): ###################################################### to check

                if problem.has_object(obs.name):
                    ################################################################################### rename
                    o = Parameter(obs.type.name +"_"+obs.name, obs.type)
                    c = Parameter('_'.join(config.get_contained_names()), config.type)
                    mc_fluents[o.name] = mov_objs_fluents[obs.name]
                    compatible_configs.append((o.name, c.name))
                    params.append(o) 
                    added.append(o)
                    params.append(c)
                    new_preconditions.append(Equals(config.fluent()(o), c))
                    
                    for old in added:
                        if old != o:
                            new_preconditions.append(Not(Equals(o, old)))
                else:
                    params.append(obs)
                    params.append(config)
                    mc_fluents[obs] = mov_objs_fluents[obs.name]
                    compatible_configs.append((obs.name, config.name))
        return Fluent(name, BoolType(), params), new_preconditions, mc_fluents, compatible_configs

    def _fluent_to_stream(self, name:str, fluent: Fluent, mov_objs_fluents: Optional[Dict[str, Fluent]], compatible_configs:Optional[List[tuple]], outputs: Optional[List[Parameter]] = None) -> str:
        s_inputs = []
        s_domain= []
        s_outputs = []
        s_output_certified = []
        for elem in fluent.signature:
            s_inputs.append(f"?{elem.name}")
            s_domain.append(f"(is_{elem.type} ?{elem.name})") 
            if elem.name in mov_objs_fluents.keys():
                s_domain.append(f"({mov_objs_fluents[elem.name].name} ?{elem.name})")

        for (mo_name, config_name) in compatible_configs:
            s_domain.append(f"(is_compatible ?{mo_name} ?{config_name})")

        if outputs:
            for o in outputs:
                s_outputs.append(f"?{o.name}")
                s_output_certified.append(f"(is_{o.type} ?{o.name})")
        stream = f'''\
  (:stream {name}
    :inputs ({' '.join(s_inputs)})
    :domain (and {' '.join(s_domain)})
    :outputs ({' '.join(s_outputs)})
    :certified (and {' '.join(s_output_certified)} ({fluent.name} {' '.join(s_inputs+s_outputs)} ))
  )
'''     
        #print(stream)
        return stream
    
    def _create_stream_pddl(self, name: str, streams: List[str]) -> str:

        stream_pddl = f"(define (stream {name}-stream)\n"
        for s in streams:
            stream_pddl+=s
        stream_pddl+=")"

        return stream_pddl

    def _solve(
        self,
        problem: "up.model.AbstractProblem",
        heuristic: Optional[Callable[["up.model.state.State"], Optional[float]]] = None,
        timeout: Optional[float] = None,
        output_stream: Optional[IO[str]] = None,
    ) -> "up.engines.results.PlanGenerationResult":
        assert isinstance(problem, up.model.Problem)
        if timeout is not None:
            warnings.warn("PDDLStream does not support timeout.", UserWarning)
        if output_stream is not None:
            warnings.warn("PDDLStream does not support output stream.", UserWarning)
        if heuristic is not None:
            warnings.warn("PDDLStream does not support custom heuristics.", UserWarning)

        streams = []
        stream_map = {}
        planning_data = []
        
        mp = MotionPlanner()

        new_actions = []
        mov_objs_fluents = {}
        original_actions = {a.name: a for a in problem.actions}

        # add is_X fluent for each movable object
        for o in problem.all_objects:
            if isinstance(o, MovableObject):
                new_f = Fluent(f"is_{o.name}", BoolType(), [Parameter(o.type.name, o.type)])
                if new_f not in mov_objs_fluents.values():
                    mov_objs_fluents[o.name] = new_f

        for f in mov_objs_fluents.values():
            problem.add_fluent(f, default_initial_value=False)

        for mo_name, mo_fluent in mov_objs_fluents.items():
            problem.set_initial_value(mo_fluent(problem.object(mo_name)), True) 

        #add compatible_config fluents

        for i, value in problem.initial_values.copy().items():
            x = i.fluent().name
            if i.fluent().name == 'open_config':
                comp_fluent = Fluent("is_compatible", BoolType(), door=i.args[0].type, door_config=value.type)
                if not problem.has_fluent(comp_fluent.name):
                    problem.add_fluent(comp_fluent, default_initial_value=False)
                problem.set_initial_value(comp_fluent(i.args[0], value), True)
            elif i.fluent().name == 'close_config':
                comp_fluent = Fluent("is_compatible", BoolType(), door=i.args[0].type, door_config=value.type)
                if not problem.has_fluent(comp_fluent.name):
                    problem.add_fluent(comp_fluent, default_initial_value=False)
                problem.set_initial_value(comp_fluent(i.args[0], value), True)

        #for i in problem.initial_values:
        #    x = f.signature 
        #prendo open_config e close_config --> compatible_config


        for a in problem.actions:
            if isinstance(a, InstantaneousMotionAction):
                
                params = OrderedDict()

                for mc in a.motion_constraints:

                    f, new_prec, mc_fluents, compatible_configs = self._mc_to_bool_fluent(f'{a.name}_connected', mc, problem, mov_objs_fluents)

                    # add new preconditions
                    for prec in new_prec:
                        a.add_precondition(prec)

                    TrajectoryType = UserType("t")
                    traj = Parameter("t", TrajectoryType)

                    # add stream with trajectory as output
                    streams.append(self._fluent_to_stream(f"{a.name}_connect", f, mc_fluents, compatible_configs, [traj]))

                    # add trajectory parameter to connection fluent
                    f.signature.append(traj)
                    
                    # add connection fluent to problem
                    problem.add_fluent(f, default_initial_value=False)

                    # add trajectory parameter to connection precondition of motion action
                    for param in a.parameters:
                        params[param.name] = param.type
                    for elem in f.signature:
                        if elem not in a.parameters:
                            params[elem.name] = elem.type
    
                    def bindFunction(name):
                        def func(*args):

                            #print("check collision: ",  args[0], # movable object
                            #                            args[1], # starting
                            #                            args[2], # goal
                            #                            {args[i]: args[i + 1] for i in range(3, len(args), 2)}, # {obstacle: obstacle_config}
                            #                            )

                            is_valid, path, _, _, data = mp.check_motion_constraint(args[0], # movable object
                                                                                 args[1], # starting
                                                                                 args[2], # goal
                                                                                 {args[i]: args[i + 1] for i in range(3, len(args), 2)}, # {obstacle: obstacle_config}
                                                                                 problem.all_objects, 
                                                                                 planning_time = self._motion_planning_time, 
                                                                                 interpolate = self._interpolate, 
                                                                                 simplified = self._simplified, 
                                                                                 tolerance = self._tolerance, 
                                                                                 distance = self._distance, 
                                                                                 motion_planner = self._motion_planner)

                            planning_data.append(data)

                            if args[0].motion_model == MotionModels.SE2:
                                return [SE2Path(path)] if is_valid else None
                            elif args[0].motion_model == MotionModels.REEDSSHEPP:
                                return [ReedsSheppPath(path)] if is_valid else None 
                            elif args[0].motion_model == MotionModels.SE3:
                                return [SE3Path(path)] if is_valid else None 
                            else:
                                raise NotImplementedError

                        func.__name__ = name
                        return func

                    stream_map[f"{a.name}_connect"] = from_fn(bindFunction(f"{a.name}_connected_configs")) #from_test(bindFunction(f"{a.name}_connected_configs"))

                new_a = InstantaneousAction(a.name, params)
                for pre in a.preconditions:
                    new_a.add_precondition(pre)

                f_params = []
                for elem in f.signature:
                    f_params.append(new_a.parameter(elem.name))
                new_a.add_precondition(FluentExp(f, f_params))

                for eff in a.effects:
                    if eff.is_increase():
                        new_a.add_increase_effect(eff.fluent, eff.value, eff.condition)
                    elif eff.is_decrease():
                        new_a.add_decrease_effect(eff.fluent, eff.value, eff.condition)
                    else:
                        new_a.add_effect(eff.fluent, eff.value, eff.condition)

                new_actions.append(new_a)
            else:
                new_actions.append(a)

        problem.clear_actions()      
        problem.add_actions(new_actions)

        stream_pddl = self._create_stream_pddl(problem.name, streams)          
        #print(stream_pddl)
       
        ufr = UsertypeFluentsRemover()
        comp_result = ufr.compile(problem)
        comp_problem = comp_result.problem
        #print(comp_problem)
        #print(comp_problem.kind)
    
        r = UserTypesRemover()
        res = r.compile(comp_problem)
        compiled_problem = res.problem

        init = []
        goal = []

        for i, value in compiled_problem.explicit_initial_values.items():
            if value.is_true():
                args = []
                for arg in i.args:
                    args.append(problem.object(arg.object().name))
                init.append((i.fluent().name, *args))

        for g in compiled_problem.goals:
            args = []
            for arg in g.args:
                args.append(problem.object(arg.object().name))
            goal.append((g.fluent().name, *args))

        w = PDDLWriter(compiled_problem)
        domain_pddl = w.get_domain()
        #print(domain_pddl)

        constant_map = {}
        for v in w.domain_objects.values():  
            for elem in v:
                constant_map[elem.name] = problem.object(elem.name)

        problem_stream = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, And(*goal))

        solution = solve(problem_stream, 
                         algorithm=self._pddl_stream_algorithm.value, 
                         unit_costs=self._pddl_stream_unit_costs, 
                         debug=self._pddl_stream_debug, 
                         visualize=self._pddl_stream_visualize,
                         max_memory=self._memout*1024) # KB

        plan, cost, evaluations = solution
        #print_solution(solution)

        if plan:

            up_actions = []

            for a in plan:
                up_motion_path = {}

                if isinstance(a, Action):
                    
                    args = a.args
                    if isinstance(a.args[-1], Path):  ################################################################################ check
                        up_motion_path[Waypoints(ObjectExp(a.args[0]), ObjectExp(a.args[1]), [ObjectExp(a.args[2])])] = a.args[-1].path
                        args = a.args[:len(original_actions[a.name].parameters)]
                    #elif isinstance(a.args[-1], FutureValue):
                    #    path = stream_map[a.args[-1].stream](*a.args[-1].input_values)
                    #    if path:
                    #        up_motion_path[Waypoints(ObjectExp(a.args[0]), ObjectExp(a.args[1]), [ObjectExp(a.args[2])])] = path
                    #    else:
                    #        return PlanGenerationResult(
                    #                    PlanGenerationResultStatus.UNSOLVABLE_INCOMPLETELY,
                    #                    None,
                    #                    self.name,
                    #        )             
                    up_a = original_actions[a.name](*args, motion_paths=up_motion_path)
                    up_actions.append(up_a)
                else:
                     return PlanGenerationResult(
                        PlanGenerationResultStatus.UNSOLVABLE_INCOMPLETELY,
                        None,
                        self.name,
                    )   
                #elif isinstance(a, StreamAction):
                    #inputs
                    #name future-NAME
                #    print('Stream action')

            up_plan = SequentialPlan(up_actions)

            return PlanGenerationResult(
               PlanGenerationResultStatus.SOLVED_SATISFICING,
               up_plan,
               self.name,
               metrics = metrics_from_planning_data(planning_data)
            )


        return PlanGenerationResult(
            PlanGenerationResultStatus.UNSOLVABLE_INCOMPLETELY,
            None,
            self.name,
        )



def metrics_from_planning_data(planning_data):
    pt = 0
    it = 0
    fcht = 0
    nw = 0
    nwai = 0
    pl = 0
    for d in planning_data:
        if d.planning_time is not None:
            pt += d.planning_time
        if d.interpolation_time is not None:
            it += d.interpolation_time
        if d.convex_hull_time is not None:
            fcht += d.convex_hull_time
        if d.n_waypoints is not None:
            nw += d.n_waypoints
        if d.n_waypoints_after_interpolation is not None:
            nwai += d.n_waypoints_after_interpolation
        if d.path_length is not None:
            pl += d.path_length
    metrics = {}
    metrics["planning_time"] = str(pt)
    metrics["interpolation_time"] = str(it)
    metrics["convex_hull_time"] = str(fcht)
    metrics["n_waypoints"] = str(nw)
    metrics["n_waypoints_after_interpolation"] = str(nwai)
    metrics["path_length"] = str(pl)
    return metrics
