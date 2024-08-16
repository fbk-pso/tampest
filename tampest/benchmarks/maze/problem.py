import ast
import os
import string
from unified_planning.shortcuts import *
import yaml

FILE_PATH = os.path.dirname(os.path.abspath(__file__))

class Maze:

    def __init__(self) -> None:
        pass

    def get_problem(self, dim: string, nd: int, nc: int):
        Robot = MovableType('robot')

        domain_file = os.path.join(FILE_PATH, "configs/domain.yaml")
        targets_file = os.path.join(FILE_PATH, "configs/tests/targets.yaml")
        doors_file = os.path.join(FILE_PATH, "configs/tests/doors.yaml")

        with open(domain_file) as f:
            domain_data = yaml.safe_load(f)
            if dim == '2D':
                domain_data = domain_data["2D"]
            elif dim == '3D':
                domain_data = domain_data["3D"]
            else:
                raise ValueError

        occ_map = OccupancyMap(os.path.join(os.path.dirname(domain_file), domain_data["world"]["file"]), domain_data["world"]["ref_frame"])
        
        # robot data
        robot_footprint = None
        if "footprint" in domain_data["robot"]:
            robot_footprint = ast.literal_eval(domain_data["robot"]["footprint"])

        robot_model = None
        if "model" in domain_data["robot"]:
            robot_model = os.path.join(os.path.dirname(domain_file), domain_data["robot"]["model"])

        robot_motion_model = MotionModels[domain_data["robot"]["motion_model"]]
        robot_params = None
        if "params" in domain_data["robot"]:
            robot_params = ast.literal_eval(domain_data["robot"]["params"])
        
        # robot configuration
        if robot_motion_model == MotionModels.REEDSSHEPP or robot_motion_model == MotionModels.SE2:
            RobotConfig = ConfigurationType('robot_config', occ_map, 3)     #(x, y, yaw)
        elif robot_motion_model == MotionModels.SE3:
            RobotConfig = ConfigurationType('robot_config', occ_map, 7)     #(x, y, z, rx, ry, rz, angle)
        else: 
             raise NotImplementedError
        

        # fluents
        robot_at = Fluent('robot_at', RobotConfig, robot=Robot)  
        visited = Fluent('visited', BoolType(), config=RobotConfig)
        
        # set problem
        problem = Problem("maze")

        # add fluents
        problem.add_fluent(robot_at, default_initial_value=False)
        problem.add_fluent(visited, default_initial_value=False)

        # add robots and their configurations
        robot_configs = {l["name"]: ast.literal_eval(l["pose"]) for l in domain_data["robot_configs"]}

        robot = MovableObject("r0", Robot, footprint=robot_footprint, model=robot_model, motion_model=robot_motion_model, parameters=robot_params)
        problem.add_object(robot)

        # start
        start_config = ConfigurationObject('c0', RobotConfig, robot_configs['c0'])
        problem.add_object(start_config)
        problem.set_initial_value(robot_at(robot), start_config)

        # goal
        goal_config = ConfigurationObject('c10', RobotConfig, robot_configs['c10'])
        problem.add_object(goal_config) 
        problem.add_goal(visited(goal_config))

        # intermediate

        if nc > 0: 
            with open(targets_file) as f:
                target_data = yaml.safe_load(f)
                for t in target_data[nc]:
                    target = ConfigurationObject(t, RobotConfig, robot_configs[t])
                    problem.add_object(target) 
                    problem.add_goal(visited(target))

        # doors
        doors = []
        if nd > 0: 
            # door data
            Door = MovableType('door')
            door_footprint = None
            if "footprint" in domain_data["door"]:
                door_footprint = ast.literal_eval(domain_data["door"]["footprint"])

            door_model = None
            if "model" in domain_data["door"]:
                door_model = os.path.join(os.path.dirname(domain_file), domain_data["door"]["model"])

            door_motion_model = MotionModels[domain_data["door"]["motion_model"]]
            door_params = None
            if "params" in domain_data["door"]:
                door_params = ast.literal_eval(domain_data["door"]["params"])

            # door configuration
            if door_motion_model  == MotionModels.REEDSSHEPP or door_motion_model == MotionModels.SE2:
                DoorConfig = ConfigurationType('door_config', occ_map, 3)      #(x, y, yaw)
            elif robot_motion_model == MotionModels.SE3:
                DoorConfig = ConfigurationType('door_config', occ_map, 7)      #(x, y, z, rx, ry, rz, angle)
            else: 
                 raise NotImplementedError
        
            door_at = Fluent('door_at', DoorConfig, door= Door)
            opened = Fluent('open_config', DoorConfig, door= Door)
            closed = Fluent('close_config', DoorConfig, door= Door)

            problem.add_fluents([door_at, opened, closed])
            
            doors_configs = {l["name"]: (ast.literal_eval(l["open"]), ast.literal_eval(l["close"])) for l in domain_data["door_configs"]}

            with open(doors_file) as f:
                doors_data = yaml.safe_load(f)
                for d in doors_data[nd]:
                    door_open, door_close = doors_configs[d]

                    door = MovableObject(d, Door, footprint=door_footprint, model=door_model, motion_model=door_motion_model, parameters=door_params)
                    problem.add_object(door)
                    doors.append(door)

                    open_config = ConfigurationObject(d+"_open", DoorConfig, door_open)
                    problem.add_object(open_config)
                    problem.set_initial_value(opened(door), open_config)

                    close_config = ConfigurationObject(d+"_closed", DoorConfig, door_close)
                    problem.add_object(close_config)
                    problem.set_initial_value(closed(door), close_config)

                    problem.set_initial_value(door_at(door), close_config)
                
        
        # add actions
        move_robot = InstantaneousMotionAction(f'move', robot=Robot, c_from=RobotConfig, c_to=RobotConfig)
        r = move_robot.parameter("robot")
        c_from = move_robot.parameter("c_from")
        c_to = move_robot.parameter("c_to")
        move_robot.add_precondition(Equals(robot_at(r), c_from))
        move_robot.add_precondition(Not(Equals(robot_at(r), c_to)))
        move_robot.add_effect(robot_at(r), c_to)
        move_robot.add_effect(visited(c_to), True)
        if nd > 0:
            move_robot.add_motion_constraint(Waypoints(r, c_from, [c_to], {d : door_at(d) for d in doors}))
        else:
            move_robot.add_motion_constraint(Waypoints(r, c_from, [c_to]))

        problem.add_action(move_robot)

        if nd > 0:
            open_door = InstantaneousMotionAction(f'open', door=Door, c_close=DoorConfig, c_open=DoorConfig)
            d = open_door.parameter("door")
            c_close = open_door.parameter("c_close")
            c_open = open_door.parameter("c_open")
            open_door.add_precondition(Equals(door_at(d), c_close))
            open_door.add_precondition(Equals(closed(d), c_close))
            open_door.add_precondition(Equals(opened(d), c_open))
            open_door.add_effect(door_at(d), c_open)
            open_door.add_motion_constraint(Waypoints(d, c_close, [c_open]))

            problem.add_action(open_door)

        return problem


    def get_problem_from_config(self, dim: string, domain_file: string, problem_file: string): 

        Robot = MovableType('robot')
        Door = MovableType('door')

        with open(domain_file) as f:
            domain_data = yaml.safe_load(f)
            if dim == '2D':
                domain_data = domain_data["2D"]
            elif dim == '3D':
                domain_data = domain_data["3D"]
            else:
                raise ValueError
        
        with open(problem_file) as f:
            problem_data = yaml.safe_load(f)

        occ_map = OccupancyMap(os.path.join(os.path.dirname(domain_file), domain_data["world"]["file"]), domain_data["world"]["ref_frame"])
        
        # robot data
        robot_footprint = None
        if "footprint" in domain_data["robot"]:
            robot_footprint = ast.literal_eval(domain_data["robot"]["footprint"])

        robot_model = None
        if "model" in domain_data["robot"]:
            robot_model = os.path.join(os.path.dirname(domain_file), domain_data["robot"]["model"])

        robot_motion_model = MotionModels[domain_data["robot"]["motion_model"]]
        robot_params = None
        if "params" in domain_data["robot"]:
            robot_params = ast.literal_eval(domain_data["robot"]["params"])
        
        # robot configuration
        if robot_motion_model == MotionModels.REEDSSHEPP or robot_motion_model == MotionModels.SE2:
            RobotConfig = ConfigurationType('robot_config', occ_map, 3)     #(x, y, yaw)
        elif robot_motion_model == MotionModels.SE3:
            RobotConfig = ConfigurationType('robot_config', occ_map, 7)     #(x, y, z, rx, ry, rz, angle)
        else: 
             raise NotImplementedError
        
        # door data
        door_footprint = None
        if "footprint" in domain_data["door"]:
            door_footprint = ast.literal_eval(domain_data["door"]["footprint"])

        door_model = None
        if "model" in domain_data["door"]:
            door_model = os.path.join(os.path.dirname(domain_file), domain_data["door"]["model"])

        door_motion_model = MotionModels[domain_data["door"]["motion_model"]]
        door_params = None
        if "params" in domain_data["door"]:
            door_params = ast.literal_eval(domain_data["door"]["params"])

        # door configuration
        if door_motion_model  == MotionModels.REEDSSHEPP or door_motion_model == MotionModels.SE2:
            DoorConfig = ConfigurationType('door_config', occ_map, 3)      #(x, y, yaw)
        elif robot_motion_model == MotionModels.SE3:
            DoorConfig = ConfigurationType('door_config', occ_map, 7)      #(x, y, z, rx, ry, rz, angle)
        else: 
             raise NotImplementedError

        # fluents
        robot_at = Fluent('robot_at', RobotConfig, robot=Robot)
        door_at = Fluent('door_at', DoorConfig, door= Door)
        visited = Fluent('visited', BoolType(), config=RobotConfig)
        opened = Fluent('open_config', DoorConfig, door= Door)
        closed = Fluent('close_config', DoorConfig, door= Door)

        # set problem
        problem = Problem("maze")

        # add fluents
        problem.add_fluents([robot_at, door_at, opened, closed])
        problem.add_fluent(visited, default_initial_value=False)

        # add robots and their configurations
        involved_robots = problem_data["robots"]
        robot_configs = {l["name"]: ast.literal_eval(l["pose"]) for l in domain_data["robot_configs"]}

        for r in involved_robots:

            # robot
            if problem.has_object(r["name"]) and (problem.object(r["name"]).footprint != robot_footprint or 
                                                  problem.object(r["name"]).foomotion_model !=robot_motion_model or 
                                                  problem.object(r["name"]).paramters != robot_params):
                raise ValueError(f"Robot {r['name']} already defined with different params.")

            if not problem.has_object(r["name"]):
                robot = MovableObject(r["name"], Robot, footprint=robot_footprint, model=robot_model, motion_model=robot_motion_model, parameters=robot_params)
                problem.add_object(robot)
             
            # start config
            if problem.has_object(r["start"]) and problem.object(r["start"]).configuration != robot_configs[r["start"]]:
                raise ValueError(f"Location {r['start']} already defined with different params.")
              
            start_config = ConfigurationObject(r["start"], RobotConfig, robot_configs[r["start"]])
             
            if not problem.has_object(r["start"]):
                problem.add_object(start_config)
                
            problem.set_initial_value(robot_at(robot), start_config)

            # goal configs
            for g in r["goals"]:
                if problem.has_object(g) and problem.object(g).configuration != robot_configs[g]:
                    raise ValueError(f"Location {g} already defined with different params.")

                goal_config = ConfigurationObject(g, RobotConfig, robot_configs[g])

                if not problem.has_object(g):
                    problem.add_object(goal_config) 

                problem.add_goal(visited(goal_config))
        

        # add doors and their configurations
        door_configs = domain_data["door_configs"]
        involved_doors = problem_data["obstacles"]
        doors = []

        for d in involved_doors:
            for c in door_configs:
                if d["name"] == c["name"]:
                    # door
                    if problem.has_object(c["name"]) and (problem.object(c["name"]).footprint != door_footprint or 
                                                          problem.object(c["name"]).foomotion_model !=door_motion_model or 
                                                          problem.object(c["name"]).paramters != door_params):
                        raise ValueError(f"Door {c['name']} already defined with different params.")

                    if not problem.has_object(c["name"]):
                        door = MovableObject(d["name"], Door, footprint=door_footprint, model=door_model, motion_model=door_motion_model, parameters=door_params)
                        problem.add_object(door)
                        doors.append(door)
                    
                    # open config
                    if problem.has_object(c["name"]+"_open") and problem.object(c["name"]+"_open").configuration != ast.literal_eval(o["open"]):
                        raise ValueError(f"Open configuration {c['name']} already defined with different params.")
                      
                    open_config = ConfigurationObject(c["name"]+"_open", DoorConfig, ast.literal_eval(c["open"]))
                    
                    if not problem.has_object(c["name"]+"_open"):
                        problem.add_object(open_config)

                    problem.set_initial_value(opened(door), open_config)

                    # close configs
                    # open config
                    if problem.has_object(c["name"]+"_closed") and problem.object(c["name"]+"_closed").configuration != ast.literal_eval(c["close"]):
                        raise ValueError(f"Open configuration {c['name']} already defined with different params.")
                      
                    close_config = ConfigurationObject(c["name"]+"_closed", DoorConfig, ast.literal_eval(c["close"]))
                    
                    if not problem.has_object(c["name"]+"_closed"):
                        problem.add_object(close_config)

                    problem.set_initial_value(closed(door), close_config)

                    if d["start"] == 'open':
                        problem.set_initial_value(door_at(door), open_config)
                    elif d["start"] == 'close':
                        problem.set_initial_value(door_at(door), close_config)

                    break
        
        # add actions
        move_robot = InstantaneousMotionAction(f'move', robot=Robot, c_from=RobotConfig, c_to=RobotConfig)
        r = move_robot.parameter("robot")
        c_from = move_robot.parameter("c_from")
        c_to = move_robot.parameter("c_to")
        move_robot.add_precondition(Equals(robot_at(r), c_from))
        move_robot.add_precondition(Not(Equals(robot_at(r), c_to)))
        move_robot.add_effect(robot_at(r), c_to)
        move_robot.add_effect(visited(c_to), True)
        move_robot.add_motion_constraint(Waypoints(r, c_from, [c_to], {d : door_at(d) for d in doors}))

        open_door = InstantaneousMotionAction(f'open', door=Door, c_close=DoorConfig, c_open=DoorConfig)
        d_open = open_door.parameter("door")
        c_close = open_door.parameter("c_close")
        c_open = open_door.parameter("c_open")
        open_door.add_precondition(Equals(door_at(d_open), c_close))
        open_door.add_precondition(Equals(closed(d_open), c_close))
        open_door.add_precondition(Equals(opened(d_open), c_open))
        open_door.add_effect(door_at(d_open), c_open)
        open_door.add_motion_constraint(Waypoints(d_open, c_close, [c_open]))

        problem.add_actions((move_robot, open_door))

        return problem
