import ast
import os
import string
from unified_planning.shortcuts import *
import yaml

PATH = os.path.dirname(os.path.abspath(__file__))

class Delivery:

    def __init__(self) -> None:
        pass

    def get_problem(self, dim: string, nd: int, nr: int, ng: int, nr_del: int, ng_del: int, robot_capacity: int):  

        Robot = MovableType('robot')
        Door = MovableType('door')
        Parcel = UserType('parcel') 

        domain_file = os.path.join(PATH, "configs/domain.yaml")
        parcels_file = os.path.join(PATH, "configs/tests/parcels.yaml")
        doors_file = os.path.join(PATH, f"configs/tests/doors_{dim}.yaml")  

        with open(domain_file) as f:
            domain_data = yaml.safe_load(f)
            if dim == '2D':
                domain_data = domain_data["2D"]
            elif dim == '3D':
                domain_data = domain_data["3D"]
            else:
                raise ValueError    

        # set problem
        problem = Problem("delivery")   

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

        #robot_capacity = 0
        #if "max_capacity" in domain_data["robot"]:
        #    robot_capacity = domain_data["robot"]["max_capacity"]   

        # robot configuration
        if robot_motion_model == MotionModels.REEDSSHEPP or robot_motion_model == MotionModels.SE2:
            Config = ConfigurationType('config', occ_map, 3)     #(x, y, yaw)
        elif robot_motion_model == MotionModels.SE3:
            Config = ConfigurationType('config', occ_map, 7)     #(x, y, z, rx, ry, rz, angle)
        else: 
             raise NotImplementedError

        robot_at = Fluent('robot_at', Config, robot=Robot)
        capacity = Fluent('capacity', IntType(), robot=Robot)
        max_capacity = Fluent('max_capacity', IntType(), robot=Robot)
        is_free = Fluent('is_free', BoolType(), config=Config)
        problem.add_fluents([robot_at, capacity, max_capacity])
        problem.add_fluent(is_free, default_initial_value=True) 

        move = InstantaneousMotionAction(f'move', r=Robot, c_from=Config, c_to=Config)
        r_move = move.parameter("r")
        c_from = move.parameter("c_from")
        c_to = move.parameter("c_to")
        move.add_precondition(Equals(robot_at(r_move), c_from))
        move.add_precondition(Not(Equals(robot_at(r_move), c_to)))
        move.add_precondition(Not(is_free(c_from)))
        move.add_precondition(is_free(c_to))
        move.add_effect(robot_at(r_move), c_to)
        move.add_effect(is_free(c_from), True)
        move.add_effect(is_free(c_to), False)
        if nd > 0:
            doors = []
            move.add_motion_constraint(Waypoints(r_move, c_from, [c_to], {d : door_at(d) for d in doors}))
        else:
            move.add_motion_constraint(Waypoints(r_move, c_from, [c_to]))   

        problem.add_action(move)    

        robot = MovableObject("r0", Robot, footprint=robot_footprint, model=robot_model, motion_model=robot_motion_model, parameters=robot_params)
        problem.add_object(robot)
        problem.set_initial_value(capacity(robot), 0)
        problem.set_initial_value(max_capacity(robot), robot_capacity)  

        # start
        robot_configs = {l["name"]: ast.literal_eval(l["pose"]) for l in domain_data["robot_configs"]}
        start_config = ConfigurationObject('c0', Config, robot_configs['c0'])
        problem.add_object(start_config)
        problem.set_initial_value(robot_at(robot), start_config)
        problem.set_initial_value(is_free(start_config), False)

        # door data
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

            open_door = InstantaneousMotionAction(f'open', d=Door, c_close=DoorConfig, c_open=DoorConfig)
            d = open_door.parameter("d")
            c_close = open_door.parameter("c_close")
            c_open = open_door.parameter("c_open")
            open_door.add_precondition(Equals(door_at(d), c_close))
            open_door.add_precondition(Equals(closed(d), c_close))
            open_door.add_precondition(Equals(opened(d), c_open))
            open_door.add_effect(door_at(d), c_open)
            open_door.add_motion_constraint(Waypoints(d, c_close, [c_open]))        

            problem.add_action(open_door)

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


        if (nr + ng) > 0:   

            parcel_at = Fluent('parcel_at', BoolType(), parcel=Parcel, config=Config)
            delivery_config = Fluent('delivery_config', BoolType(), config1=Config, config2=Config)
            unloading_config = Fluent('unloading_config', BoolType(), parcel=Parcel, config=Config) 
            loaded = Fluent('loaded', BoolType(), parcel=Parcel)
            delivered = Fluent('delivered', BoolType(), parcel=Parcel)      

            # add fluents

            problem.add_fluent(loaded, default_initial_value=False)
            problem.add_fluent(unloading_config, default_initial_value=False)
            problem.add_fluent(delivered, default_initial_value=False)
            problem.add_fluent(parcel_at, default_initial_value=False)
            problem.add_fluent(delivery_config, default_initial_value=False)    

            load = InstantaneousAction('load', robot=Robot, robot_config=Config, parcel=Parcel, parcel_config=Config)
            r_load = load.parameter('robot')
            r_c_load = load.parameter('robot_config')
            p_load = load.parameter('parcel')
            p_c_load = load.parameter('parcel_config')
            load.add_precondition(Equals(robot_at(r_load), r_c_load))
            load.add_precondition(parcel_at(p_load, p_c_load))
            load.add_precondition(delivery_config(r_c_load, p_c_load))
            load.add_precondition(Not(is_free(r_c_load)))
            load.add_precondition(Not(is_free(p_c_load)))
            load.add_precondition(Not(loaded(p_load)))
            load.add_precondition(LE(capacity(r_load), max_capacity(r_load)))
            load.add_effect(loaded(p_load), True)
            load.add_effect(delivered(p_load), False)
            load.add_effect(parcel_at(p_load, p_c_load), False)
            load.add_effect(is_free(p_c_load), True)
            load.add_increase_effect(capacity(r_load), 1)       

            unload = InstantaneousAction('unload', robot=Robot, robot_config=Config, parcel=Parcel, parcel_config=Config)
            r_unload = unload.parameter('robot')
            r_c_unload = unload.parameter('robot_config')
            p_unload = unload.parameter('parcel')
            p_c_unload = unload.parameter('parcel_config')
            unload.add_precondition(Equals(robot_at(r_unload), r_c_unload))
            unload.add_precondition(Not(is_free(r_c_unload)))
            unload.add_precondition(loaded(p_unload))
            unload.add_precondition(unloading_config(p_unload, r_c_unload))
            unload.add_precondition(delivery_config(r_c_unload, p_c_unload))
            unload.add_precondition(is_free(p_c_unload))
            unload.add_effect(delivered(p_unload), True)
            unload.add_effect(loaded(p_unload), False)
            unload.add_effect(parcel_at(p_unload, p_c_unload), True)
            unload.add_effect(is_free(p_c_unload), False)
            unload.add_decrease_effect(capacity(r_unload), 1)   

            # add actions
            problem.add_actions((load, unload)) 

            # add parcels
            available_parcel_configs = {l["name"]: ast.literal_eval(l["pose"]) for l in domain_data["parcel_configs"]}
            available_unloading_configs = {l["color"]: l["config"] for l in domain_data["unload_configs"]}
            delivery_configs = {l["to"]: l["from"] for l in domain_data["matchings"]}
            default_unloading_configs = {}
            p_color ={} 

            parcels = {}

            with open(parcels_file) as f:
                parcel_data = yaml.safe_load(f)
                for color, targets in parcel_data[str((nr, ng, nr_del, ng_del))].items():
                    n = 0
                    for t in targets:
                        parcel_config = ConfigurationObject(t, Config, available_parcel_configs[t])

                        if not problem.has_object(parcel_config.name):
                            problem.add_object(parcel_config)   

                        if not delivery_configs[t]:
                            raise ValueError(f"Delivery config of {t} does not exists.")    

                        del_config = ConfigurationObject(delivery_configs[t], Config, robot_configs[delivery_configs[t]])   

                        if not problem.has_object(del_config.name):    
                            problem.add_object(del_config)          

                        parcel = Object(f"{color}{n}", Parcel)
                        n+=1
                        problem.add_object(parcel)
                        problem.add_goal(delivered(parcel))
                        parcels[parcel] = color 

                        problem.set_initial_value(parcel_at(parcel, parcel_config), True) 
                        problem.set_initial_value(is_free(parcel_config), False)            

                        problem.set_initial_value(delivery_config(del_config, parcel_config), True)         

                        if not t in available_unloading_configs[color]:
                            if not color in p_color.keys():
                                p_color[color] = 1
                            else:
                                p_color[color]+=1       

                        if t in available_unloading_configs[color]:
                            problem.set_initial_value(delivered(parcel), True)
                            available_unloading_configs[color].remove(t)
                            if not color in default_unloading_configs.keys():
                                default_unloading_configs[color] = [t]
                            else:
                                default_unloading_configs[color].append(t)  

                for color, n in p_color.items():
                    unloading_configs = available_unloading_configs[color][:n]  

                    if color in default_unloading_configs.keys():
                        unloading_configs+=default_unloading_configs[color]         

                    for elem in unloading_configs:
                        if problem.has_object(elem) and problem.object(elem).configuration != available_parcel_configs[elem]:
                            raise ValueError(f"Configuration {elem} already defined with different params.")    

                        un_config = ConfigurationObject(elem, Config, available_parcel_configs[elem])   

                        if not problem.has_object(elem):
                            problem.add_object(un_config)   

                        if not delivery_configs[elem]: 
                            raise ValueError(f"Delivery config of {elem} does not exists.") 

                        del_config = ConfigurationObject(delivery_configs[elem], Config, robot_configs[delivery_configs[elem]])         

                        if not problem.has_object(delivery_configs[elem]):
                            problem.add_object(del_config)          

                        problem.set_initial_value(delivery_config(del_config, un_config), True)         

                        for p, p_color in parcels.items():
                            if p_color== color:
                                problem.set_initial_value(unloading_config(p, del_config), True)    

        return problem

    def get_problem_from_config(self, dim: string, domain_file: string, problem_file: string):

        Robot = MovableType('robot')
        Door = MovableType('door')
        Parcel = UserType('parcel') 

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

        robot_capacity = 0
        if "max_capacity" in domain_data["robot"]:
            robot_capacity = domain_data["robot"]["max_capacity"]

        # robot configuration
        if robot_motion_model == MotionModels.REEDSSHEPP or robot_motion_model == MotionModels.SE2:
            Config = ConfigurationType('config', occ_map, 3)     #(x, y, yaw)
        elif robot_motion_model == MotionModels.SE3:
            Config = ConfigurationType('config', occ_map, 7)     #(x, y, z, rx, ry, rz, angle)
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
        robot_at = Fluent('robot_at', Config, robot=Robot)  

        door_at = Fluent('door_at', DoorConfig, door= Door)
        opened = Fluent('open_config', DoorConfig, door= Door)
        closed = Fluent('close_config', DoorConfig, door= Door) 

        parcel_at = Fluent('parcel_at', BoolType(), parcel=Parcel, config=Config)
        delivery_config = Fluent('delivery_config', BoolType(), config1=Config, config2=Config)
        unloading_config = Fluent('unloading_config', BoolType(), parcel=Parcel, config=Config) 

        is_free = Fluent('is_free', BoolType(), config=Config)
        loaded = Fluent('loaded', BoolType(), parcel=Parcel)
        delivered = Fluent('delivered', BoolType(), parcel=Parcel)  

        capacity = Fluent('capacity', IntType(), robot=Robot)
        max_capacity = Fluent('max_capacity', IntType(), robot=Robot)   

        doors = []
        move = InstantaneousMotionAction(f'move', robot=Robot, c_from=Config, c_to=Config)
        r_move = move.parameter("robot")
        c_from = move.parameter("c_from")
        c_to = move.parameter("c_to")
        move.add_precondition(Equals(robot_at(r_move), c_from))
        move.add_precondition(Not(Equals(robot_at(r_move), c_to)))
        move.add_precondition(Not(is_free(c_from)))
        move.add_precondition(is_free(c_to))
        move.add_effect(robot_at(r_move), c_to)
        move.add_effect(is_free(c_from), True)
        move.add_effect(is_free(c_to), False)
        move.add_motion_constraint(Waypoints(r_move, c_from, [c_to], {d : door_at(d) for d in doors}))  

        open_door = InstantaneousMotionAction(f'open', door=Door, c_close=DoorConfig, c_open=DoorConfig)
        d = open_door.parameter("door")
        c_close = open_door.parameter("c_close")
        c_open = open_door.parameter("c_open")
        open_door.add_precondition(Equals(door_at(d), c_close))
        open_door.add_precondition(Equals(closed(d), c_close))
        open_door.add_precondition(Equals(opened(d), c_open))
        open_door.add_effect(door_at(d), c_open)
        open_door.add_motion_constraint(Waypoints(d, c_close, [c_open]))    

        load = InstantaneousAction('load', robot=Robot, robot_config=Config, parcel=Parcel, parcel_config=Config)
        r_load = load.parameter('robot')
        r_c_load = load.parameter('robot_config')
        p_load = load.parameter('parcel')
        p_c_load = load.parameter('parcel_config')
        load.add_precondition(Equals(robot_at(r_load), r_c_load))
        load.add_precondition(parcel_at(p_load, p_c_load))
        load.add_precondition(delivery_config(r_c_load, p_c_load))
        load.add_precondition(Not(is_free(r_c_load)))
        load.add_precondition(Not(is_free(p_c_load)))
        load.add_precondition(Not(loaded(p_load)))
        load.add_precondition(LE(capacity(r_load), max_capacity(r_load)))
        load.add_effect(loaded(p_load), True)
        load.add_effect(delivered(p_load), False)
        load.add_effect(parcel_at(p_load, p_c_load), False)
        load.add_effect(is_free(p_c_load), True)
        load.add_increase_effect(capacity(r_load), 1)   

        unload = InstantaneousAction('unload', robot=Robot, robot_config=Config, parcel=Parcel, parcel_config=Config)
        r_unload = unload.parameter('robot')
        r_c_unload = unload.parameter('robot_config')
        p_unload = unload.parameter('parcel')
        p_c_unload = unload.parameter('parcel_config')
        unload.add_precondition(Equals(robot_at(r_unload), r_c_unload))
        unload.add_precondition(Not(is_free(r_c_unload)))
        unload.add_precondition(loaded(p_unload))
        unload.add_precondition(unloading_config(p_unload, r_c_unload))
        unload.add_precondition(delivery_config(r_c_unload, p_c_unload))
        unload.add_precondition(is_free(p_c_unload))
        unload.add_effect(delivered(p_unload), True)
        unload.add_effect(loaded(p_unload), False)
        unload.add_effect(parcel_at(p_unload, p_c_unload), True)
        unload.add_effect(is_free(p_c_unload), False)
        unload.add_decrease_effect(capacity(r_unload), 1)

        # set up problem
        problem = Problem("navigation") 

        # add fluents
        problem.add_fluents([robot_at, door_at, opened, closed, capacity, max_capacity])
        problem.add_fluent(is_free, default_initial_value=True)
        problem.add_fluent(loaded, default_initial_value=False)
        problem.add_fluent(unloading_config, default_initial_value=False)
        problem.add_fluent(delivered, default_initial_value=False)
        problem.add_fluent(parcel_at, default_initial_value=False)
        problem.add_fluent(delivery_config, default_initial_value=False)    

        # add actions
        problem.add_actions((move, open_door, load, unload))    

        # add robots
        involved_robots = problem_data["robots"]
        available_robot_configs = {l["name"]: ast.literal_eval(l["pose"]) for l in domain_data["robot_configs"]}    

        for r in involved_robots:   

            # robot
            if problem.has_object(r["name"]) and (problem.object(r["name"]).footprint != robot_footprint or 
                                                  problem.object(r["name"]).foomotion_model !=robot_motion_model or 
                                                  problem.object(r["name"]).paramters != robot_params):
                raise ValueError(f"Robot {r['name']} already defined with different params.")   

            robot = MovableObject(r["name"], Robot, footprint=robot_footprint, model=robot_model, motion_model=robot_motion_model, parameters=robot_params)

            if not problem.has_object(r["name"]): 
                problem.add_object(robot)
                problem.set_initial_value(capacity(robot), 0)
                problem.set_initial_value(max_capacity(robot), robot_capacity)

            # start config
            if problem.has_object(r["start"]) and problem.object(r["start"]).configuration != available_robot_configs[r["start"]]:
                raise ValueError(f"Location {r['start']} already defined with different params.")

            start_config = ConfigurationObject(r["start"], Config, available_robot_configs[r["start"]])
            if not problem.has_object(r["start"]):
                problem.add_object(start_config)

            problem.set_initial_value(robot_at(robot), start_config)
            problem.set_initial_value(is_free(start_config), False) 

        # add doors
        involved_doors = problem_data["doors"]
        door_configs = {l["name"]: (ast.literal_eval(l["open"]), ast.literal_eval(l["close"])) for l in domain_data["door_configs"]}    

        for d in involved_doors:    

            # door
            if problem.has_object(d["name"]) and (problem.object(d["name"]).footprint != door_footprint or 
                                                  problem.object(d["name"]).foomotion_model !=door_motion_model or 
                                                  problem.object(d["name"]).paramters != door_params):
                raise ValueError(f"Door {d['name']} already defined with different params.")    

            door = MovableObject(d["name"], Door, footprint=door_footprint, model=door_model, motion_model=door_motion_model, parameters=door_params)

            if not problem.has_object(d["name"]): 
                problem.add_object(door)
                doors.append(door)

            # open config
            if problem.has_object(d["name"]+"_open") and problem.object(d["name"]+"_open").configuration != door_configs[d["name"]][0]:
                raise ValueError(f"Open configuration {d['name']}_open already defined with different params.")

            open_config = ConfigurationObject(d["name"]+"_open", DoorConfig, door_configs[d["name"]][0])

            if not problem.has_object(d["name"]+"_open"):
                problem.add_object(open_config) 

            problem.set_initial_value(opened(door), open_config)    

            # close configs
            if problem.has_object(d["name"]+"_closed") and problem.object(d["name"]+"_closed").configuration != door_configs[d["name"]][1]:
                raise ValueError(f"Open configuration {d['name']}_close already defined with different params.")

            close_config = ConfigurationObject(d["name"]+"_closed", DoorConfig, door_configs[d["name"]][1])

            if not problem.has_object(d["name"]+"_closed"):
                problem.add_object(close_config)    

            problem.set_initial_value(closed(door), close_config)   

            if d["start"] == 'open':
                problem.set_initial_value(door_at(door), open_config)
            elif d["start"] == 'close':
                problem.set_initial_value(door_at(door), close_config)
    

        # add parcels
        involved_parcels = problem_data["parcels"]
        available_parcel_configs = {l["name"]: ast.literal_eval(l["pose"]) for l in domain_data["parcel_configs"]}
        available_unloading_configs = {l["color"]: l["config"] for l in domain_data["unload_configs"]}
        delivery_configs = {l["to"]: l["from"] for l in domain_data["matchings"]}
        default_unloading_configs = {}  

        p_color = {}    

        for p in involved_parcels:

            parcel = Object(p["name"], Parcel)
            if not problem.has_object(p["name"]):
                problem.add_object(parcel)
                problem.add_goal(delivered(parcel)) 

            if problem.has_object(p["start"]) and problem.object(p["start"]).configuration != available_parcel_configs[p["start"]]:
                raise ValueError(f"Configuration {p['start']} already defined with different params.")

            parcel_config = ConfigurationObject(p["start"], Config, available_parcel_configs[p["start"]])

            if not problem.has_object(p["start"]):
                problem.add_object(parcel_config)

            if not delivery_configs[p["start"]]:
                raise ValueError(f"Delivery config of {p['start']} does not exists.")

            del_config = ConfigurationObject(delivery_configs[p["start"]], Config, available_robot_configs[delivery_configs[p["start"]]])   

            if not problem.has_object(delivery_configs[p["start"]]):
                problem.add_object(del_config)  

            problem.set_initial_value(parcel_at(parcel, parcel_config), True)
            problem.set_initial_value(is_free(parcel_config), False)    

            problem.set_initial_value(delivery_config(del_config, parcel_config), True) 

            if not p["start"] in available_unloading_configs[p["color"]]:
                if not p["color"] in p_color.keys():
                    p_color[p["color"]] = 1
                else:
                    p_color[p["color"]]+=1  

            if p["start"] in available_unloading_configs[p["color"]]:
                problem.set_initial_value(delivered(parcel), True)
                available_unloading_configs[p["color"]].remove(p["start"])
                if not p["color"] in default_unloading_configs.keys():
                    default_unloading_configs[p["color"]] = [p["start"]]
                else:
                    default_unloading_configs[p["color"]].append(p["start"])

        for color, n in p_color.items():
            unloading_configs = available_unloading_configs[color][:n]

            if color in default_unloading_configs.keys():
                unloading_configs+=default_unloading_configs[color] 

            for elem in unloading_configs:
                if problem.has_object(elem) and problem.object(elem).configuration != available_parcel_configs[elem]:
                    raise ValueError(f"Configuration {elem} already defined with different params.")

                un_config = ConfigurationObject(elem, Config, available_parcel_configs[elem])

                if not problem.has_object(elem):
                    problem.add_object(un_config)

                if not delivery_configs[elem]:
                    raise ValueError(f"Delivery config of {elem} does not exists.")

                del_config = ConfigurationObject(delivery_configs[elem], Config, available_robot_configs[delivery_configs[elem]])   

                if not problem.has_object(delivery_configs[elem]):
                    problem.add_object(del_config)  

                problem.set_initial_value(delivery_config(del_config, un_config), True) 

                for p in involved_parcels:
                    if p["color"] == color:
                        problem.set_initial_value(unloading_config(problem.object(p["name"]), del_config), True)    

        return problem
