import itertools
import math
import os
import yaml
from typing import Tuple, Dict
from PIL import Image
import numpy as np
from unified_planning.shortcuts import *


FILE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "configs/2D")

class Doors:

    def __init__(self, tmpdirname=None) -> None:
        if tmpdirname:
            self.dirname = tmpdirname
        else:
            self.dirname = FILE_PATH

    def get_map(self, n: int) -> str:

        image_file = os.path.join(self.dirname, f'maps/{n}.png')
        yaml_file = os.path.join(self.dirname, f'maps/{n}.yaml')
        
        if os.path.exists(image_file) and os.path.exists(image_file):
            return yaml_file

        yaml_file = ""

        map_image = Image.new('RGBA', (300+100*(n-1), 300), (255, 255, 255, 255))
        door_image = os.path.join(self.dirname, "maps/door.png")
        door = Image.open(door_image)
        for i in range(n):
            offset = (142+100*i, 0)
            map_image.paste(door, offset)

        map_image.save(image_file)

        map = {}
        map['image'] = f'{n}.png'
        map['resolution'] = 0.1
        map['origin'] = [0, 0, 0]
        map['negate'] = 0
        map['occupied_thresh'] = 0.3
        map['free_thresh'] = 0.1

        with open(yaml_file, 'w') as file:
            yaml.dump(map, file)

        return yaml_file

    def get_random_configs(self, n: int, minx: float, maxx: float, miny: float, maxy: float) -> List[Tuple[float, ...]]:
        configs = []
        for i in range(n):
            yaw = np.random.choice([- math.pi / 2.0, math.pi / 2.0])
            pnt = (np.random.uniform(minx, maxx), np.random.uniform(miny, maxy), yaw)
            configs.append(pnt)
        return configs

    def get_closed_locs(self, n: int) -> Dict[str, Tuple[float, ...]]:
        locs = {}
        for i in range(n):
            locs[f'c{i}'] = (16.3+10.0*i, 15.0, 0.0)
        return locs

    def get_open_locs(self, n: int) -> Dict[str, Tuple[float, ...]]:
        locs = {}
        for i in range(n):
            locs[f'o{i}'] = (16.3+10.0*i, 21.0, 0.0)
        return locs

    def get_button_locs(self, n:int) -> Dict[str, Tuple[float, ...]]:
        locs = {}
        for i in range(n):
            locs[f'b{i}'] = (12.2+10.0*i, 15.0, - math.pi / 2.0)
        return locs

    def get_configs_from_file(region: int, n: int, filename: str) -> Dict[str, Tuple[float, ...]]:
        configs = {}
        with open(filename) as f:
            configs = yaml.safe_load(f)

        selection = {k: configs[region][k] for k in list(configs[configs])[:n]}
    
        return selection

    def get_problem(self, n_door: int, *, c0: Optional[int] = 0, c1: Optional[int] = 0): # co = # of configurations left - c1 = # of configurations right

        occ_map = OccupancyMap(self.get_map(n_door), (0, 0))
        Robot = MovableType('robot')
        Door = MovableType('door')
        RobotConfig = ConfigurationType('robot_config', occ_map, 3)
        DoorConfig = ConfigurationType('door_config', occ_map, 3)

        robot_at = Fluent('robot_at', RobotConfig, robot=Robot)
        door_at = Fluent('door_at', DoorConfig, door= Door)
        visited = Fluent('visited', BoolType(), config=RobotConfig)
        open_config = Fluent('open_config', DoorConfig, door= Door)
        close_config = Fluent('close_config', DoorConfig, door= Door)
        button_config = Fluent('button_config', RobotConfig, door= Door)

        robot = MovableObject('r1', Robot, footprint=[(-1.0, 0.5), (1.0, 0.5), (1.0, -0.5), (-1.0, -0.5)], motion_model=MotionModels.REEDSSHEPP, parameters={"turning_radius": 2.0})

        start_config = ConfigurationObject('s', RobotConfig, (3.0, 26.0,  - math.pi / 2.0))

        connection_configs = []
        if c0 != 0 or c1 != 0:
            filename = os.path.join(FILE_PATH, f"targets/{n_door}_doors.txt")

            dic = ''
            with open(filename,'r') as f:
                for i in f.readlines():
                    dic=i
            dic = eval(dic)

            connections = []
            if c0 != 0:
                connections += dic[0][:c0]
            if c1 != 0:
                connections += dic[1][:c1]
            
            connection_configs = [ConfigurationObject('t%s' % i, RobotConfig, connections[i]) for i in range(len(connections))]
            attached = Fluent('attached', BoolType(), c_from=RobotConfig, c_to=RobotConfig)

        goal_config = ConfigurationObject('g', RobotConfig, (26.0+ 10.0*(n_door-1), 26.0, math.pi / 2.0))
        
        doors = [MovableObject('d%s' % i, Door, footprint=[(-0.5, 3.0), (0.5, 3.0), (0.5, -3.0), (-0.5, -3.0)], motion_model=MotionModels.SE2, parameters = {}) for i in range(n_door)]

        open_locs = [ConfigurationObject(k, DoorConfig, v) for k, v in self.get_open_locs(n_door).items()]
        close_locs = [ConfigurationObject(k, DoorConfig, v) for k, v in self.get_closed_locs(n_door).items()]
        button_locs = [ConfigurationObject(k, RobotConfig, v) for k, v in self.get_button_locs(n_door).items()]

        move_action = InstantaneousMotionAction(f'move', robot=Robot, c_from=RobotConfig, c_to=RobotConfig)
        robot_move_param = move_action.parameter("robot")
        c_from = move_action.parameter("c_from")
        c_to = move_action.parameter("c_to")
        move_action.add_precondition(Equals(robot_at(robot_move_param), c_from))
        if connection_configs:
            move_action.add_precondition(attached(c_from, c_to))
        move_action.add_precondition(Not(Equals(robot_at(robot_move_param), c_to)))
        move_action.add_effect(robot_at(robot_move_param), c_to)
        move_action.add_effect(visited(c_to), True)
        move_action.add_motion_constraint(Waypoints(robot_move_param, c_from, [c_to], {d : door_at(d) for d in doors}))


        open_action = InstantaneousAction(f'open', robot=Robot, door=Door, c_r=RobotConfig, c_close=DoorConfig, c_open=DoorConfig)
        robot_open_param = open_action.parameter("robot")
        door = open_action.parameter("door")
        c_r = open_action.parameter("c_r")
        c_close = open_action.parameter("c_close")
        c_open = open_action.parameter("c_open")
        open_action.add_precondition(Equals(robot_at(robot_open_param), c_r))
        open_action.add_precondition(Equals(button_config(door), c_r))
        #open_action.add_precondition(Equals(robot_at(robot_open_param), button_config(door)))
        open_action.add_precondition(Equals(door_at(door), c_close))
        open_action.add_precondition(Not(Equals(door_at(door), c_open)))
        open_action.add_precondition(Equals(close_config(door), c_close))
        open_action.add_precondition(Equals(open_config(door), c_open))
        open_action.add_effect(door_at(door), c_open)

        problem = Problem("nav_with_sliding_door")

        problem.add_object(robot)
        problem.add_objects(doors)
        problem.add_object(start_config)
        problem.add_object(goal_config)
        if connection_configs:
            problem.add_objects(connection_configs)
        problem.add_objects(button_locs)
        problem.add_objects(open_locs)
        problem.add_objects(close_locs)

        problem.add_fluent(robot_at)
        problem.add_fluent(door_at)
        problem.add_fluent(open_config)
        problem.add_fluent(close_config)
        problem.add_fluent(button_config)
        problem.add_fluent(visited, default_initial_value=False)

        problem.add_actions((move_action, open_action))

        problem.set_initial_value(robot_at(robot), start_config)

        for i in range(n_door):
            problem.set_initial_value(button_config(doors[i]), button_locs[i])
            problem.set_initial_value(open_config(doors[i]), open_locs[i])
            problem.set_initial_value(close_config(doors[i]), close_locs[i])

        for i in range(n_door):
            initial_config = close_locs[i]
            problem.set_initial_value(door_at(doors[i]), initial_config)

        if connection_configs:
            problem.add_fluent(attached, default_initial_value=False)
            all_conn = itertools.permutations(connection_configs + [start_config, goal_config] + button_locs, 2)
            for c in all_conn:
                problem.set_initial_value(attached(c[0], c[1]), True)
        
        problem.add_goal(visited(goal_config))

        return problem