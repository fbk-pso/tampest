import os
from typing import Optional
import yaml

def get_problem(domain, dim, d, c, capacity: Optional[int] = None):
    if domain == "doors":
        from benchmarks.doors.problem import Doors
        c0, c1 = [(0, 0), (10, 0), (0, 10), (5, 5)][c]
        if dim == '3D':
            raise NotImplementedError('Benchmark Doors is only available in 2D.') 
        return Doors().get_problem(d, c0=c0, c1=c1)
    if domain == "maze":
        from benchmarks.maze.problem import Maze
        return Maze().get_problem(dim, d, c)
    if domain == "delivery": 
        from benchmarks.delivery.problem import Delivery
        FILE_PATH = os.path.dirname(os.path.abspath(__file__))
        CONFIG_FILE = os.path.join(FILE_PATH, 'delivery/configs/tests/all.yaml')
        with open(CONFIG_FILE) as f:
            config_data = yaml.safe_load(f)
        current_config = config_data[c] # (red parcels, green parcels, red delivered parcels, green delivered parcels) 
        return Delivery().get_problem(dim, d, current_config["c0"], current_config["c1"], current_config["d0"], current_config["d1"], capacity) 
    if domain == "rover":
        from benchmarks.rover.problem import Rover
        return Rover().get_problem(dim, d, c)
    return None
