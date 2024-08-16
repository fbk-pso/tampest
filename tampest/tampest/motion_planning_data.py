from enum import Enum, auto
from typing import Optional


class SupportedTopologicalRefinement(Enum):
    NONE = auto() # [to] AND [all osbstacles]
    UNREACH = auto() # [to] + [unreachable targets] AND [all obstacles]
    OBS = auto() # [to] AND [only useful obstacles]
    ALL = auto() # [to] + [unreachable targets] AND [only useful obstacles]

class SupportedPlanner(Enum):

    EST = auto()
    KPIECE1 = auto()
    RRT = auto()
    LazyPRM = auto()
    LazyRRT = auto()
    PRM = auto()
    RRTstar = auto()
    SBL = auto()

class MotionPlanningData:

    def __init__(self, planning_time: Optional[float] = None, 
                       interpolation_time: Optional[float] = None, 
                       convex_hull_time : Optional[float] = None,
                       n_waypoints: Optional[int] = None,
                       n_waypoints_after_interpolation: Optional[int] = None,
                       path_length: Optional[float] = None):
        
        self._planning_time = planning_time
        self._interpolation_time = interpolation_time
        self._convex_hull_time = convex_hull_time
        self._n_waypoints = n_waypoints
        self._n_waypoints_after_interpolation = n_waypoints_after_interpolation
        self._path_length = path_length

    @property
    def planning_time(self) -> Optional[float]:
        return self._planning_time
    
    @planning_time.setter
    def planning_time(self, new_planning_time: float):
        self._planning_time = new_planning_time
        
    @property
    def interpolation_time(self) -> Optional[float]:
        return self._interpolation_time
    
    @interpolation_time.setter
    def interpolation_time(self, new_interpolation_time: float):
        self._interpolation_time = new_interpolation_time
        
    @property
    def convex_hull_time(self) -> Optional[float]:
        return self._convex_hull_time
        
    @convex_hull_time.setter
    def convex_hull_time(self, new_convex_hull_time: float):
        self._convex_hull_time = new_convex_hull_time
    @property
    def n_waypoints(self) -> Optional[int]:
        return self._n_waypoints
    
    @n_waypoints.setter
    def n_waypoints(self, new_n_waypoints: int):
        self._n_waypoints = new_n_waypoints

    @property
    def n_waypoints_after_interpolation(self) -> Optional[int]:
        return self._n_waypoints_after_interpolation
        
    @n_waypoints_after_interpolation.setter
    def n_waypoints_after_interpolation(self, new_n_waypoints_after_interpolation: int):
        self._n_waypoints_after_interpolation = new_n_waypoints_after_interpolation
    
    @property
    def path_length(self) -> Optional[float]:
        return self._path_length
    
    @path_length.setter
    def path_length(self, new_path_length: float):
        self._path_length = new_path_length



