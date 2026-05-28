# Copyright (C) 2024-2026 PSO Unit, Fondazione Bruno Kessler
# This file is part of TAMPEST.
#
# TAMPEST is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# TAMPEST is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
#

import math
from typing import Dict, List, Optional
from ompl import base as ob
from tampest.motion.map import Map
from unified_planning.shortcuts import MovableObject, ConfigurationObject


class SpaceTimeMotionValidator(ob.MotionValidator):

    def __init__(
        self,
        si: ob.SpaceInformation,
        map: Map,
        robots: List[MovableObject],
        action_starts: Dict[int, float],
        start_configs: Dict[int, ConfigurationObject],
        sequential_check: Optional[bool] = False,
    ):
        super().__init__(si)
        self._si = si
        self._map = map
        self._robots = robots
        self._action_starts = action_starts
        self._start_configs = start_configs
        self._sequential_check = sequential_check

    def xy_distance(self, p1, p2) -> float:
        return math.sqrt((p2.getX() - p1.getX()) ** 2 + (p2.getY() - p1.getY()) ** 2)

    def checkMotion(self, s1, s2) -> bool:

        current_time = self._si.getStateSpace().getStateTime(s1)
        for i, t in self._action_starts.items():
            if t > 0 and current_time <= t + 0.1:
                x = self._start_configs[i].configuration.x / self._map.resolution
                y = (
                    self._map.image.size[1]
                    - self._start_configs[i].configuration.y / self._map.resolution
                )
                yaw = self._start_configs[i].configuration.theta

                if (
                    s1[0][i].getX() != x
                    or s1[0][i].getY() != y
                    or s1[0][i].getYaw() != yaw
                ):
                    if self._sequential_check:
                        return False
                    else:
                        s1[0][i].setXY(x, y)
                        s1[0][i].setYaw(yaw)

        if not self._si.isValid(s2):
            return False

        t1 = self._si.getStateSpace().getStateTime(s1)
        t2 = self._si.getStateSpace().getStateTime(s2)
        delta_t = t2 - t1
        if delta_t <= 0:  # s2 after s1
            return False

        for i in range(len(self._robots)):
            r = self._robots[i]
            if not r.control_parameters or not r.control_parameters["v_max"]:
                raise (
                    f"Unable to check motion for robot {r.name}. Max velocity missing."
                )

            if len(self._robots) > 1:
                delta_pos = self.xy_distance(s1[0][i], s2[0][i])
                if delta_pos > 0.0001 and t1 < self._action_starts[i]:
                    return False
            else:
                delta_pos = self._si.getStateSpace().distanceSpace(s1, s2)

            if (delta_pos / delta_t) > r.control_parameters["v_max"]:
                return False

        return True
