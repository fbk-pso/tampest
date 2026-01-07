# Copyright (C) 2024-2025 PSO Unit, Fondazione Bruno Kessler
# This file is part of TAMPEST.
#
# TAMPEST is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# TAMPEST is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
#

import math
from typing import Dict, List
from ompl import base as ob
from unified_planning.shortcuts import MovableObject


class SpaceTimeMotionValidator(ob.MotionValidator):

    def __init__(
        self,
        si: ob.SpaceInformation,
        robots: List[MovableObject],
        action_starts: Dict[int, float],
    ):
        super().__init__(si)
        self._si = si
        self._robots = robots
        self._action_starts = action_starts

    def xy_distance(self, p1, p2):
        return math.sqrt((p2.getX() - p1.getX()) ** 2 + (p2.getY() - p1.getY()) ** 2)

    def checkMotion(self, s1, s2):
        if not self._si.isValid(s2):
            return False

        # delta_t = self._si.getStateSpace().distanceTime(s1, s2)
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

            # delta_pos=self.xy_distance(s1[0][i], s2[0][i]

            if len(self._robots) > 1:
                delta_pos = self.xy_distance(s1[0][i], s2[0][i])
                if delta_pos > 0.0001 and t1 < self._action_starts[i]:
                    return False
                # delta_pos = self._si.getStateSpace().distanceSpace(s1[0][i], s2[0][i])
            else:
                delta_pos = self._si.getStateSpace().distanceSpace(s1, s2)

            if (delta_pos / delta_t) > r.control_parameters[
                "v_max"
            ]:  # self._si.getStateSpace().getVMax()
                return False

        return True
