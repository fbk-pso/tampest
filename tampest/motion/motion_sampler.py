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

from ompl import base as ob
from tampest.motion.map import Map2D
from unified_planning.shortcuts import ConfigurationObject
from typing import Dict


class StartAwareSampler(ob.StateSampler):

    def __init__(
        self,
        space: ob.StateSpace,
        start_timings: Dict[int, float],
        start_configs: Dict[int, ConfigurationObject],
        map: Map2D,
    ):
        super().__init__(space)
        self.name_ = "StartAwareSampler"
        self.space = space
        self.default_sampler = space.allocDefaultStateSampler()
        self.start_timings = start_timings
        self.start_configs = start_configs
        self.map = map

    def sampleUniform(self, state):

        self.default_sampler.sampleUniform(state)
        current_time = self.space.getStateTime(state)

        for i, t in self.start_timings.items():

            if t > 0 and current_time <= t + 0.1:
                x = self.start_configs[i].configuration.x / self.map.resolution
                y = (
                    self.map.image.size[1]
                    - self.start_configs[i].configuration.y / self.map.resolution
                )
                yaw = self.start_configs[i].configuration.theta
                state[0][i].setXY(x, y)
                state[0][i].setYaw(yaw)
