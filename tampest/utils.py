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

def metrics_from_planning_data(planning_data):
    pt = 0
    it = 0
    fcht = 0
    nw = 0
    nwai = 0
    pl = 0
    for d in planning_data.values():
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
    metrics["motion_planning_time"] = str(pt)
    metrics["interpolation_time"] = str(it)
    metrics["convex_hull_time"] = str(fcht)
    metrics["n_waypoints"] = str(nw)
    metrics["n_waypoints_after_interpolation"] = str(nwai)
    metrics["path_length"] = str(pl)
    return metrics
