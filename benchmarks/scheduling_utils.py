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

from typing import List
import math
from unified_planning.model.scheduling import SchedulingProblem, Activity
from unified_planning.model.motion import MotionActivity
from unified_planning.shortcuts import *


def compute_time_using_trapezoidal_velocity_profile(
    distance: int, acceleration: int, max_velocity: int
) -> float:
    """
    Calculates the total time required to traverse a given distance using a symmetric trapezoidal velocity profile.

    In this profile, the object accelerates at a constant rate up to a maximum velocity (not exceeding `max_velocity`),
    maintains that velocity for a period, and then decelerates at the same rate to a stop.

    Args:
        distance (int): The total distance to be traveled (in meters).
        acceleration (int): The constant acceleration and deceleration rate (in meters per second squared).
        max_velocity (int): The maximum velocity allowed (in meters per second).

    Returns:
        float: The total elapsed time (in seconds) to traverse the specified distance using the trapezoidal velocity profile.

    Note:
        Assumes initial and final velocities are zero. If the distance is too short to reach `max_velocity`, a triangular profile is used.
    """

    # Calculate time to reach max_velocity
    t_accel = max_velocity / acceleration
    # Distance covered during acceleration (and deceleration)
    d_accel = 0.5 * acceleration * t_accel**2

    if 2 * d_accel >= distance:
        # Triangular profile: never reaches max_velocity
        t_peak = math.sqrt(distance / acceleration)
        total_time = 2 * t_peak
    else:
        # Trapezoidal profile: accelerates, cruises, decelerates
        d_cruise = distance - 2 * d_accel
        t_cruise = d_cruise / max_velocity
        total_time = 2 * t_accel + t_cruise

    return total_time


def activity_precondition_constraint(
    activity: MotionActivity,
    helper_activities: List[MotionActivity],
    rival_activities: List[MotionActivity],
    is_initially_applicable: bool,
) -> FNode:
    cc0 = []
    for activity_h in helper_activities:
        assert activity != activity_h

        cc1 = []
        for activity_r in rival_activities:
            assert activity != activity_r and activity_h != activity_r

            cc1.append(
                Implies(
                    activity_r.present,
                    Or(
                        LE(
                            activity_r.end,
                            activity_h.start,
                        ),
                        GE(
                            activity_r.start,
                            activity.end,
                        ),
                    ),
                )
            )
        cc0.append(
            And(
                activity_h.present,
                LE(activity_h.end, activity.start),
                And(cc1),
            )
        )

    if is_initially_applicable:
        cc1 = []
        for activity_r in rival_activities:
            assert activity != activity_r

            cc1.append(
                Implies(
                    activity_r.present,
                    GE(activity_r.start, activity.end),
                )
            )
        cc0.append(And(cc1))

    return Or(cc0)


def no_overlap(
    activity1: Activity, activity2: Activity, em: ExpressionManager
) -> FNode:
    return em.Or(
        em.LE(activity1.end, activity2.start), em.GE(activity1.start, activity2.end)
    )


def overlap(activity1: Activity, activity2: Activity, em: ExpressionManager) -> FNode:
    return em.And(
        em.GT(activity1.end, activity2.start), em.GT(activity2.end, activity1.start)
    )


def enforce_activity_non_overlap_constraints(
    activities: List[Activity], problem: SchedulingProblem
):
    em = problem.environment.expression_manager
    for i, activity1 in enumerate(activities):
        for j, activity2 in enumerate(activities):
            if i < j:  # Avoid duplicate pairs and self-comparison
                problem.add_constraint(
                    no_overlap(activity1, activity2, em),
                    scope=[activity1.present, activity2.present],
                )
