#!/usr/bin/env python3
# Copyright 2024 Alex Arbogast
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import numpy as np
from copy import copy

from pyrobopath.toolpath import Toolpath, Contour
from pyrobopath_ros import ScheduleExecution
from pyrobopath.toolpath_scheduling import animate_multi_agent_toolpath_full

NAME = "sync_testing"
LIMITS = ((-1.0, 1.0), (-1.0, 1.0))


class SyncTesting:
    def __init__(self):
        self.sched_exec = ScheduleExecution()

    def run(self):
        # read gcode to filtered toolpath
        toolpath = self.create_toolpath()

        # move all robots home
        self.sched_exec.move_home()

        # schedule toolpath
        self.sched_exec.schedule_toolpath(toolpath)

        animate_multi_agent_toolpath_full(
            toolpath,
            self.sched_exec._schedule,
            self.sched_exec.agent_models,
            limits=LIMITS,
        )

        # execute schedule
        self.sched_exec.execute_schedule()
        rospy.spin()

    def create_toolpath(self) -> Toolpath:
        height = -0.005
        # return two_lines(height)
        # return three_lines(height)
        # return zig_zags(height)
        return coordinate_frame(height)


def two_lines(h) -> Toolpath:
    toolpath = Toolpath()

    p1 = np.array([[0.3, 0.3, h], [-0.3, 0.3, h]])
    p2 = np.array([[0.3, -0.3, h], [-0.3, -0.3, h]])

    c1 = Contour(p1, 0)
    c2 = Contour(p2, 1)

    toolpath.contours = [c1, c2]
    return toolpath


def three_lines(h) -> Toolpath:
    toolpath = Toolpath()

    p1 = np.array([[0.3, 0.3, h], [-0.3, 0.3, h]])
    p2 = np.array([[0.3, -0.3, h], [-0.3, -0.3, h]])
    p3 = np.array([[0.3, -0.3, h + 0.001], [-0.3, -0.3, h + 0.001]])

    c1 = Contour(p1, 0)
    c2 = Contour(p2, 1)
    c3 = Contour(p3, 1)

    toolpath.contours = [c1, c2, c1, c3]
    return toolpath


def coordinate_frame(h) -> Toolpath:
    toolpath = Toolpath()

    # x_axis = np.array([[0.0, 0.0, h], [0.05, 0.0, h]])
    # y_axis = np.array([[0.0, 0.0, h], [0.0, 0.05, h]])

    x_axis = np.array([[-0.250, 0.0, h], [-0.2, 0.0, h]])
    y_axis = np.array([[-0.25, 0.0, h], [-0.25, 0.05, h]])

    c1 = Contour(x_axis, 0)
    c2 = Contour(y_axis, 0)
    c3 = Contour(x_axis, 1)
    c4 = Contour(y_axis, 1)

    toolpath.contours = [c1, c2, c3, c4]
    return toolpath


def zig_zags(h) -> Toolpath:
    toolpath = Toolpath()

    p11 = raster_rect([0.3, 0.2, h], 0.03, -0.05, 12)
    p12 = raster_rect([-0.3, 0.2, h], 0.03, 0.05, 12)
    p21 = raster_rect([0.3, -0.2, h], 0.03, -0.05, 12)
    p22 = raster_rect([-0.3, -0.2, h], 0.03, 0.05, 12)

    c1 = Contour(p11, 0)
    c2 = Contour(p12, 0)
    c3 = Contour(p21, 1)
    c4 = Contour(p22, 1)

    toolpath.contours = [c1, c2, c3, c4]
    return toolpath


def raster_rect(p, h, spacing, n):
    pi = np.array(p)
    raster = [copy(pi)]
    dir = 1.0
    for _ in range(n):
        pi[1] = pi[1] + (h * dir)
        raster.append(copy(pi))
        pi[0] = pi[0] + spacing
        dir *= -1
        raster.append(copy(pi))
    pi[1] = pi[1] + (h * dir)
    raster.append(copy(pi))
    return raster


if __name__ == "__main__":
    rospy.init_node(NAME)
    try:
        app = SyncTesting()
        app.run()
    except rospy.ROSInterruptException:
        pass
