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

import os
import numpy as np

import rclpy
from ament_index_python.packages import get_package_share_directory

from pyrobopath.toolpath import Toolpath
from pyrobopath.toolpath.preprocessing import *
from pyrobopath.toolpath_scheduling import animate_multi_agent_toolpath_full

from pyrobopath_execution import ScheduleExecution, toolpath_from_gcode

NAME = "pyrobopath_demo"

PART = "GT_Logo"
Z_HEIGHT = 0.000  # mm

filepaths = {
    "multi_tool_square": "resources/multi_tool_square.gcode",
    "multi_tool_demo": "resources/multi_tool_demo.gcode",
    "GT_Logo": "resources/GT_Logo.gcode",
    "gear": "resources/gear.gcode",
    "mona_lisa": "resources/mona_lisa.gcode",  # single material
    "foresight_logo": "resources/foresight_logo.gcode",
    "part1_2xbracket": "resources/part1_2Xbracket.gcode",  # single material
    "part6_mold_insert": "resources/part6_mold_insert.gcode",
}


LIMITS = ((-1.0, 1.0), (-1.0, 1.0))


class GcodeExecutionDemo(ScheduleExecution):
    def __init__(self):
        super().__init__(NAME)

    def run(self):
        # read gcode to filtered toolpath
        toolpath = self.load_toolpath()

        # move all robots home
        self.move_home()

        # schedule toolpath
        self.schedule_toolpath(toolpath)

        animate_multi_agent_toolpath_full(
            toolpath,
            self.schedule,
            self.agent_models,
            limits=LIMITS,
        )

        # execute schedule
        self.execute_schedule()

    def load_toolpath(self) -> Toolpath:
        pkg_share = get_package_share_directory("hydra_sandbox")
        self.get_logger().info(pkg_share)
        filepath = os.path.join(pkg_share, filepaths[PART])
        self.get_logger().info(filepath)

        toolpath = toolpath_from_gcode(filepath)
        self.preprocess_toolpath(toolpath)
        return toolpath

    def preprocess_toolpath(self, toolpath: Toolpath):
        preprocessor = ToolpathPreprocessor()
        preprocessor.add_step(ScalingStep(0.001))  # to meters

        # extract desired layers
        if PART == "multi_tool_square":
            preprocessor.add_step(LayerRangeStep(0, 1))
        elif PART == "multi_tool_demo":
            preprocessor.add_step(ScalingStep(0.9))
            preprocessor.add_step(LayerRangeStep(0, 1))
        elif PART == "GT_Logo":
            preprocessor.add_step(ScalingStep(0.75))
            preprocessor.add_step(LayerRangeStep(0, 1))
        elif PART == "gear":
            preprocessor.add_step(LayerRangeStep(0, 1))
        elif PART == "mona_lisa":
            preprocessor.add_step(ScalingStep(0.9))
        elif PART == "foresight_logo":
            preprocessor.add_step(LayerRangeStep(0, 1))
        elif PART == "part1_2xbracket":
            preprocessor.add_step(RotateStep(Rotation.Rz(np.pi / 2)))
        elif PART == "part6_mold_insert":
            preprocessor.add_step(RotateStep(Rotation.Rz(np.pi / 2)))

        # adjust z height
        preprocessor.add_step(TranslateStep([0.0, 0.0, Z_HEIGHT]))
        preprocessor.process(toolpath)


if __name__ == "__main__":
    rclpy.init()

    node = GcodeExecutionDemo()
    node.run()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()
