import rospy
import rospkg

from pyrobopath.toolpath import *
from pyrobopath_ros import ScheduleExecution, toolpath_from_gcode
from pyrobopath.toolpath_scheduling import animate_multi_agent_toolpath_full

NAME = "gcode_execution_demo"

PART = "GT_Logo"
Z_HEIGHT = -0.005 # mm

filepaths = {
    "multi_tool_square": "/resources/multi_tool_square.gcode",
    "multi_tool_demo": "/resources/multi_tool_demo.gcode",
    "GT_Logo": "/resources/GT_Logo.gcode",
    "gear": "/resources/gear.gcode",
    "mona_lisa": "/resources/mona_lisa.gcode",
}


LIMITS = ((-1.0, 1.0), (-1.0, 1.0))


class GcodeExecutionDemo:
    def __init__(self):
        self.sched_exec = ScheduleExecution()

    def run(self):
        # read gcode to filtered toolpath
        toolpath = self.load_toolpath()

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

    def load_toolpath(self) -> Toolpath:
        rospack = rospkg.RosPack()
        filepath = rospack.get_path("hydra_sandbox") + filepaths[PART]
        toolpath = toolpath_from_gcode(filepath)
        self.filter_toolpath(toolpath)
        return toolpath

    def filter_toolpath(self, toolpath: Toolpath):
        toolpath.scale(0.001)  # to meters

        # extract desired layers
        if PART == "multi_tool_square":
            toolpath.contours = toolpath.contours[:12]
        elif PART == "multi_tool_demo":
            toolpath.contours = toolpath.contours[:29]
        elif PART == "GT_Logo":
            # toolpath.scale(0.9)
            toolpath.contours = toolpath.contours[:14]
        elif PART == "gear":
            toolpath.contours = toolpath.contours[:100]
        elif PART == "mona_lisa":
            toolpath.scale(0.9)

        # adjust z height
        for c in toolpath.contours:
            c.path += np.array([0.0, 0.0, Z_HEIGHT])


if __name__ == "__main__":
    rospy.init_node(NAME)
    try:
        demo = GcodeExecutionDemo()
        demo.run()
    except rospy.ROSInterruptException:
        pass
