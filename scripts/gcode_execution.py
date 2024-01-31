import rospy
import rospkg

from pyrobopath.toolpath import *
from pyrobopath_ros import ScheduleExecution, toolpath_from_gcode
from pyrobopath.toolpath_scheduling import animate_multi_agent_toolpath_full

NAME = "gcode_execution_demo"
# GCODE_PATH = "/resources/multi_tool_square_reprap.gcode"
# GCODE_PATH = "/resources/multi_tool_demo.gcode"
GCODE_PATH = "/resources/GT_Logo.gcode"

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

        #animate_multi_agent_toolpath_full(
        #    toolpath,
        #    self.sched_exec._schedule,
        #    self.sched_exec.agent_models,
        #    limits=LIMITS,
        #)

        # execute schedule
        self.sched_exec.execute_schedule()
        rospy.spin()

    def load_toolpath(self) -> Toolpath:
        rospack = rospkg.RosPack()
        filepath = rospack.get_path("hydra_sandbox") + GCODE_PATH
        toolpath = toolpath_from_gcode(filepath)
        self.filter_toolpath(toolpath)
        return toolpath

    def filter_toolpath(self, toolpath: Toolpath):
        toolpath.scale(0.001)
        toolpath.contours = toolpath.contours[:5]


if __name__ == "__main__":
    rospy.init_node(NAME)
    try:
        demo = GcodeExecutionDemo()
        demo.run()
    except rospy.ROSInterruptException:
        pass
