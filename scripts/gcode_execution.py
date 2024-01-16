import rospy
import rospkg

from gcodeparser import GcodeParser
from pyrobopath.toolpath import *
from pyrobopath_ros import ScheduleExecution, toolpath_from_gcode
from pyrobopath.toolpath_scheduling import animate_multi_agent_toolpath_full

NAME = "gcode_execution_demo"
#GCODE_PATH = "/resources/multi_tool_square_reprap.gcode"
#GCODE_PATH = "/resources/multi_tool_demo.gcode"
GCODE_PATH = "/resources/GT_Logo.gcode"

LIMITS = ((-1.0, 1.0), (-1.0, 1.0))

class GcodeExecutionDemo:
    def __init__(self):
        self.sched_exec = ScheduleExecution()

    def run(self):
        rospack = rospkg.RosPack()
        filepath = rospack.get_path("hydra_sandbox") + GCODE_PATH
        toolpath = toolpath_from_gcode(filepath)
        self.filter_toolpath(toolpath)

        self.sched_exec.move_home()
        # schedule toolpath
        self.sched_exec.plan_toolpath(toolpath)
        
        agent_models = {id: context.agent for id, context in self.sched_exec._contexts.items()}
        animate_multi_agent_toolpath_full(toolpath, self.sched_exec._schedule, agent_models, limits=LIMITS)
        self.sched_exec.execute_plan()

    def filter_toolpath(self, toolpath: Toolpath):
        toolpath.scale(0.001)
        toolpath.contours = toolpath.contours[:30]

if __name__ == "__main__":
    rospy.init_node(NAME)
    try:
        demo = GcodeExecutionDemo()
        demo.run()
    except rospy.ROSInterruptException:
        pass
