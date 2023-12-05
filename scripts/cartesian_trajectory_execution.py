import numpy as np
import quaternion
from copy import copy

import rospy
import actionlib
import tf2_ros

from control_msgs.msg import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Transform
from trajectory_msgs.msg import JointTrajectoryPoint

from cartesian_planning_server.srv import *
from copy import deepcopy


HOME_POSITION = [0.0, 0.53, 0.47, 0.0, -1.0, 0.0]
NAME = "cartesian_trajectory_execution"
VEL = 0.500


def transform_tf_to_np(transform: Transform):
    p = transform.translation
    q = transform.rotation

    p = np.array([p.x, p.y, p.z])
    q = np.quaternion(q.w, q.x, q.y, q.z)

    np_tf = np.identity(4, dtype=np.float64)
    np_tf[:3, :3] = quaternion.as_rotation_matrix(q)
    np_tf[:3, 3] = p
    return np_tf


class CartesianPlanningDemo(object):
    def __init__(self):
        # temporary: find better way to do base frame transformations
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        rospy.sleep(0.1)

        try:
            rob1_transform_stamped = tf_buffer.lookup_transform(
                "rob1_base_link", "world", rospy.Time()
            )
            rob2_transform_stamped = tf_buffer.lookup_transform(
                "rob2_base_link", "world", rospy.Time()
            )
        except:
            print("Failed to find base_frame transforms")

        self._rob1_transform = transform_tf_to_np(rob1_transform_stamped.transform)
        self._rob2_transform = transform_tf_to_np(rob2_transform_stamped.transform)

        # cartesian planning clients
        self._rob1_planning_client = rospy.ServiceProxy(
            "robot1/cartesian_planning_server/plan_cartesian_trajectory",
            PlanCartesianTrajectory,
        )
        self._rob2_planning_client = rospy.ServiceProxy(
            "robot2/cartesian_planning_server/plan_cartesian_trajectory",
            PlanCartesianTrajectory,
        )

        # trajectory exection clients
        self._rob1_action_client = actionlib.SimpleActionClient(
            "rob1/position_trajectory_controller/follow_joint_trajectory",
            control_msgs.msg.FollowJointTrajectoryAction,
        )
        self._rob2_action_client = actionlib.SimpleActionClient(
            "rob2/position_trajectory_controller/follow_joint_trajectory",
            control_msgs.msg.FollowJointTrajectoryAction,
        )

        rospy.loginfo("Waiting for plan_cartesian_trajectory servers...")
        self._rob1_planning_client.wait_for_service()
        self._rob2_planning_client.wait_for_service()

        rospy.loginfo("Waiting for follow_trajectory_action server...")
        self._rob1_action_client.wait_for_server()
        self._rob2_action_client.wait_for_server()

        rospy.loginfo("Ready to plan!")

    def run(self):
        self.move_home()
        req1 = PlanCartesianTrajectoryRequest()
        req2 = PlanCartesianTrajectoryRequest()

        # set start state
        start_state1 = rospy.wait_for_message("/rob1/joint_states", JointState)
        req1.start_state = start_state1

        start_state2 = rospy.wait_for_message("/rob2/joint_states", JointState)
        req2.start_state = start_state2

        # create paths
        path1 = [np.array([-0.5, -0.25, 1.0]), np.array([0.5, -0.25, 1.0])]
        path1 = self._transform_path(path1, self._rob1_transform)
        req1 = self._add_path_to_request(path1, req1)

        path2 = [np.array([-0.5, 0.25, 1.0]), np.array([0.5, 0.25, 1.0])]
        path2 = self._transform_path(path2, self._rob2_transform)
        req2 = self._add_path_to_request(path2, req2)

        # set velocities
        req1.velocity, req2.velocity = VEL, VEL
        resp1, resp2 = None, None
        try:
            resp1 = self._rob1_planning_client(req1)
            resp2 = self._rob2_planning_client(req2)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to plan cartesian trajectory: " + str(e))
            return

        # create goals
        goal1 = control_msgs.msg.FollowJointTrajectoryGoal()
        goal2 = control_msgs.msg.FollowJointTrajectoryGoal()
        goal1.trajectory = resp1.trajectory
        goal2.trajectory = resp2.trajectory
        self._rob1_action_client.send_goal(goal1)
        self._rob2_action_client.send_goal(goal2)
        self.move_home()

    def move_home(self):
        start_state1 = rospy.wait_for_message("/rob1/joint_states", JointState)
        start_state2 = rospy.wait_for_message("/rob2/joint_states", JointState)

        # create trajectory to home
        point_start1 = JointTrajectoryPoint()
        point_start1.positions = start_state1.position
        point_start1.time_from_start = rospy.Duration(0.0)

        point_start2 = JointTrajectoryPoint()
        point_start2.positions = start_state2.position
        point_start2.time_from_start = rospy.Duration(0.0)

        point_goal = JointTrajectoryPoint()
        point_goal.positions = HOME_POSITION
        point_goal.time_from_start = rospy.Duration(2.0)
        # send trajectory to action server
        goal1 = FollowJointTrajectoryGoal()
        goal1.trajectory.joint_names = start_state1.name
        goal1.trajectory.points = [point_start1, point_goal]
        goal2 = FollowJointTrajectoryGoal()
        goal2.trajectory.joint_names = start_state2.name
        goal2.trajectory.points = [point_start2, point_goal]
        self._rob1_action_client.send_goal(goal1)
        self._rob2_action_client.send_goal(goal2)

        self._rob1_action_client.wait_for_result()
        self._rob2_action_client.wait_for_result()

    def _transform_path(self, path, transform):
        return [(transform @ np.array([*p, 1]))[:3] for p in path]

    def _add_path_to_request(self, points, req: PlanCartesianTrajectoryRequest):
        for point in points:
            pose = Pose()
            pose.position.x = point[0]
            pose.position.y = point[1]
            pose.position.z = point[2]

            theta = np.arctan2(point[1], point[0])
            pose.orientation.w = np.cos(theta / 2)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = np.sin(theta / 2)
            req.path.append(pose)
        return req


if __name__ == "__main__":
    rospy.init_node(NAME)

    try:
        demo = CartesianPlanningDemo()
        demo.run()
    except rospy.ROSInternalException:
        pass
