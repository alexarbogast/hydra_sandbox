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

import numpy as np
import quaternion
from copy import copy
from enum import Enum
from dataclasses import dataclass
from collections import defaultdict

# ros
import rospy
import actionlib
import tf2_ros

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

# pyrobopath
from pyrobopath.toolpath import Toolpath, Contour
from pyrobopath.collision_detection import FCLRobotBBCollisionModel
from pyrobopath.scheduling import DependencyGraph
from pyrobopath.toolpath_scheduling import *

from cartesian_planning_server.srv import *

NAME = "schedule_execution_demo"
HOME_POSITION = [0.0, 0.53, 0.47, 0.0, -1.0, 0.0]
TRAVEL_VEL = 0.500
CONTOUR_VEL = 0.300


class Materials(Enum):
    MATERIAL_A = 1
    MATERIAL_B = 2


def raster_rect(p, h, spacing, n):
    pi = np.array(p)
    raster = [copy(pi)]
    dir = 1.0
    for _ in range(n):
        pi[0] = pi[0] + (h * dir)
        raster.append(copy(pi))
        pi[1] = pi[1] + spacing
        dir *= -1
        raster.append(copy(pi))
    pi[0] = pi[0] + (h * dir)
    raster.append(copy(pi))
    return raster


def rotate_pathZ(path, about, rad):
    about = np.array(about)
    new_path = [p - about for p in path]
    s, c = np.sin(rad), np.cos(rad)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    new_path = [(R @ p) + about for p in new_path]
    return new_path


def create_example_toolpath() -> Toolpath:
    # Layer 1
    path1 = raster_rect([-0.150, -0.150, 1.0], 0.3, 0.020, 7)
    path2 = raster_rect([-0.150, 0.01, 1.0], 0.3, 0.020, 7)
    path2.reverse()

    # Layer 2
    path3 = raster_rect([-0.150, 0.01, 1.02], 0.14, 0.020, 7)
    path4 = raster_rect([0.01, 0.150, 1.02], 0.3, 0.020, 7)
    path4 = rotate_pathZ(path4, [0.01, 0.150, 1.02], -np.pi / 2)
    path5 = raster_rect([-0.150, -0.150, 1.02], 0.14, 0.020, 7)

    # Layer 3
    path6 = raster_rect([0.150, 0.150, 1.04], 0.200, 0.020, 5)
    path6 = rotate_pathZ(path6, [0.150, 0.150, 1.04], -np.pi)
    path7 = raster_rect([0.150, 0.03, 1.04], 0.200, 0.020, 5)
    path7 = rotate_pathZ(path7, [0.150, 0.03, 1.04], -np.pi)
    path8 = raster_rect([-0.150, -0.070, 1.04], -0.220, 0.020, 4)
    path8 = rotate_pathZ(path8, [-0.150, -0.070, 1.04], -np.pi / 2)
    path9 = raster_rect([-0.150, -0.150, 1.04], 0.300, 0.020, 3)

    c1 = Contour(path1, tool=Materials.MATERIAL_A)
    c2 = Contour(path2, tool=Materials.MATERIAL_B)
    c3 = Contour(path3, tool=Materials.MATERIAL_B)
    c4 = Contour(path4, tool=Materials.MATERIAL_A)
    c5 = Contour(path5, tool=Materials.MATERIAL_A)
    c6 = Contour(path6, tool=Materials.MATERIAL_B)
    c7 = Contour(path7, tool=Materials.MATERIAL_A)
    c8 = Contour(path8, tool=Materials.MATERIAL_B)
    c9 = Contour(path9, tool=Materials.MATERIAL_A)

    toolpath = Toolpath()
    toolpath.contours = [c1, c2, c3, c4, c5, c6, c7, c8, c9]

    dg = DependencyGraph()
    dg.add_node("start")
    dg.add_node(0, ["start"])
    dg.add_node(1, ["start"])
    dg.add_node(2, [0])
    dg.add_node(3, [0, 1])
    dg.add_node(4, [1])
    dg.add_node(5, [2, 3])
    dg.add_node(6, [2, 3, 4])
    dg.add_node(7, [2, 4])
    dg.add_node(8, [3, 4])

    return toolpath, dg


def transform_tf_to_np(transform):
    p = transform.translation
    q = transform.rotation

    p = np.array([p.x, p.y, p.z])
    q = np.quaternion(q.w, q.x, q.y, q.z)

    np_tf = np.identity(4, dtype=np.float64)
    np_tf[:3, :3] = quaternion.as_rotation_matrix(q)
    np_tf[:3, 3] = p
    return np_tf


class AgentExecutionContext(object):
    def __init__(self, id, tf_buffer: tf2_ros.Buffer, capabilities):
        self.id = id
        try:
            base_to_world = tf_buffer.lookup_transform(
                "world", f"{self.id}_base_link", rospy.Time()
            )
            world_to_base = tf_buffer.lookup_transform(
                f"{self.id}_base_link", "world", rospy.Time()
            )
            eef_to_world = tf_buffer.lookup_transform(
                f"world", f"{self.id}_flange", rospy.Time()
            )
        except:
            rospy.logfatal(f"Failed to find transforms for agent {id}")

        self.world_to_base = transform_tf_to_np(world_to_base.transform)
        self.base_to_world = transform_tf_to_np(base_to_world.transform)
        self.eef_to_world = transform_tf_to_np(eef_to_world.transform)

        self.agent = AgentModel()
        self.agent.base_frame_position = self.base_to_world[:3, 3]
        self.agent.home_position = self.eef_to_world[:3, 3]
        self.agent.capabilities = capabilities
        self.agent.collision_model = FCLRobotBBCollisionModel(
            0.50, 0.1, 2.0, self.agent.base_frame_position
        )

        self.planning_client = rospy.ServiceProxy(
            f"{self.id}/cartesian_planning_server/plan_cartesian_trajectory",
            PlanCartesianTrajectory,
        )
        self.action_client = actionlib.SimpleActionClient(
            f"{self.id}/position_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )

    def update_home_tf(self, tf_buffer):
        try:
            eef_to_world = tf_buffer.lookup_transform(
                f"world", f"{self.id}_flange", rospy.Time()
            )
        except:
            rospy.logfatal(f"Failed to find transforms for agent {self.id}")
        self.eef_to_world = transform_tf_to_np(eef_to_world.transform)
        self.agent.home_position = self.eef_to_world[:3, 3]


class ScheduleExecutionDemo(object):
    def __init__(self):
        # temporary: find better way to do base frame transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.sleep(0.1)

        self.contexts = {
            "rob1": AgentExecutionContext(
                "rob1", self.tf_buffer, [Materials.MATERIAL_A]
            ),
            "rob2": AgentExecutionContext(
                "rob2", self.tf_buffer, [Materials.MATERIAL_B]
            ),
        }

        rospy.loginfo("Waiting for plan_cartesian_trajectory servers...")
        for context in self.contexts.values():
            context.planning_client.wait_for_service()

        rospy.loginfo("Waiting for follow_trajectory_action servers...")
        for context in self.contexts.values():
            context.action_client.wait_for_server()

        self._schedule_plan_buffer = defaultdict(list)
        rospy.loginfo("Ready to plan!")

    def initialize_pyrobopath(self):
        agent_models = {id: context.agent for id, context in self.contexts.items()}
        self._planner = MultiAgentToolpathPlanner(agent_models)
        self._options = PlanningOptions(
            travel_velocity=TRAVEL_VEL,
            contour_velocity=CONTOUR_VEL,
            retract_height=0.1,
            collision_offset=1.0,
            collision_gap_threshold=0.003,
        )

    def run(self):
        """move all robots home"""
        self.move_home()
        for context in self.contexts.values():
            context.update_home_tf(self.tf_buffer)
        self.initialize_pyrobopath()

        """ Schedule multi agent toolpath """
        schedule = self.create_and_plan_toolpath()

        """ Cartesian motion planning for schedule events """
        self.plan_multi_agent_schedule(schedule)

        """ Execute schedule """
        self.execute_schedule()

        # wait for completion
        for id in self.contexts.keys():
            self.contexts[id].action_client.wait_for_result()

    def move_home(self):
        # send trajectories to home
        for id in self.contexts.keys():
            start_state = rospy.wait_for_message(f"/{id}/joint_states", JointState)

            point_start = JointTrajectoryPoint()
            point_start.positions = start_state.position
            point_start.time_from_start = rospy.Duration(0.0)

            point_goal = JointTrajectoryPoint()
            point_goal.positions = HOME_POSITION
            point_goal.time_from_start = rospy.Duration(2.0)

            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = start_state.name
            goal.trajectory.points = [point_start, point_goal]
            self.contexts[id].action_client.send_goal(goal)

        # wait for completion
        for id in self.contexts.keys():
            self.contexts[id].action_client.wait_for_result()

    def create_and_plan_toolpath(self):
        """create and plan toolpath schedule"""
        toolpath, _ = create_example_toolpath()
        dg = create_dependency_graph_by_layers(toolpath)

        rospy.loginfo(
            f"\n{(50 * '#')}\nScheduling Simple Two Materal Toolpath:\n{(50 * '#')}\n"
        )
        schedule = self._planner.plan(toolpath, dg, self._options)
        rospy.loginfo(f"\n{(50 * '#')}\nFound Toolpath Plan!\n{(50 * '#')}\n")

        #animate_multi_agent_toolpath_full(
        #   toolpath,
        #   schedule,
        #   self._planner._agent_models,
        #   0.01,
        #   limits=((-5, 5), (-4, 4)),
        #)
        return schedule

    def plan_multi_agent_schedule(self, schedule: MultiAgentToolpathSchedule):
        """Populates the schedule plan buffer with motion plans from each
        event in `schedule`.
        """
        rospy.loginfo(f"\n{(50 * '#')}\nPlanning events\n{(50 * '#')}\n")
        rospy.loginfo("Planning and buffering events in schedule")
        for agent, sched in schedule.schedules.items():
            start_state = rospy.wait_for_message(f"/{agent}/joint_states", JointState)
            for event in sched._events:
                resp = self.plan_event(event, agent, start_state)

                # create trajectory action server goal
                goal = FollowJointTrajectoryGoal()
                goal.trajectory = resp.trajectory
                self._schedule_plan_buffer[agent].append((event.start, goal))

                start_state.position = resp.trajectory.points[-1].positions
                start_state.velocity = resp.trajectory.points[-1].velocities

    def plan_event(
        self, event: MoveEvent, agent, start_state: JointState
    ) -> PlanCartesianTrajectoryResponse:
        context = self.contexts[agent]
        path_base = [
            (context.world_to_base @ np.array([*p, 1]))[:3] for p in event.data
        ]
        req = PlanCartesianTrajectoryRequest()
        req.start_state = start_state

        for point in path_base:
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

        req.velocity = CONTOUR_VEL if isinstance(event, ContourEvent) else TRAVEL_VEL
        resp = None
        try:
            resp = context.planning_client(req)
        except rospy.ServiceException as e:
            rospy.logerror("Failed to plan cartesian trajectory: " + str(e))
        return resp

    def execute_schedule(self):
        rospy.loginfo(f"\n\n{(50 * '#')}\nExecuting Schedule\n{(50 * '#')}\n")
        start_time = rospy.get_time()
        rate = rospy.Rate(10)
        while any(self._schedule_plan_buffer.values()) and not rospy.is_shutdown():
            now = rospy.get_time()
            for agent, plans in self._schedule_plan_buffer.items():
                if not plans or now - start_time < plans[0][0]:
                    continue

                _, jt_goal = self._schedule_plan_buffer[agent].pop(0)
                rospy.loginfo(f"Starting event for {agent}")
                self.contexts[agent].action_client.send_goal(jt_goal)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node(NAME)
    try:
        demo = ScheduleExecutionDemo()
        demo.run()
    except rospy.ROSInteruptException:
        pass
