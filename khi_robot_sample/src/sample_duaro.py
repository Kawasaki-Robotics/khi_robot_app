#!/usr/bin/env python
# coding:utf-8

# Software License Agreement (BSD License)
#
#  Copyright (c) 2019, Kawasaki Heavy Industries, LTD.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Willow Garage nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

import copy
import genpy
import math
import tf
import rospkg
import rospy

from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from visualization_msgs.msg import Marker
from geometry_msgs.msg import *
from moveit_msgs.msg import *

def publish_text(pub, text):
    marker_data = Marker()
    marker_data.header.frame_id = "base_link"
    marker_data.header.stamp = rospy.Time.now()

    marker_data.ns = ""
    marker_data.id = 0

    marker_data.action = Marker.ADD

    marker_data.pose.position.x = 1.0
    marker_data.pose.position.y = 0.0
    marker_data.pose.position.z = 1.0

    marker_data.pose.orientation.x=0.0
    marker_data.pose.orientation.y=0.0
    marker_data.pose.orientation.z=1.0
    marker_data.pose.orientation.w=0.0

    marker_data.color.r = 1.0
    marker_data.color.g = 0.0
    marker_data.color.b = 0.0
    marker_data.color.a = 1.0

    marker_data.scale.x = 1
    marker_data.scale.y = 0.1
    marker_data.scale.z = 0.1

    marker_data.lifetime = rospy.Duration()

    marker_data.type = Marker.TEXT_VIEW_FACING
    marker_data.text = text

    pub.publish(marker_data)

def show_text(text):
    pub = rospy.Publisher("visualization_marker", Marker, queue_size = 10)
    rate = rospy.Rate(25)

    for i in range(10):
        publish_text(pub,text)
        rate.sleep()

    rospy.loginfo(text)

def setup_environment(lower_arm, upper_arm, botharms):
    # add object to Planning Scene
    rospy.loginfo( "Planning Scene Settings")
    scene = PlanningSceneInterface()
    rospy.sleep(2)   # Waiting for PlanningSceneInterface
    scene.remove_world_object()

    # upper tool
    box_pose = PoseStamped()
    box_pose.header.frame_id = upper_arm.get_planning_frame()
    pos = upper_arm.get_current_pose()

    rot_o = 90.0/180.0*math.pi
    rot_a = 0.0/180.0*math.pi
    rot_t = -90.0/180.0*math.pi

    q = tf.transformations.quaternion_from_euler(rot_o, rot_a, rot_t, 'szyx')

    # setting origin of upper tool
    box_pose.pose.position.x = pos.pose.position.x
    box_pose.pose.position.y = pos.pose.position.y
    box_pose.pose.position.z = pos.pose.position.z
    box_pose.pose.orientation.x = q[0]
    box_pose.pose.orientation.y = q[1]
    box_pose.pose.orientation.z = q[2]
    box_pose.pose.orientation.w = q[3]

    rospack = rospkg.RosPack()
    resourcepath = rospack.get_path('khi_robot_sample')+"/config/work/"

    meshpath = resourcepath + "upper_tool_001.stl"
    scene.attach_mesh('upper_link_j4', 'upper_tool', box_pose, meshpath, (1,1,1),['upper_link_j3','upper_link_j4'])
    rospy.sleep(1)

    rospy.loginfo( "Planning Scene Settings Finish")

def sample_motion(lower_arm, upper_arm, botharms):
    # environment
    cycle_num = 2
    sleep_time = 0.2
    velocity = 1.0
    acceleration = 1.0
    position_accuracy = 0.01
    orientation_accuracy = 0.01
    planner = 'RRTConnectkConfigDefault'

    # lower_arm
    show_text("Sample duAro: lower_arm")
    lower_arm.set_max_velocity_scaling_factor(velocity)
    lower_arm.set_max_acceleration_scaling_factor(acceleration)
    lower_arm.set_planner_id(planner)
    lower_arm.set_goal_position_tolerance(position_accuracy)
    lower_arm.set_goal_position_tolerance(orientation_accuracy)
    lower_path_constraints = Constraints()
    lower_path_constraints.name = "lower_arm"
    lower_joint_constraint = JointConstraint()
    lower_joint_constraint.joint_name = "lower_joint1"
    lower_joint_constraint.position =  -0.78
    lower_joint_constraint.tolerance_above = 0.1
    lower_joint_constraint.tolerance_below = 0.1
    lower_joint_constraint.weight = 1
    lower_path_constraints.joint_constraints.append(lower_joint_constraint)
    lower_joint_constraint = JointConstraint()
    lower_joint_constraint.joint_name = "lower_joint2"
    lower_joint_constraint.position =  0.78
    lower_joint_constraint.tolerance_above = 0.1
    lower_joint_constraint.tolerance_below = 0.1
    lower_joint_constraint.weight = 1
    lower_path_constraints.joint_constraints.append(lower_joint_constraint)
    lower_arm.set_path_constraints(lower_path_constraints)

    # go home by current joint
    l_jt1 = [-0.78, 0.78, 0.14, 0.0]
    lower_arm.set_joint_value_target(l_jt1)
    show_text("Sample duAro: lower_arm\nl_jt1")
    lower_arm.go()
    # go by current pose
    l_pos1 = lower_arm.get_current_pose()
    l_pos1.pose.position.z = l_pos1.pose.position.z - 0.1
    for i in range(cycle_num):
        show_text("Sample duAro: lower_arm\nl_pos1")
        lower_arm.set_pose_target(l_pos1)
        lower_arm.go()
        l_pos2 = lower_arm.get_current_pose()
        l_pos2.pose.position.z = l_pos2.pose.position.z + 0.1
        lower_arm.set_pose_target(l_pos2)
        show_text("Sample duAro: lower_arm\nl_pos2")
        lower_arm.go()

        # certasian path
        l_waypoints = []
        l_waypoints.append(lower_arm.get_current_pose().pose)

        l_wpos = geometry_msgs.msg.Pose()
        l_wpos.orientation.w = 1.0
        l_wpos.position.x = l_waypoints[0].position.x + 0.1
        l_wpos.position.y = l_waypoints[0].position.y
        l_wpos.position.z = l_waypoints[0].position.z - 0.1
        l_waypoints.append(copy.deepcopy(l_wpos))

        l_wpos.position.x = l_waypoints[1].position.x - 0.1
        l_wpos.position.y = l_waypoints[1].position.y
        l_wpos.position.z = l_waypoints[1].position.z + 0.1
        l_waypoints.append(copy.deepcopy(l_wpos))

        l_wpos.position.x = l_waypoints[2].position.x - 0.1
        l_wpos.position.y = l_waypoints[2].position.y
        l_wpos.position.z = l_waypoints[2].position.z - 0.1
        l_waypoints.append(copy.deepcopy(l_wpos)) 

        l_wpos.position.x = l_waypoints[3].position.x + 0.1
        l_wpos.position.y = l_waypoints[3].position.y
        l_wpos.position.z = l_waypoints[3].position.z + 0.1
        l_waypoints.append(copy.deepcopy(l_wpos))

        (plan, fraction) = lower_arm.compute_cartesian_path(l_waypoints, 0.01, 0.0)
        show_text("Sample duAro: lower_arm\nl_waypoints")
        lower_arm.execute(plan)

    # upper_arm
    show_text("Sample duAro: upper_arm")
    upper_arm.set_max_velocity_scaling_factor(velocity)
    upper_arm.set_max_acceleration_scaling_factor(acceleration)
    upper_arm.set_planner_id(planner)
    upper_arm.set_goal_position_tolerance(position_accuracy)
    upper_arm.set_goal_position_tolerance(orientation_accuracy)
    upper_path_constraints = Constraints()
    upper_path_constraints.name = "upper_arm"
    upper_joint_constraint = JointConstraint()
    upper_joint_constraint.joint_name = "upper_joint1"
    upper_joint_constraint.position =  2.34
    upper_joint_constraint.tolerance_above = 0.1
    upper_joint_constraint.tolerance_below = 0.1
    upper_joint_constraint.weight = 1
    upper_path_constraints.joint_constraints.append(upper_joint_constraint)
    upper_joint_constraint = JointConstraint()
    upper_joint_constraint.joint_name = "upper_joint2"
    upper_joint_constraint.position =  -0.78
    upper_joint_constraint.tolerance_above = 0.1
    upper_joint_constraint.tolerance_below = 0.1
    upper_joint_constraint.weight = 1
    upper_path_constraints.joint_constraints.append(upper_joint_constraint)
    upper_arm.set_path_constraints(upper_path_constraints)

    # go home by current joint
    u_jt1 = [2.34, -0.78, 0.14, 0.0]
    upper_arm.set_joint_value_target(u_jt1)
    show_text("Sample duAro: upper_arm\nu_jt1")
    upper_arm.go()
    # go by current pose
    u_pos1 = upper_arm.get_current_pose()
    u_pos1.pose.position.z = u_pos1.pose.position.z - 0.1
    for i in range(cycle_num):
        upper_arm.set_pose_target(u_pos1)
        show_text("Sample duAro: upper_arm\nu_pos1")
        upper_arm.go()
        u_pos2 = upper_arm.get_current_pose()
        u_pos2.pose.position.z = u_pos2.pose.position.z + 0.1
        upper_arm.set_pose_target(u_pos2)
        show_text("Sample duAro: upper_arm\nu_pos2")
        upper_arm.go()

        # certasian path
        u_waypoints = []
        u_waypoints.append(upper_arm.get_current_pose().pose)

        u_wpos = geometry_msgs.msg.Pose()
        u_wpos.orientation.w = 1.0
        u_wpos.position.x = u_waypoints[0].position.x
        u_wpos.position.y = u_waypoints[0].position.y + 0.1
        u_wpos.position.z = u_waypoints[0].position.z - 0.1
        u_waypoints.append(copy.deepcopy(u_wpos))

        u_wpos.position.x = u_waypoints[1].position.x
        u_wpos.position.y = u_waypoints[1].position.y - 0.1
        u_wpos.position.z = u_waypoints[1].position.z + 0.1
        u_waypoints.append(copy.deepcopy(u_wpos))

        u_wpos.position.x = u_waypoints[2].position.x
        u_wpos.position.y = u_waypoints[2].position.y - 0.1
        u_wpos.position.z = u_waypoints[2].position.z - 0.1
        u_waypoints.append(copy.deepcopy(u_wpos)) 

        u_wpos.position.x = u_waypoints[3].position.x
        u_wpos.position.y = u_waypoints[3].position.y + 0.1
        u_wpos.position.z = u_waypoints[3].position.z + 0.1
        u_waypoints.append(copy.deepcopy(u_wpos)) 

        (plan, fraction) = upper_arm.compute_cartesian_path(u_waypoints, 0.01, 0.0)
        show_text("Sample duAro: upper_arm\nu_waypoints")
        upper_arm.execute(plan)

    # botharms
    show_text("Sample duAro: botharms")
    botharms.set_max_velocity_scaling_factor(velocity)
    botharms.set_max_acceleration_scaling_factor(acceleration)
    botharms.set_planner_id(planner)
    botharms.set_goal_position_tolerance(position_accuracy)
    botharms.set_goal_position_tolerance(orientation_accuracy)

    for i in range(cycle_num):
        # go by joint value
        botharms.set_max_velocity_scaling_factor(velocity*0.2)
        b_jt1 = [-0.78, 0.78, 0.14, 0.0, 0.78, -0.78, 0.14, 0.0]
        botharms.set_joint_value_target(b_jt1)
        show_text("Sample duAro: botharms\nb_jt1")
        botharms.go()

        # plan and execute by joint value
        botharms.set_max_velocity_scaling_factor(velocity*0.5)
        b_jt2 = botharms.get_current_joint_values()
        b_jt2[0] += -0.78
        b_jt2[1] +=  0.78
        b_jt2[4] +=  0.78
        b_jt2[5] += -0.78
        botharms.set_joint_value_target(b_jt2)
        b_plan2 = botharms.plan()
        show_text("Sample duAro: botharms\nb_jt2")
        botharms.execute(b_plan2)

    botharms.set_joint_value_target(b_jt1)
    show_text("Sample duAro: botharms\nb_jt1")
    botharms.go()

    show_text("")

if __name__ == '__main__':
    rospy.init_node('sample_duaro')

    lower_arm = MoveGroupCommander("lower_arm")
    upper_arm = MoveGroupCommander("upper_arm")
    botharms = MoveGroupCommander("botharms")

    setup_environment(lower_arm, upper_arm, botharms)
    rospy.sleep(2)
    cmd = 's'
    while(cmd == 's'):
        show_text("Sample duAro")
        rospy.loginfo("\n********** Sample duAro s:start q:quit **********")
        cmd = raw_input('-> ')
        if cmd == 's':
            sample_motion(lower_arm, upper_arm, botharms)
