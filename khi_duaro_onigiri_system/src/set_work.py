#!/usr/bin/env python
# coding:utf-8

# Software License Agreement (BSD License)
#
#  Copyright (c) 2020, Kawasaki Heavy Industries, LTD.
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
#  POSSIBILITY OF SUCH DAMAG


import sys, copy, math
import rospy, tf
import rospkg, genpy

from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from geometry_msgs.msg import Pose, PoseStamped



def main():
    rospy.init_node('duaro')
    botharms = MoveGroupCommander("botharms")
    upper = MoveGroupCommander("upper_arm")
    lower = MoveGroupCommander("lower_arm")
    duaro = RobotCommander()

    # Add Object to Planning Scene
    rospy.loginfo( "Planning Scene Settings")
    scene = PlanningSceneInterface()
    rospy.sleep(2)   # Waiting for PlanningSceneInterface
    scene.remove_world_object()

    current_robot = duaro.get_current_state()
    current_joint = current_robot.joint_state.position

    rospack = rospkg.RosPack()
    resourcepath = rospack.get_path('khi_duaro_onigiri_system')+"/config/meshes/"

    #conveyer
    box_pose = PoseStamped()
    box_pose.header.frame_id = botharms.get_planning_frame()

    rot_o = 180.0/180.0*math.pi
    rot_a = 0.0/180.0*math.pi
    rot_t = 0.0/180.0*math.pi

    q = tf.transformations.quaternion_from_euler(rot_o, rot_a, rot_t, 'szyx')

    #setting origin of conveyer
    box_pose.pose.position.x = 0.2153
    box_pose.pose.position.y = 0.32454
    box_pose.pose.position.z = 0.0
    box_pose.pose.orientation.x = q[0]
    box_pose.pose.orientation.y = q[1]
    box_pose.pose.orientation.z = q[2]
    box_pose.pose.orientation.w = q[3]

    meshpath = resourcepath + "conveyer.stl"
    scene.add_mesh('conveyer', box_pose, meshpath,(1,1,1) )
    rospy.sleep(3)   # Waiting for setting conveyer

    #stand
    rot_o = 0.0/180.0*math.pi
    rot_a = 0.0/180.0*math.pi
    rot_t = 0.0/180.0*math.pi

    q = tf.transformations.quaternion_from_euler(rot_o, rot_a, rot_t, 'szyx')

    #setting origin of stand
    box_pose.pose.position.x = -0.385
    box_pose.pose.position.y = 0.2342
    box_pose.pose.position.z = 0.0
    box_pose.pose.orientation.x = q[0]
    box_pose.pose.orientation.y = q[1]
    box_pose.pose.orientation.z = q[2]
    box_pose.pose.orientation.w = q[3]

    meshpath = resourcepath + "stand.stl"
    scene.add_mesh('stand', box_pose, meshpath,(1,1,1) )
    rospy.sleep(3)   # Waiting for setting stand
    rospy.loginfo( "Planning Scene Settings Finish")

    #tray
    rot_o = 0.0/180.0*math.pi
    rot_a = 0.0/180.0*math.pi
    rot_t = 0.0/180.0*math.pi

    q = tf.transformations.quaternion_from_euler(rot_o, rot_a, rot_t, 'szyx')

    #setting origin of tray
    box_pose.pose.position.x = -0.58952
    box_pose.pose.position.y = -0.0613
    box_pose.pose.position.z = 0.6375
    box_pose.pose.orientation.x = q[0]
    box_pose.pose.orientation.y = q[1]
    box_pose.pose.orientation.z = q[2]
    box_pose.pose.orientation.w = q[3]

    meshpath = resourcepath + "tray_nooffset.stl"
    scene.add_mesh('tray', box_pose, meshpath,(1,1,1) )
    rospy.loginfo( "Planning Scene Settings Finish")

def change_tray_model(scene,onigiri_offset):
    botharms = MoveGroupCommander("botharms")
    box_pose = PoseStamped()
    box_pose.header.frame_id = botharms.get_planning_frame()
    
    scene.remove_world_object("tray")
    rospack = rospkg.RosPack()
    resourcepath = rospack.get_path('khi_duaro_onigiri_system')+"/config/meshes/"
    
    rot_o = 0.0/180.0*math.pi
    rot_a = 0.0/180.0*math.pi
    rot_t = 0.0/180.0*math.pi

    q = tf.transformations.quaternion_from_euler(rot_o, rot_a, rot_t, 'szyx')

    if onigiri_offset == True:
        meshpath = resourcepath + "tray.stl"
        zoffset = 0.02
        xoffset = 0.0085
        yoffset = -0.005
    else:
        meshpath = resourcepath + "tray_nooffset.stl"
        zoffset = 0.0
        xoffset = 0.0
        yoffset = 0.0
        
    #setting origin of tray
    box_pose.pose.position.x = -0.58952 + xoffset
    box_pose.pose.position.y = -0.0613 + yoffset
    box_pose.pose.position.z = 0.6375 + zoffset
    box_pose.pose.orientation.x = q[0]
    box_pose.pose.orientation.y = q[1]
    box_pose.pose.orientation.z = q[2]
    box_pose.pose.orientation.w = q[3]

    scene.add_mesh('tray', box_pose, meshpath,(1,1,1) )
    rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass