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


import rospy
import numpy as np
import math
import tf
import copy

from geometry_msgs.msg import Pose
import moveit_commander
from moveit_msgs.msg import RobotTrajectory

from actionlib import SimpleActionClient
from control_msgs.msg import (FollowJointTrajectoryAction, FollowJointTrajectoryGoal)
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint, JointConstraint

#lower arm path class
class LowerArmPath:
    
    #fix member
    __PLANNER = 'RRTConnectkConfigDefault'
    __GROUP_NAME = "lower_arm"
    __ACTION_NAME = '/duaro_lower_arm_controller/follow_joint_trajectory'
    __MG = moveit_commander.MoveGroupCommander(__GROUP_NAME)
    __ROBOT = moveit_commander.RobotCommander()

    #path constraints
    path_constraints = Constraints()
    
    #set class common parameters
    max_speed = 0.1
    max_acceralation = 0.1
    tolerance = 0.001
    
    def __init__(self, type, goal_state_list, joints_start_state = None ):

        #set joint constraints
        LowerArmPath.__MG.set_path_constraints(LowerArmPath.path_constraints)

        #plan
        self.plan = None
        self.pose_quat_list = list()
        ################planning###############
        #goal_state_list is 1D list
        if isinstance(goal_state_list[0], list) == False:
            if type == "JOINT":#joints interpolation
 
                if len(goal_state_list) < 5:#goal_state_list is joints list
                    self.joints = copy.deepcopy(goal_state_list)
                    self.start_joints = copy.deepcopy(joints_start_state)
                    self.start_joints = self.joints_to_joints(self,self.start_joints)

                    self.pose = None
                    self.joints = self.joints_to_joints(self,self.joints)

                    self.plan_from_joint(self.start_joints ) 
                else:#goal_state_list is pose list
                    self.pose_oat = copy.deepcopy(goal_state_list)
                    self.joints = None

                    self.start_joints = copy.deepcopy(joints_start_state)
                    self.start_joints = self.joints_to_joints(self,self.start_joints)

                    self.pose_quat = self.list_to_pose(self, self.pose_oat)
                    self.plan_from_pose(self.start_joints)

            elif type == "LINEAR":#linear interpolation

                if len(goal_state_list) < 5:#goal_state_list is joints list
                    #cannot plan linear path from joints
                    return False
                else:#goal_state_list is pose list
                    self.pose_oat = copy.deepcopy(goal_state_list)
                    self.joints = None

                    self.start_joints = copy.deepcopy(joints_start_state)
                    self.start_joints = self.joints_to_joints(self,self.start_joints)

                    self.pose_quat_list.append( self.list_to_pose(self, self.pose_oat) )
                    self.plan_cartesian_path(0.1, self.start_joints)

            else:
                return False

        #goal_state_list is not  1D list
        else:
            if type == "JOINT":#joints interpolation
                if len(goal_state_list[0]) < 5:#goal_state_list is joints list
                    # cannot plan from joint waypoints
                    return False
                else:#goal_state_list is pose list
                    self.pose_oat = copy.deepcopy(goal_state_list)
                    self.joints = None

                    self.start_joints = copy.deepcopy(joints_start_state)
                    self.start_joints = self.joints_to_joints(self,self.start_joints)

                    for i in range(0,len(goal_state_list)):
                        self.pose_quat_list.append( self.list_to_pose(self, self.pose_oat[i]) )

                    self.plan_cartesian_path(1.0, self.start_joints)

            elif type == "LINEAR":#linear interpolation

                if len(goal_state_list[0]) < 5:#goal_state_list is joints list
                    # cannot plan from joint waypoints
                    return False
                else:#goal_state_list is pose list
                    self.pose_oat = copy.deepcopy(goal_state_list)
                    self.joints = None

                    self.start_joints = copy.deepcopy(joints_start_state)
                    self.start_joints = self.joints_to_joints(self,self.start_joints)

                    for i in range(0,len(goal_state_list)):
                        self.pose_quat_list.append( self.list_to_pose(self, self.pose_oat[i]) )

                    self.plan_cartesian_path(0.01, self.start_joints)

            else:
                return False
        
        # Temporary workaround of planner's issue similar to https://github.com/tork-a/rtmros_nextage/issues/170
        LowerArmPath.__MG.set_planner_id(self.__PLANNER)
        # Allow replanning to increase the odds of a solution
        LowerArmPath.__MG.allow_replanning(True)
        
        #ActionClient is class variable because should make only one instance.
        LowerArmPath.client = SimpleActionClient(
            LowerArmPath.__ACTION_NAME,
            FollowJointTrajectoryAction)
        
        LowerArmPath.client.wait_for_server()

        self.start_state = self.__ROBOT.get_current_state()

    def plan_from_joint(self, joints_start_state = None):
        rospy.loginfo( "Planning Group : %s",LowerArmPath.__GROUP_NAME )

        LowerArmPath.__MG.set_max_velocity_scaling_factor(LowerArmPath.max_speed)
        LowerArmPath.__MG.set_max_acceleration_scaling_factor(LowerArmPath.max_acceralation)
        LowerArmPath.__MG.set_goal_orientation_tolerance(LowerArmPath.tolerance)
        LowerArmPath.__MG.set_goal_position_tolerance(LowerArmPath.tolerance)
        
        LowerArmPath.__MG.set_joint_value_target(self.joints)
        self.start_state = self.__ROBOT.get_current_state()
        LowerArmPath.__MG.set_start_state_to_current_state()

        if joints_start_state != None:
            tpl_joints_start_state = tuple(joints_start_state)
            self.start_state.joint_state.position = tpl_joints_start_state[0:4] + self.start_state.joint_state.position[4:8:1]
            LowerArmPath.__MG.set_start_state(self.start_state)

        self.plan = LowerArmPath.__MG.plan()
        return self.check_plan()


    def plan_from_pose(self, joints_start_state = None):
        rospy.loginfo( "Planning Group : %s",self.__GROUP_NAME )

        LowerArmPath.__MG.set_max_velocity_scaling_factor(LowerArmPath.max_speed)
        LowerArmPath.__MG.set_max_acceleration_scaling_factor(LowerArmPath.max_acceralation)
        LowerArmPath.__MG.set_goal_orientation_tolerance(LowerArmPath.tolerance)
        LowerArmPath.__MG.set_goal_position_tolerance(LowerArmPath.tolerance)


        LowerArmPath.__MG.set_pose_target(self.pose_quat)
        self.start_state = self.__ROBOT.get_current_state()
        LowerArmPath.__MG.set_start_state_to_current_state()

        if joints_start_state != None:
            tpl_joints_start_state = tuple(joints_start_state)
            self.start_state.joint_state.position = tpl_joints_start_state[0:4] + self.start_state.joint_state.position[4:8:1] 
            LowerArmPath.__MG.set_start_state(self.start_state)

        self.plan = LowerArmPath.__MG.plan()
        return self.check_plan()

    def plan_cartesian_path(self, eef_step = 0.01, joints_start_state = None):
        rospy.loginfo( "Planning Group : %s",self.__GROUP_NAME )

        LowerArmPath.__MG.set_max_velocity_scaling_factor(LowerArmPath.max_speed)
        LowerArmPath.__MG.set_max_acceleration_scaling_factor(LowerArmPath.max_acceralation)
        LowerArmPath.__MG.set_goal_orientation_tolerance(LowerArmPath.tolerance)
        LowerArmPath.__MG.set_goal_position_tolerance(LowerArmPath.tolerance)

        self.start_state = self.__ROBOT.get_current_state()

        if joints_start_state != None:
            tpl_joints_start_state = tuple(joints_start_state)
            self.start_state.joint_state.position = tpl_joints_start_state[0:4] + self.start_state.joint_state.position[4:8:1]
            
        LowerArmPath.__MG.set_start_state(self.start_state)
        (self.plan , fraction) = LowerArmPath.__MG.compute_cartesian_path(self.pose_quat_list, eef_step, math.pi,False)

        return self.check_plan()
            
    def execute( self, brk):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = self.plan.joint_trajectory
        self.check_plan()

        LowerArmPath.client.send_goal(goal)
        
        if brk == True :
            LowerArmPath.client.wait_for_result()

    def check_plan(self):
        if len(self.plan.joint_trajectory.joint_names) == 0:
            rospy.logerr("Planning has failed.")
            raw_input('Continue ?')
            return False
        else:
            return True

    @staticmethod
    def get_current_joint_value():
        robot_joints = LowerArmPath.__ROBOT.get_current_state()
        return list(robot_joints.joint_state.position[0:4])

    @staticmethod
    def set_max_speed_acceleration(speed, acceleration):
        LowerArmPath.max_speed = speed
        LowerArmPath.max_acceralation = acceleration

    @staticmethod
    def init_path_constraints(joint_no, upper_limit, lower_limit, weight):
        
        joint_constraint = JointConstraint()

        LowerArmPath.path_constraints.name = LowerArmPath.__GROUP_NAME
        
        upper_limit = upper_limit/180.0*math.pi
        lower_limit = lower_limit/180.0*math.pi
        
        joint_constraint.position =  ( upper_limit + lower_limit ) / 2.0
        joint_constraint.tolerance_above = ( upper_limit - lower_limit ) / 2.0
        joint_constraint.tolerance_below = ( upper_limit - lower_limit ) / 2.0
        joint_constraint.weight = weight

        joint_constraint.joint_name = "lower_joint%d" % (joint_no)
        LowerArmPath.path_constraints.joint_constraints.append(joint_constraint)

    @staticmethod
    def is_finish():
        ret = LowerArmPath.client.get_result()
        if ret == None:
            return False
        else:
            return True

    @staticmethod
    def cancel_goal():
        ret = LowerArmPath.client.cancel_all_goals()
        if ret == None:
            return False
        else:
            return True

    #Kawasaki-AS to ROS , euler[deg] to quaternion[rad]
    @staticmethod
    def list_to_pose(self, list):
        pose = Pose()
        list[0] = list[0]/1000
        list[1] = list[1]/1000
        list[2] = list[2]/1000 + 0.961
        list[3] = list[3]/180.0*math.pi
        list[4] = list[4]/180.0*math.pi
        list[5] = list[5]/180.0*math.pi

        pose.position.x = list[0]
        pose.position.y = list[1]
        pose.position.z = list[2]
        
        #Kawasaki-AS euler is zyz
        q = tf.transformations.quaternion_from_euler(list[3], list[4], list[5], "rzyz")
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        return pose

    #Kawasaki-AS to ROS, joints[deg] to joints[rad]
    @staticmethod
    def joints_to_joints(self, joints):
        
        if joints == None: return None

        joints[0] = joints[0]/180.0*math.pi
        joints[1] = joints[1]/180.0*math.pi
        joints[2] = joints[2]/1000
        joints[3] = joints[3]/180.0*math.pi

        return joints

#upper arm path class
class UpperArmPath:
    
    #fix member
    __PLANNER = 'RRTConnectkConfigDefault'
    __GROUP_NAME = "upper_arm"
    __ACTION_NAME = '/duaro_upper_arm_controller/follow_joint_trajectory'
    __MG = moveit_commander.MoveGroupCommander(__GROUP_NAME)
    __ROBOT = moveit_commander.RobotCommander()

    path_constraints = Constraints()

    #set class common parameters
    max_speed = 0.1
    max_acceralation = 0.1
    tolerance = 0.001    
    
    def __init__(self, type, goal_state_list, joints_start_state = None ):
    
        #set joint path_constraints
        UpperArmPath.__MG.set_path_constraints(UpperArmPath.path_constraints)

        #plan
        self.plan = RobotTrajectory()
        self.pose_quat_list = list()
        ################planning###############
        #goal_state_list is 1D list
        if isinstance(goal_state_list[0], list) == False:

            if type == "JOINT":#joints interpolation
 
                if len(goal_state_list) < 5:#goal_state_list is joints list
                    self.joints = copy.deepcopy(goal_state_list)
                    self.start_joints = copy.deepcopy(joints_start_state)
                    self.start_joints = self.joints_to_joints(self,self.start_joints)

                    self.pose = None
                    self.joints = self.joints_to_joints(self,self.joints)

                    self.plan_from_joint(self.start_joints ) 
                else:#goal_state_list is pose list
                    self.pose_oat = copy.deepcopy(goal_state_list)
                    self.joints = None

                    self.start_joints = copy.deepcopy(joints_start_state)
                    self.start_joints = self.joints_to_joints(self,self.start_joints)

                    self.pose_quat = self.list_to_pose(self, self.pose_oat)
                    self.plan_from_pose(self.start_joints)

            elif type == "LINEAR":#linear interpolation

                if len(goal_state_list) < 5:#goal_state_list is joints list
                    #cannot plan linear path from joints
                    return False
                else:#goal_state_list is pose list

                    self.pose_oat = copy.deepcopy(goal_state_list)
                    self.joints = None

                    self.start_joints = copy.deepcopy(joints_start_state)
                    self.start_joints = self.joints_to_joints(self,self.start_joints)

                    self.pose_quat_list.append( self.list_to_pose(self, self.pose_oat) )
                    self.plan_cartesian_path(0.1, self.start_joints)

            else:
                return False

        #goal_state_list is not  1D list
        else:
            if type == "JOINT":#joints interpolation
                if len(goal_state_list[0]) < 5:#goal_state_list is joints list
                    # cannot plan from joint waypoints
                    return False
                else:#goal_state_list is pose list

                    self.pose_oat = copy.deepcopy(goal_state_list)
                    self.joints = None

                    self.start_joints = copy.deepcopy(joints_start_state)
                    self.start_joints = self.joints_to_joints(self,self.start_joints)

                    for i in range(0,len(goal_state_list)):
                        self.pose_quat_list.append( self.list_to_pose(self, self.pose_oat[i]) )

                    self.plan_cartesian_path(1.0, self.start_joints)

            elif type == "LINEAR":#linear interpolation

                if len(goal_state_list[0]) < 5:#goal_state_list is joints list
                    # cannot plan from joint waypoints
                    return False
                else:#goal_state_list is pose list

                    self.pose_oat = copy.deepcopy(goal_state_list)
                    self.joints = None

                    self.start_joints = copy.deepcopy(joints_start_state)
                    self.start_joints = self.joints_to_joints(self,self.start_joints)

                    for i in range(0,len(goal_state_list)):
                        self.pose_quat_list.append( self.list_to_pose(self, self.pose_oat[i]) )

                    self.plan_cartesian_path(0.01, self.start_joints)

            else:
                return False

        # Temporary workaround of planner's issue similar to https://github.com/tork-a/rtmros_nextage/issues/170
        UpperArmPath.__MG.set_planner_id(self.__PLANNER)
        # Allow replanning to increase the odds of a solution
        UpperArmPath.__MG.allow_replanning(True)
        
        UpperArmPath.client = SimpleActionClient(
            UpperArmPath.__ACTION_NAME,
            FollowJointTrajectoryAction)
        
        UpperArmPath.client.wait_for_server()

        self.start_state = self.__ROBOT.get_current_state()

    def plan_from_joint(self, joints_start_state = None):
        rospy.loginfo( "Planning Group : %s",UpperArmPath.__GROUP_NAME )

        UpperArmPath.__MG.set_max_velocity_scaling_factor(UpperArmPath.max_speed)
        UpperArmPath.__MG.set_max_acceleration_scaling_factor(UpperArmPath.max_acceralation)
        UpperArmPath.__MG.set_goal_orientation_tolerance(UpperArmPath.tolerance)
        UpperArmPath.__MG.set_goal_position_tolerance(UpperArmPath.tolerance)
        
        UpperArmPath.__MG.set_joint_value_target(self.joints)
        self.start_state = self.__ROBOT.get_current_state()
        UpperArmPath.__MG.set_start_state_to_current_state()

        if joints_start_state != None:
            tpl_joints_start_state = tuple(joints_start_state)
            self.start_state.joint_state.position = self.start_state.joint_state.position[0:4] + tpl_joints_start_state[0:4] 
            UpperArmPath.__MG.set_start_state(self.start_state)

        self.plan = UpperArmPath.__MG.plan()
        return self.check_plan()

    def plan_from_pose(self, joints_start_state = None):
        rospy.loginfo( "Planning Group : %s",self.__GROUP_NAME )

        UpperArmPath.__MG.set_max_velocity_scaling_factor(UpperArmPath.max_speed)
        UpperArmPath.__MG.set_max_acceleration_scaling_factor(UpperArmPath.max_acceralation)
        UpperArmPath.__MG.set_goal_orientation_tolerance(UpperArmPath.tolerance)
        UpperArmPath.__MG.set_goal_position_tolerance(UpperArmPath.tolerance)


        UpperArmPath.__MG.set_pose_target(self.pose_quat)
        self.start_state = self.__ROBOT.get_current_state()
        UpperArmPath.__MG.set_start_state_to_current_state()

        if joints_start_state != None:
            tpl_joints_start_state = tuple(joints_start_state)
            self.start_state.joint_state.position = self.start_state.joint_state.position[0:4] + tpl_joints_start_state[0:4] 
            UpperArmPath.__MG.set_start_state(self.start_state)

        self.plan = UpperArmPath.__MG.plan()
        return self.check_plan()

    def plan_cartesian_path(self, eef_step = 0.01, joints_start_state = None):
        rospy.loginfo( "Planning Group : %s",self.__GROUP_NAME )

        UpperArmPath.__MG.set_max_velocity_scaling_factor(UpperArmPath.max_speed)
        UpperArmPath.__MG.set_max_acceleration_scaling_factor(UpperArmPath.max_acceralation)
        UpperArmPath.__MG.set_goal_orientation_tolerance(UpperArmPath.tolerance)
        UpperArmPath.__MG.set_goal_position_tolerance(UpperArmPath.tolerance)

        self.start_state = self.__ROBOT.get_current_state()

        if joints_start_state != None:
            tpl_joints_start_state = tuple(joints_start_state)
            self.start_state.joint_state.position = self.start_state.joint_state.position[0:4] + tpl_joints_start_state[0:4] 

        UpperArmPath.__MG.set_start_state(self.start_state)
        (self.plan , fraction) = UpperArmPath.__MG.compute_cartesian_path(self.pose_quat_list, eef_step, math.pi,False)

        return self.check_plan()

    def execute( self, brk):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = self.plan.joint_trajectory
        self.check_plan()
        
        UpperArmPath.client.send_goal(goal)

        
        if brk == True :
            UpperArmPath.client.wait_for_result()

    def check_plan(self):
        if len(self.plan.joint_trajectory.joint_names) == 0:
            rospy.logerr("Planning has failed.")
            raw_input('Continue ?')
            return False
        else:
            return True

    @staticmethod
    def get_current_joint_value():
        robot_joints = UpperArmPath.__ROBOT.get_current_state()
        return list(robot_joints.joint_state.position[4:8:1])
    
    @staticmethod
    def set_max_speed_acceleration(speed, acceleration):
        UpperArmPath.max_speed = speed
        UpperArmPath.max_acceralation = acceleration

    @staticmethod
    def init_path_constraints(joint_no,upper_limit, lower_limit, weight):
        joint_constraint = JointConstraint()

        UpperArmPath.path_constraints.name = UpperArmPath.__GROUP_NAME
        
        upper_limit = upper_limit/180.0*math.pi
        lower_limit = lower_limit/180.0*math.pi
        
        joint_constraint.position =  ( upper_limit + lower_limit ) / 2.0
        joint_constraint.tolerance_above = ( upper_limit - lower_limit ) / 2.0
        joint_constraint.tolerance_below = ( upper_limit - lower_limit ) / 2.0
        joint_constraint.weight = weight

        joint_constraint.joint_name = "upper_joint%d" % (joint_no)
        UpperArmPath.path_constraints.joint_constraints.append(joint_constraint)

    @staticmethod
    def is_finish():
        ret = UpperArmPath.client.get_result()
        if ret == None:
            return False
        else:
            return True

    @staticmethod
    def cancel_goal():
        ret = UpperArmPath.client.cancel_all_goals()
        if ret == None:
            return False
        else:
            return True

    #Kawasaki-AS to ROS , euler to quaternion
    @staticmethod
    def list_to_pose(self, list):
        pose = Pose()
        list[0] = list[0]/1000
        list[1] = list[1]/1000
        list[2] = list[2]/1000 + 0.961 #ROS origin is lower than Kawasaki-AS origin.
        list[3] = list[3]/180.0*math.pi
        list[4] = list[4]/180.0*math.pi
        list[5] = list[5]/180.0*math.pi

        pose.position.x = list[0]
        pose.position.y = list[1]
        pose.position.z = list[2]
        
        #Kawasaki-AS euler is zyz
        q = tf.transformations.quaternion_from_euler(list[3], list[4], list[5], "rzyz")
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        return pose

    #Kawasaki-AS to ROS , euler[deg] to euler[rad]
    @staticmethod
    def list_to_list(self, list): 
        list[0] = list[0]/1000
        list[1] = list[1]/1000
        list[2] = list[2]/1000 + 0.961 #ROS origin is lower than Kawasaki-AS origin.
        list[3] = list[3]/180.0*math.pi
        list[4] = list[4]/180.0*math.pi
        list[5] = list[5]/180.0*math.pi
        return list


    #Kawasaki-AS to ROS, joints[deg] to joints[rad]
    @staticmethod
    def joints_to_joints(self, joints):

        if joints == None: return None

        joints[0] = joints[0]/180.0*math.pi
        joints[1] = joints[1]/180.0*math.pi
        joints[2] = joints[2]/1000
        joints[3] = joints[3]/180.0*math.pi

        return joints

#Both arm pose class
class BothArmsPose:
 
    #fix member
    __PLANNER = 'RRTConnectkConfigDefault'
    __GROUP_NAME = "botharms"
    __ACTION_NAME = '/duaro_lower_arm_controller/follow_joint_trajectory'
    __MG = moveit_commander.MoveGroupCommander(__GROUP_NAME)
    __ROBOT = moveit_commander.RobotCommander()

    path_constraints = Constraints()
    
    def __init__(self):
        #set default parameter
        self.tolerance = 0.001
        self.max_speed = 0.1
        self.max_acceralation = 0.2

        #set joint constraints
        #BothArmsPose.__MG.set_path_constraints(BothArmsPose.path_constraints)

        #Temporary workaround of planner's issue similar to https://github.com/tork-a/rtmros_nextage/issues/170
        BothArmsPose.__MG.set_planner_id(self.__PLANNER)
        # Allow replanning to increase the odds of a solution
        BothArmsPose.__MG.allow_replanning(True)
        
    def plan_and_execute(self, goal_joint):
        rospy.loginfo( "Returning home. ")

        BothArmsPose.__MG.set_max_velocity_scaling_factor(self.max_speed)
        BothArmsPose.__MG.set_max_acceleration_scaling_factor(self.max_acceralation)
        BothArmsPose.__MG.set_goal_orientation_tolerance(self.tolerance)
        BothArmsPose.__MG.set_goal_position_tolerance(self.tolerance)
        BothArmsPose.__MG.set_joint_value_target(self.joints_to_joints(self,goal_joint) )
        BothArmsPose.__MG.go()

    #Kawasaki-AS to ROS, joints[deg] to joints[rad]
    @staticmethod
    def joints_to_joints(self, joints):

        if joints == None: return None

        joints[0] = joints[0]/180.0*math.pi
        joints[1] = joints[1]/180.0*math.pi
        joints[2] = joints[2]/1000
        joints[3] = joints[3]/180.0*math.pi

        joints[4] = joints[4]/180.0*math.pi
        joints[5] = joints[5]/180.0*math.pi
        joints[6] = joints[6]/1000
        joints[7] = joints[7]/180.0*math.pi

        return joints
