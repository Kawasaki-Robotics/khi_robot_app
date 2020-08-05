#!/usr/bin/env python
# coding:utf-8
#
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

from visualization_msgs.msg import Marker
from khi_robot_msgs.srv import *

class RvizUtil:
    def publish_text(self,pub,text,y,z,scale):
        marker_data = Marker()
        marker_data.header.frame_id = "base_link"
        marker_data.header.stamp = rospy.Time.now()

        marker_data.ns = ""
        marker_data.id = 0

        marker_data.action = Marker.ADD

        marker_data.pose.position.x = 0.0
        marker_data.pose.position.y = y
        marker_data.pose.position.z = z
        

        marker_data.pose.orientation.x=0.0
        marker_data.pose.orientation.y=0.0
        marker_data.pose.orientation.z=scale
        marker_data.pose.orientation.w=0.0

        marker_data.color.r = 1.0
        marker_data.color.g = 0.0
        marker_data.color.b = 0.0
        marker_data.color.a = 1.0

        marker_data.scale.x = 1
        marker_data.scale.y = 0.1
        marker_data.scale.z = 0.3

        marker_data.lifetime = rospy.Duration()

        marker_data.type = Marker.TEXT_VIEW_FACING
        marker_data.text = text

        pub.publish(marker_data)

    def show_text(self,text,y,z,scale):
        pub = rospy.Publisher("visualization_marker", Marker, queue_size = 10)
        rate = rospy.Rate(25)

        for i in range(10):
            self.publish_text(pub,text,y,z,scale)
            rate.sleep()

class CmdService:
    __SERVICE_NAME = "khi_robot_command_service"

    def __init__(self,controller):
        if (controller=="REAL"):
            self.controller = "REAL"
            rospy.wait_for_service(self.__SERVICE_NAME)
            self.khi_robot_command_service = rospy.ServiceProxy(self.__SERVICE_NAME,KhiRobotCmd)
        elif (controller=="GAZEBO"):
            self.controller = "GAZEBO"
            self.khi_robot_command_service = DummyService()


    def execute_service(self,type,cmd,info=True):
        if info == True:
            rospy.loginfo('execute_service(%s,%s)',type,cmd)
        ret = self.khi_robot_command_service(type,cmd)
        if ret.driver_ret != 0:
            rospy.logerr('execute_service %s cmd_ret: %s driver_ret: %s',cmd,ret.cmd_ret,ret.driver_ret)
        if ret.as_ret != 0:
            rospy.logerr('execute_service %s cmd_ret: %s as_ret: %s',cmd,ret.cmd_ret,ret.as_ret)
        return ret

    def set_status(self,cmd,res,info=True):
        timeout = 10

        if info == True:
            rospy.loginfo('set_status(%s,%s)',cmd,res)

        ret = self.khi_robot_command_service('driver',cmd)
        if ret.driver_ret != 0:
            rospy.logerr('driver ' + cmd + ' failed.')
            return False
        for i in range(timeout):
            ret = self.khi_robot_command_service('driver','get_status')
            if ret.cmd_ret == res:
                return True
            elif i >= timeout - 1:
                return False
            rospy.sleep(0.5)
        return False

    def execute_as_robot_program(self,lower_prg,upper_prg,info=True):
        timeout = 10

        if info == True:
            rospy.loginfo('execute_as_robot_program(%s,%s)',lower_prg,upper_prg)

        ret = self.hold_as_robot_program()
        if ret != True:
            return ret

        ret = self.execute_service('as','zpow on')
        self.execute_service('as','execute 1: ' + lower_prg)
        self.execute_service('as','execute 2: ' + upper_prg)
        for i in range(timeout):
            ret = self.execute_service('as','type switch(cs)')
            ret.cmd_ret = ''.join(ret.cmd_ret.splitlines()).strip(' ')
            if int(ret.cmd_ret) == -1:
                break
            elif i >= timeout - 1:
                return False
            rospy.sleep(0.5)

        if info == True:
            rospy.loginfo('execute_as_robot_program(%s,%s) ok.',lower_prg,upper_prg)
        return True

    def execute_as_pc_program(self,no,prg,abort=True,info=True):
        timeout = 10

        if info == True:
            rospy.loginfo('execute_as_pc_program(%d,%s)',no,prg)
        if abort == True:
            ret = self.hold_as_pc_program()
            if ret != True:
                return ret

        self.execute_service('as','pcexecute ' + str(no) + ': ' + prg)

        if info == True:
            rospy.loginfo('execute_as_pc_program(%d,%s) ok.',no,prg)
        return True

    def hold_as_robot_program(self,info=False):
        timeout = 5

        if info == True:
            rospy.loginfo('hold_as_robot_program()')

        for i in range(2):
            self.execute_service('as','hold ' + str(i+1) + ':',info=info)
            rospy.sleep(2)

        for i in range(timeout):
            ret1 = self.execute_service('as','type switch(cs)')
            ret2 = self.execute_service('driver','get_status')
            ret1.cmd_ret = ''.join(ret1.cmd_ret.splitlines()).strip(' ')
            if ret1.cmd_ret.isdigit() and int(ret1.cmd_ret) == 0 and ret2.cmd_ret == 'INACTIVE':
                break
            elif i >= timeout - 1:
                return False
            rospy.sleep(1)
        return True

    def hold_as_pc_program(self,info=False):
        timeout = 5

        for i in range(8):
            self.execute_service('as','pcabort ' + str(i+1) + ':',info=info)

        if info == True:
            rospy.loginfo('hold_as_pc_program()')

        ret = self.execute_service('driver','get_status')
        if ret.cmd_ret == 'ERROR':
            return False

        return True

class DummyService:
    #stub
    def execute_service(self,type, cmd):
        return True

class IoService:
    def __init__(self,controller):
        if (controller=="REAL"):
            self.controller = "REAL"
            self.service = CmdService()
        elif (controller=="GAZEBO"):
            self.controller = "GAZEBO"
            self.service = DummyService()

    def set_io_signal(self,signal):
        api_cmd = 'signal %s' %signal
        ret = self.service.execute_service('as', api_cmd)
        return ret

    def get_io_signal(self,signal):
        int_signal = int(signal)
        abs_signal = abs(int_signal)
        api_cmd = 'get_signal %s' %abs_signal
        ret = self.service.execute_service('driver', api_cmd)
        if self.controller == "REAL":
            if ret.cmd_ret == "-1" and  int_signal > 0 :
                return True
            elif ret.cmd_ret == "0" and  int_signal < 0 :
                return True
            else:
                return False                
        elif self.controller == "GAZEBO":
            return ret
