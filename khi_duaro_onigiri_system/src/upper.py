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
import duaro_plan
from transitions import Machine
import copy
import IO
from threading import Thread

class StateMachine(object):
    #define status
    states=["START",  "WAITING","WAITING_FOR_SET", "DEPARTING", "WAITING_FOR_TRAY", "TRAY_PULL", "ERROR", "END"]

    #init
    def __init__(self, name):
        self.name = name
        self.machine = Machine(model=self, states=StateMachine.states, initial="START")


class Arm:


    #pose
    
    #home position
    home2 = [190.608, -127.648, 149.000, 27.54]
    
    
    l_wait2 = list()
    l_wait2_front = list()
    l_wait2_front_up = list()
    l_wait2_up = list()
    
    #waiting position until lower arm set onigiri set1
    wait2st_trans = [-240.0, -172.0, 8.0, 90.5, 0.0, 0.0, 0.0]
    wait2st = [200.130, -134.634, 8.000, 25.259]
    #front position of waiting position to pull onigiri set1
    wait2st_front = [189.718, -126.917, 8.000, 27.699]
    #approach position of front position of waiting position set1
    wait2st_front_up = [189.718, -126.917, 149.000, 27.699]
    l_wait2.append(copy.deepcopy(copy.deepcopy(wait2st) ) ) #set1
    l_wait2_front.append(copy.deepcopy(copy.deepcopy(wait2st_front) ) ) #set1
    l_wait2_front_up.append(copy.deepcopy(copy.deepcopy(wait2st_front_up) ) ) #set1
    #approach position of waiting position set1
    l_wait2_up.append(copy.deepcopy(copy.deepcopy(wait2st) ) ) #set1
    l_wait2_up[0][2] += 79.0 #in total 87
    
    #each position in allocating tray from set2 to 10
    l_wait2.append(copy.deepcopy([192.418, -129.093, 25.000, 27.181] ) ) #set2
    l_wait2_front.append(copy.deepcopy([183.742, -121.655, 25.000, 28.348] ) ) #set2
    l_wait2_front_up.append(copy.deepcopy([183.742, -121.655, 87.000, 28.348] ) ) #set2
    l_wait2_up.append(copy.deepcopy([192.418, -129.093, 87.000, 27.181] ) ) #set2
    l_wait2.append(copy.deepcopy([186.250, -123.936, 29.000, 28.196] ) ) #set3
    l_wait2_front.append(copy.deepcopy([178.120, -116.126, 29.000, 28.504] ) ) #set3
    l_wait2_front_up.append(copy.deepcopy([178.120, -116.126, 87.000, 28.504] ) ) #set3
    l_wait2_up.append(copy.deepcopy([186.250, -123.936, 87.000, 28.504] ) ) #set3
    l_wait2.append(copy.deepcopy([180.489, -118.519, 32.000, 28.526] ) ) #set4
    l_wait2_front.append(copy.deepcopy([172.755, -110.328, 32.000, 28.073] ) ) #set4
    l_wait2_front_up.append(copy.deepcopy([172.755, -110.328, 87.000, 28.073] ) ) #set4
    l_wait2_up.append(copy.deepcopy([180.489, -118.519, 87.000, 28.526] ) ) #set4
    l_wait2.append(copy.deepcopy([175.025, -112.837, 35.000, 28.312] ) ) #set5
    l_wait2_front.append(copy.deepcopy([167.561, -104.237, 35.000, 27.174] ) ) #set5
    l_wait2_front_up.append(copy.deepcopy([167.561, -104.237, 87.000, 27.174] ) ) #set5
    l_wait2_up.append(copy.deepcopy([175.025, -112.837, 87.000, 28.312,] ) ) #set5
    l_wait2.append(copy.deepcopy([169.768, -106.875, 38.000, 27.604] ) ) #set6
    l_wait2_front.append(copy.deepcopy([162.459, -97.813, 38.000, 25.854] ) ) #set6
    l_wait2_front_up.append(copy.deepcopy([162.459, -97.813, 87.000, 25.854] ) ) #set6
    l_wait2_up.append(copy.deepcopy([169.768, -106.875, 87.000, 27.604] ) ) #set6
    l_wait2.append(copy.deepcopy([164.634, -100.601, 41.000, 26.460] ) ) #set7
    l_wait2_front.append(copy.deepcopy([157.373, -91.000, 41.000, 24.125] ) ) #set7
    l_wait2_front_up.append(copy.deepcopy([157.373, -91.000, 87.000, 24.125] ) ) #set7
    l_wait2_up.append(copy.deepcopy([164.634, -100.601, 87.000, 26.460] ) ) #set7
    l_wait2.append(copy.deepcopy([159.551, -93.962, 44.000, 24.908] ) ) #set8
    l_wait2_front.append(copy.deepcopy([152.223, -83.708, 44.000, 21.986] ) ) #set8
    l_wait2_front_up.append(copy.deepcopy([152.223, -83.708, 87.000, 21.986] ) ) #set8
    l_wait2_up.append(copy.deepcopy([159.551, -93.962, 87.000, 24.908] ) ) #set8
    l_wait2.append(copy.deepcopy([155.907, -88.962, 10.000, 23.550] ) ) #set9
    l_wait2_front.append(copy.deepcopy([141.299, -67.105, 10.000, 16.311] ) ) #set9
    l_wait2_front_up.append(copy.deepcopy([141.299, -67.105, 87.000, 16.311] ) ) #set9
    l_wait2_up.append(copy.deepcopy([155.907, -88.962, 87.000, 23.550] ) ) #set9
    l_wait2.append(copy.deepcopy([152.218, -83.704, 10.000, 21.986] ) ) #set10
    l_wait2_front.append(copy.deepcopy([141.299, -67.105, 10.000, 16.311] ) ) #set10
    l_wait2_front_up.append(copy.deepcopy([141.299, -67.105, 87.000, 16.311] ) ) #set10
    l_wait2_up.append(copy.deepcopy([152.218, -83.704, 149.000, 21.986] ) ) #set10
    
    #position to pull new allocating tray
    pull2 = [104.706108, -3.079942, 20.000, -11.12616]
    #approach position of pulling position
    pull2_up = copy.deepcopy(pull2)
    pull2_up[2] = 149.0
    #new allocating tray set position after pulling
    pull2_dest = copy.deepcopy(wait2st)
    pull2_dest[2] = 1.0
    pull2_dest_trans = copy.deepcopy(wait2st_trans)
    pull2_dest_trans[2] = 1.0

    #status
    arm = StateMachine("upper")
    
    wait_time = 1

    __GROUP_NAME = "upper_arm"
    __ACTION_NAME = '/duaro_upper_arm_controller/follow_joint_trajectory'

    def __init__(self, device_info, controller):
        #Ioservice
        Arm.io = IO.IoService(controller)

        #set joint constraints 
        duaro_plan.UpperArmPath.init_path_constraints(1, 205.0, 100.0, 3)
        duaro_plan.UpperArmPath.init_path_constraints(2, 0.0, -140.0 , 1)
        # duaro_plan.UpperArmPath.init_path_constraints(3, 0.1499, 0.0, 1)
        duaro_plan.UpperArmPath.init_path_constraints(4, 35.0, -15.0, 1)

    def plan_all(self, speed = 10):
        #set speed
        speed = float(speed)/100.0
        duaro_plan.UpperArmPath.set_max_speed_acceleration( speed, speed )
        
        self.p_home2 = duaro_plan.UpperArmPath("JOINT",self.home2)
        
        self.p_home2_to_pull2_up = duaro_plan.UpperArmPath("JOINT", self.pull2_up, self.home2)
 
        self.p_home2_to_wait2_front_up = list()
        self.p_wait2_up_to_wait2_front_up = list()
        self.p_wait2_front_up_to_wait2_front = list()
        self.p_wait2_front_to_wait2 = list()
        self.p_wait2_to_wait2_up = list()
        
        for set_no in range(10):
          self.p_home2_to_wait2_front_up.append(copy.deepcopy(duaro_plan.UpperArmPath("JOINT", self.l_wait2_front_up[set_no], self.home2)))
          if set_no > 0:
            self.p_wait2_up_to_wait2_front_up.append(copy.deepcopy(duaro_plan.UpperArmPath("JOINT", self.l_wait2_front_up[set_no], self.l_wait2_up[set_no-1])))
          
          
          self.p_wait2_front_up_to_wait2_front.append(copy.deepcopy(duaro_plan.UpperArmPath("JOINT", self.l_wait2_front[set_no], self.l_wait2_front_up[set_no] )))
          
          self.p_wait2_front_to_wait2.append(copy.deepcopy(duaro_plan.UpperArmPath("JOINT", self.l_wait2[set_no], self.l_wait2_front[set_no] )))
          
          self.p_wait2_to_wait2_up.append(copy.deepcopy(duaro_plan.UpperArmPath("JOINT", self.l_wait2_up[set_no], self.l_wait2[set_no] )))

        self.p_wait2_up_to_pull2_up = duaro_plan.UpperArmPath("JOINT", self.pull2_up, self.l_wait2_up[9])
        
        self.p_pull2_up_to_pull2 = duaro_plan.UpperArmPath("JOINT", self.pull2, self.pull2_up)
        
        self.p_pull2_to_pull2_dest = duaro_plan.UpperArmPath("LINEAR", self.pull2_dest_trans, self.pull2)
        
    def go_pullpose_for_planning(self):
        self.ctrl_cylinder("UP")
        p_pull2_dest = duaro_plan.UpperArmPath("JOINT", self.pull2_dest)
        p_pull2_dest.plan_from_joint()
        p_pull2_dest.execute(True)
        
    #set/get IO number
    def ctrl_cylinder(self, stat):
        if(stat=="UP"):
            Arm.io.set_io_signal("65,-66")
        elif(stat=="DOWN"):
            Arm.io.set_io_signal("-65,66")
        elif(stat=="EXHAUST"):
            Arm.io.set_io_signal("-65,-66")

    def tray_exist_check(self):
        if Arm.io.get_io_signal("1153") == True :
            return True
        else:
            return False

    def nexttray_exist_check(self):
        if Arm.io.get_io_signal("1113") == True :
            if Arm.io.get_io_signal("1111") == True :
                Arm.io.set_io_signal("-111")
                return True
        return False

    def nexttray_pull_check(self):
        if Arm.io.get_io_signal("1103") == True :
            Arm.io.set_io_signal("103,-111")
            if Arm.io.get_io_signal("-1153") == True :
                if Arm.io.get_io_signal("-1154") == True :
                    return True
        return False

    def cylinder_check(self, stat):
        if (stat=="DOWN"):
            for i in range(1000):
                if Arm.io.get_io_signal("1065") == True :
                    return True
                rospy.sleep(0.002)
            return False
        elif (stat=="UP"):
            for i in range(1000):
                if Arm.io.get_io_signal("1066") == True :
                    return True
                rospy.sleep(0.002)
            return False

    def complete_to_plc(self):
        Arm.io.set_io_signal("111")

    def tray_pull_comp(self):
        Arm.io.set_io_signal("-103,104")
        if Arm.io.get_io_signal("-1103") == True :
            Arm.io.set_io_signal("-104")
            return True
        else:
            return False
        
    def is_finish_current_move(self):
        if duaro_plan.UpperArmPath.is_finish() == True :
            return True
        else:
            return False

    def hold(self):
        if duaro_plan.UpperArmPath.cancel_goal() == True :
            return True
        else:
            return False