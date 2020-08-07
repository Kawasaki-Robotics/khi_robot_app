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

class StateMachine(object):
    #define stats
    states=["START", "WAITING", "WAITING_FOR_CV", "PICKING","DEPARTING", "RELAY_POINT","PUTTING","PACKING", "TRAY_DEPARTING", "WAIT_UPPER_CNTUP", "CHUCK_OUT", "ERROR", "END"]

    #init
    def __init__(self, name):
        self.name = name
        self.machine = Machine(model=self, states=StateMachine.states, initial="START")

class Arm:

    #pose
    
    #onigiri pick place from conveyor
    pick1 = [13.304, 82.196, 20.000, -96.300] #joint value
    
    #onigiri wait place above the conveyor
    wait1 = [13.660, 78.331, 20.000, -92.191] #joint value
    
    #home position
    home1 = copy.deepcopy(wait1)
    home1[2] = home1[2] + 78.0

    #approach position of pick1
    pick1_up = copy.deepcopy(pick1)
    pick1_up[2] = pick1_up[2] + 129.0
    
    #midpoint between conveyor and allocating tray
    relay1 = [0.319, 95.419, 149.000, -70.738] #joint value

    #onigiri set position of 1st set in allocating tray
    set1 = [20.223, 134.537, 13.000, -64.198] #joint value
    l_set1 = list()
    
    #onigiri set position of last and one before last set in allocating tray
    setla = [48.554, 82.390, 21.000, -40.485] #joint value
    setbefla = [46.768, 85.778, 21.000, -42.063] #joint value
    
    #onigiri dispose position when error detected
    dispose_up = [20.902, 21.856, 149.000, -42.758] #joint value
    
    #onigiri set position for each set.
    l_set1.append(copy.deepcopy([20.223, 134.537, 13.000, -64.198]) ) #set1
    l_set1.append(copy.deepcopy([24.246, 127.467, 13.000, -61.212]) ) #set2
    l_set1.append(copy.deepcopy([27.601, 121.438, 13.000, -58.542]) ) #set3
    l_set1.append(copy.deepcopy([30.942, 115.349, 13.000, -55.791]) ) #set4
    l_set1.append(copy.deepcopy([34.376, 109.022, 13.000, -52.896]) ) #set5
    l_set1.append(copy.deepcopy([37.958, 102.360, 13.000, -49.816]) ) #set6
    l_set1.append(copy.deepcopy([41.483, 95.756, 13.000, -46.740]) ) #set7
    l_set1.append(copy.deepcopy([45.408, 88.366, 13.000, -43.274]) ) #set8
    l_set1.append(copy.deepcopy([46.768, 85.778, 21.000, -42.063]) ) #set9
    l_set1.append(copy.deepcopy([48.554, 82.390, 21.000, -40.485]) ) #set10
    
    #approach position of set1
    l_set1_up = copy.deepcopy(l_set1)
    l_set1_up2 = copy.deepcopy(l_set1)
    
    for i in range(10):
      if i < 8:
          l_set1_up[i][2] += 136.0
          l_set1_up2[i][2] += (97.0+i)
      elif i == 8:
          l_set1_up[i][2] += 128.0
          l_set1_up2[i][2] += (89.0+i)
      elif i == 9:
          l_set1_up[i][2] += 128.0
          l_set1_up2[i][2] += 128.0

    #status
    arm = StateMachine("lower")

    __GROUP_NAME = "lower_arm"
    __ACTION_NAME = '/duaro_lower_arm_controller/follow_joint_trajectory'

    def __init__(self,device_info,controller):
        #Ioservice
        Arm.io = IO.IoService(controller)   

        #set joint constraints 
        duaro_plan.LowerArmPath.init_path_constraints(1, 55.0, -20.0, 1)
        duaro_plan.LowerArmPath.init_path_constraints(2, 140.0, 15.0, 0.5)
        #duaro_plan.LowerArmPath.init_path_constraints(3, 0.1499, 0.0, 0.1)
        duaro_plan.LowerArmPath.init_path_constraints(4, -35.0, -100.0, 0.5)

        #init_cylinder
        self.ctrl_cylinder("UP")

    def plan_all(self, speed = 10):
        #set speed
        speed = float(speed)/100.0
        duaro_plan.LowerArmPath.set_max_speed_acceleration( speed, speed )
        
        self.p_home1 = duaro_plan.LowerArmPath("JOINT", self.home1 )
        
        self.p_home1_to_wait1 = duaro_plan.LowerArmPath("JOINT", self.wait1, self.home1 )
        
        self.p_wait1_to_pick1 = duaro_plan.LowerArmPath("JOINT", self.pick1, self.wait1 )
        
        self.p_pick1_to_pick1_up = duaro_plan.LowerArmPath("JOINT", self.pick1_up, self.pick1 )

        self.p_pick1_up_to_dispose_up = duaro_plan.LowerArmPath("JOINT", self.dispose_up, self.pick1_up )
        
        self.p_pick1_up_to_relay1 = duaro_plan.LowerArmPath("JOINT", self.relay1, self.pick1_up )

        self.p_relay1_to_set1_up = list()
        
        self.p_set1_up_to_set1 = list()
        self.p_set1_up_to_dispose_up = list()
        self.p_set1_to_set1_up2 = list()
        self.p_set1_up2_to_home1 = list()
        
        self.p_pick1_up_to_set1_up = list()

        for set_no in range(10):
          self.p_relay1_to_set1_up.append(copy.deepcopy(duaro_plan.LowerArmPath("JOINT", self.l_set1_up[set_no], self.relay1)))
          self.p_pick1_up_to_set1_up.append(copy.deepcopy(duaro_plan.LowerArmPath("JOINT", self.l_set1_up[set_no], self.pick1_up)))
          
          self.p_set1_up_to_dispose_up.append(copy.deepcopy(duaro_plan.LowerArmPath("JOINT", self.dispose_up, self.l_set1_up[set_no] )))
          
          self.p_set1_up_to_set1.append(copy.deepcopy(duaro_plan.LowerArmPath("JOINT", self.l_set1[set_no], self.l_set1_up[set_no] )))
          
          self.p_set1_to_set1_up2.append(copy.deepcopy(duaro_plan.LowerArmPath("JOINT", self.l_set1_up2[set_no], self.l_set1[set_no] )))
          
          self.p_set1_up2_to_home1.append(copy.deepcopy(duaro_plan.LowerArmPath("JOINT", self.home1, self.l_set1_up2[set_no] )))
          
          
          #for plannning
          if set_no < 9:
            self.p_home1_to_relay1 = duaro_plan.LowerArmPath("JOINT", self.relay1, self.home1 )
            
    #set/get IO number
    def ctrl_hand(self, stat):
        if(stat=="CLOSE"):
            Arm.io.set_io_signal("49,-50")
        elif(stat=="OPEN"):
            Arm.io.set_io_signal("-49,50")
        elif(stat=="EXHAUST"):
            Arm.io.set_io_signal("-49,50")
            #rospy.sleep(0.05)
            Arm.io.set_io_signal("-49,-50")
    
    def ctrl_cylinder(self, stat):
        if(stat=="UP"):
            Arm.io.set_io_signal("51,-52")
        elif(stat=="DOWN"):
            Arm.io.set_io_signal("-51,52")
    
    def work_pick_request(self):
        if Arm.io.get_io_signal("1097") == True :
            return True
        else:
            return False
            
    def tray_exist_check(self):
        if Arm.io.get_io_signal("1153") == True :
            return True
        else:
            return False
    
    def hand_check(self, stat, count):
        if (stat=="CLOSE"):
            if Arm.io.get_io_signal("1058") == True :
                if count == 1:
                    return True
                if Arm.io.get_io_signal("1057") == True :
                    if count == 2:
                        return True
                    if Arm.io.get_io_signal("1056") == True :
                        if count == 3:
                            return True
                        if Arm.io.get_io_signal("1055") == True :
                            if count == 4:
                                return True
                            if Arm.io.get_io_signal("1054") == True :
                                return True
            return False
        elif (stat=="OPEN"):
            if Arm.io.get_io_signal("-1058") == True :
                if Arm.io.get_io_signal("-1057") == True :
                    if Arm.io.get_io_signal("-1056") == True :
                        if Arm.io.get_io_signal("-1055") == True :
                            if Arm.io.get_io_signal("-1054") == True :
                                return True
            return False
    
    def cylinder_check(self, stat):
        if (stat=="DOWN"):
            for i in range(1000):
                if Arm.io.get_io_signal("1059") == True :
                    return True
                rospy.sleep(0.002)
            return False
        elif (stat=="UP"):
            for i in range(1000):
                if Arm.io.get_io_signal("1060") == True :
                    return True
                rospy.sleep(0.002)
            return False

    def operate_mode(self):
        if Arm.io.get_io_signal("2081") == True :
            return True
        else:
            return False
            
    def is_finish_current_move(self):
        if duaro_plan.LowerArmPath.is_finish() == True :
            return True
        else:
            return False

    def hold(self):
        if duaro_plan.LowerArmPath.cancel_goal() == True :
            return True
        else:
            return False