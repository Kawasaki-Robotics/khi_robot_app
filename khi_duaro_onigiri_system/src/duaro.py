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
import device
from transitions import Machine
from moveit_commander import PlanningSceneInterface

import duaro_plan
import lower
import upper
import IO
import set_work
import time
import math

class StateMachine(object):
    #define stats
    states=["ROS_CONTROL","ROS_CONTROL_RESET", "AS_CONTROL", "MANUAL_RECOVERY_ERROR"]

    #init
    def __init__(self, name):
        self.name = name
        self.machine = Machine(model=self, states=StateMachine.states, initial="START")


class Control:


    def __init__(self, device_info,controller,input_num_cycle):
        self.lower = lower.Arm(device_info,controller)
        self.upper = upper.Arm(device_info,controller)
        self.device_info = device_info
        self.cur_num_cycle = 0
        self.input_num_cycle = input_num_cycle
        self.operate_mode_checked = "NO"
        self.controller = controller

        #state
        self.control = StateMachine("duAro")

        #khi_command_service
        self.khi_command_service = IO.CmdService(controller)
        self.IO_service = IO.IoService(controller)

        #scene interface
        self.scene = PlanningSceneInterface()
        rospy.sleep(2)   # Waiting for PlanningSceneInterface

        #time
        self.start_time = time.time()
        
        #set no onigiri offset tray
        set_work.change_tray_model(self.scene,onigiri_offset = False)
        
        #show corrent situation
        self.rviz_util = IO.RvizUtil()
    def plan_all(self, speed = 10):
        #go pull pose for planning 
        self.upper.go_pullpose_for_planning()
        #plan all path
        self.lower.plan_all(speed)
        self.upper.plan_all(speed)
    
    def cycle_move(self,switch_app):

        #robot stats update
        if self.control.state == "AS_CONTROL":
            return True

        elif self.control.state == "MANUAL_RECOVERY_ERROR":
            self.error_handling()

        elif self.control.state == "ROS_CONTROL":
            if( self.controller == "REAL"):
                ret = self.khi_command_service.execute_service('driver','get_status')
                if ret.cmd_ret != "ACTIVE":
                    rospy.logerr("Auto recovery has been impossible and Robot stopped. Please recovery in manual.")
                    self.control.to_MANUAL_RECOVERY_ERROR()
                    return True

        elif self.control.state == "ROS_CONTROL_RESET":
            self.device_info.reset_parameters(50)
            self.cur_set_no = 0
            self.operate_mode_checked = "NO"

            self.lower.arm.to_START()
            self.upper.arm.to_START() 
            self.control.to_ROS_CONTROL()
            
            return True

        if self.operate_mode_checked == "NO":
            if self.lower.operate_mode() == True :
                self.demo_mode = "ON"
            else :
                self.demo_mode = "OFF"
            self.operate_mode_checked = "YES"
            self.is_finish_packing = False
                        
        if self.lower.arm.state == "END" and self.upper.arm.state == "END":
            self.finish_time = time.time()
            self.cur_num_cycle += 1
            rospy.loginfo( "[Both Arms]:All End (takt time:%f)",self.finish_time - self.start_time)
            self.device_info.reset_parameters(50)
            self.cur_set_no = 0
            self.operate_mode_checked = "NO"

            #ROS will switch to AS
            if switch_app == True:
                return False
            if self.input_num_cycle <= self.cur_num_cycle:
                rospy.loginfo ("Input the number of cycle set you want to operate.")
                self.input_num_cycle = input('>>')
                self.cur_num_cycle = 0
            if self.input_num_cycle == 0:
                self.lower.arm.to_END() 
                self.upper.arm.to_END()
                return False
            self.lower.arm.to_WAITING()
            self.upper.arm.to_WAITING_FOR_SET()  
            rospy.loginfo( "[Lower Arm]:[Current State]:%s ",self.lower.arm.state )
            rospy.loginfo( "[Upper Arm]:[Current State]:%s ",self.upper.arm.state )

        elif self.lower.arm.state == "ERROR" and self.upper.arm.state == "ERROR":
            rospy.loginfo( "[botharms]:all end")
            rospy.logerr('System malfunction is detected. Please check system.')
            self.control.to_MANUAL_RECOVERY_ERROR()
            return True
        
        else:
            #lower arm's each stats move
            if self.upper.arm.state == "ERROR" :
                self.lower.arm.to_ERROR()
            
            elif self.lower.arm.state == "END" :
                pass
                
            elif self.lower.arm.state == "ERROR" :
                pass

            elif  self.lower.arm.state == "START" :
                self.lower.p_home1.plan_from_joint()
                self.lower.p_home1.execute(True)
                self.lower.arm.to_WAITING()
                self.lower.ctrl_hand("OPEN")
                self.lower.ctrl_cylinder("UP")
                rospy.sleep(0.2)
                rospy.loginfo( "[Lower Arm]:[Current State]:%s ",self.lower.arm.state )
                self.lower.p_home1_to_wait1.execute(False)  

            elif self.lower.arm.state == "WAITING":
                if self.lower.is_finish_current_move() == True :
                    if self.device_info.get_sum_remain_no() == 0:
                        self.lower.arm.to_END()
                        rospy.loginfo( "[Lower Arm]:[Current State]:%s ",self.lower.arm.state )
                    else:
                        self.lower.arm.to_WAITING_FOR_CV()
                        rospy.loginfo( "[Lower Arm]:[Current State]:%s ",self.lower.arm.state )

            elif self.lower.arm.state == "WAITING_FOR_CV":
                if self.lower.work_pick_request() == True or self.demo_mode == "ON" :
                    self.lower.arm.to_PICKING()
                    self.lower.ctrl_hand("CLOSE")
                    rospy.loginfo( "[Lower Arm]:[Current State]:%s ",self.lower.arm.state )
                    self.lower.p_wait1_to_pick1.execute(False)
                    rospy.sleep(0.05)

            elif self.lower.arm.state == "PICKING":
                if self.lower.is_finish_current_move() == True :
                    self.lower.arm.to_DEPARTING()
                    rospy.loginfo( "[Lower Arm]:[Current State]:%s ",self.lower.arm.state )
                    self.lower.p_pick1_to_pick1_up.execute(False)

            elif self.lower.arm.state == "DEPARTING":
                if self.lower.is_finish_current_move() == True :
                    if self.demo_mode == "OFF":
                        if self.lower.hand_check("CLOSE", 5) == True :
                            if self.upper.arm.state == "WAITING_FOR_SET":
                                self.lower.arm.to_PUTTING()        
                                self.lower.p_pick1_up_to_set1_up[self.device_info.get_cur_set_no()].execute(False)
                            else:
                                self.lower.arm.to_RELAY_POINT()
                                self.lower.p_pick1_up_to_relay1.execute(False)
                            rospy.loginfo( "[Lower Arm]:[Current State]:%s ",self.lower.arm.state )
                        else:
                            self.lower.arm.to_CHUCK_OUT()
                            self.dispose_status = "INITIAL"
                            rospy.loginfo( "[Lower Arm]:[Current State]:%s ",self.lower.arm.state )
                            self.lower.p_pick1_up_to_dispose_up.execute(False)
                    else:
                        if self.upper.arm.state == "WAITING_FOR_SET":
                            self.lower.p_pick1_up_to_set1_up[self.device_info.get_cur_set_no()].execute(False)
                            self.lower.arm.to_PUTTING()        
                        else:
                            self.lower.arm.to_RELAY_POINT()
                            self.lower.p_pick1_up_to_relay1.execute(False)
                        rospy.loginfo( "[Lower Arm]:[Current State]:%s ",self.lower.arm.state )
                    #time
                    if self.device_info.get_cur_set_no() == 0:
                        rospy.loginfo( "[Both Arms]:time start")
                        self.start_time = time.time()
            elif self.lower.arm.state == "RELAY_POINT":
                if self.lower.is_finish_current_move() == True :    
                    if self.upper.arm.state == "WAITING_FOR_SET":
                        self.lower.arm.to_PUTTING()
                        rospy.loginfo( "[Lower Arm]:[Current State]:%s ",self.lower.arm.state )
                        self.lower.p_relay1_to_set1_up[self.device_info.get_cur_set_no()].execute(False)

            elif self.lower.arm.state == "PUTTING":
                if self.lower.is_finish_current_move() == True :
                    if self.demo_mode == "OFF":
                        if self.lower.hand_check("CLOSE", 5) == True :
                            self.lower.ctrl_cylinder("DOWN")
                            self.lower.arm.to_PACKING()
                            rospy.loginfo( "[Lower Arm]:[Current State]:%s ",self.lower.arm.state )
                            self.lower.p_set1_up_to_set1[self.device_info.get_cur_set_no()].execute(False)
                        else:
                            self.lower.arm.to_CHUCK_OUT()
                            self.dispose_status = "INITIAL"
                            rospy.loginfo( "[Lower Arm]:[Current State]:%s ",self.lower.arm.state )
                            self.lower.p_set1_up_to_dispose_up[self.device_info.get_cur_set_no()].execute(False)
                    else:
                        self.lower.ctrl_cylinder("DOWN")
                        self.lower.arm.to_PACKING()
                        rospy.loginfo( "[Lower Arm]:[Current State]:%s ",self.lower.arm.state )
                        self.lower.p_set1_up_to_set1[self.device_info.get_cur_set_no()].execute(False)

            elif self.lower.arm.state == "PACKING":
                if self.lower.is_finish_current_move() == True :
                    if self.lower.cylinder_check("DOWN") == False :
                        self.lower.arm.to_ERROR()
                        rospy.loginfo( "[Lower Arm]:[Current State]:%s ",self.lower.arm.state )
                    else:
                        self.lower.ctrl_hand("EXHAUST")
                        self.lower.ctrl_cylinder("UP")
                        rospy.sleep(0.01)
                        self.device_info.putting_in_Tray()
                        self.lower.arm.to_TRAY_DEPARTING()
                        rospy.loginfo( "[Lower Arm]:[Current State]:%s ",self.lower.arm.state )
                        self.lower.p_set1_to_set1_up2[self.device_info.get_cur_set_no()-1].execute(False)

            elif self.lower.arm.state == "TRAY_DEPARTING":
                if self.lower.is_finish_current_move() == True :
                    if self.device_info.get_cur_set_no() >= 10:
                        self.lower.ctrl_hand("OPEN")
                    if self.lower.cylinder_check("UP") == False :
                        self.lower.arm.to_ERROR()
                        rospy.loginfo( "[Lower Arm]:[Current State]:%s ",self.lower.arm.state )
                    else:
                        self.lower.arm.to_WAIT_UPPER_CNTUP()
                        rospy.loginfo( "[Lower Arm]:[Current State]:%s ",self.lower.arm.state )
                        self.lower.p_set1_up2_to_home1[self.device_info.get_cur_set_no()-1].execute(False)
            
            elif self.lower.arm.state == "WAIT_UPPER_CNTUP":
                if self.lower.is_finish_current_move() == True :
                    if self.device_info.get_cur_set_no() < 10:
                        self.lower.ctrl_hand("OPEN")
                    if self.lower.hand_check("OPEN", 0) == True :
                        self.is_finish_packing = True
                        self.lower.arm.to_WAITING()
                        rospy.loginfo( "[Lower Arm]:[Current State]:%s ",self.lower.arm.state )
                        self.lower.p_home1_to_wait1.execute(False)
                    else:
                        self.lower.arm.to_ERROR()
                        rospy.loginfo( "[Lower Arm]:[Current State]:%s ",self.lower.arm.state )

            elif self.lower.arm.state == "CHUCK_OUT":
                if self.lower.is_finish_current_move() == True :
                    if self.dispose_status == "INITIAL" :
                        self.lower.ctrl_hand("OPEN")
                        rospy.sleep(0.15)
                        self.lower.p_home1.plan_from_joint()
                        self.lower.p_home1.execute(False)
                        self.dispose_status = "COME_BACK"
                    elif self.dispose_status == "COME_BACK" :
                        self.lower.arm.to_WAITING()
                        rospy.loginfo( "[Lower Arm]:[Current State]:%s ",self.lower.arm.state )
                        self.lower.p_home1_to_wait1.execute(False)
                        self.dispose_status = "INITIAL"
                    
            #upper arm's each stats move
            if self.lower.arm.state == "ERROR" :
                self.upper.arm.to_ERROR()
            
            elif self.upper.arm.state == "END" :
                pass

            elif self.upper.arm.state == "ERROR" :
                pass
                
            elif self.upper.arm.state == "START" :
                self.upper.ctrl_cylinder("UP")
                self.upper.p_home2.plan_from_joint()
                self.upper.p_home2.execute(True)
                self.flg_waiting_state = "NEUTRAL" #reset status
                self.flg_pulling_state = "NEUTRAL" #reset status
                if self.upper.tray_exist_check() == True :
                    self.upper.arm.to_WAITING()
                else:
                    self.upper.arm.to_WAITING_FOR_TRAY()
                    self.upper.p_home2_to_pull2_up.execute(False)
                rospy.loginfo( "[Upper Arm]:[Current State]:%s ",self.upper.arm.state )

            elif self.upper.arm.state == "WAITING":
                if self.upper.is_finish_current_move() == True :
                    self.cur_set_no = self.device_info.get_cur_set_no()
                    if self.flg_waiting_state == "NEUTRAL": #move1
                        if self.upper.cylinder_check("UP") == False :
                            self.upper.arm.to_ERROR()
                            rospy.loginfo( "[Upper Arm]:[Current State]:%s ",self.upper.arm.state )
                        if self.device_info.get_cur_set_no() == 0:
                            rospy.loginfo( "[Upper Arm]:[Current State]:%s ",self.upper.arm.state )
                            self.upper.p_home2_to_wait2_front_up[self.device_info.get_cur_set_no()].execute(False)
                            self.flg_waiting_state = "FORWARD"
                        else:
                            rospy.loginfo( "[Upper Arm]:[Current State]:%s ",self.upper.arm.state )
                            self.upper.p_wait2_up_to_wait2_front_up[self.device_info.get_cur_set_no()-1].execute(False)
                            self.flg_waiting_state = "FORWARD"
                    elif self.flg_waiting_state == "FORWARD": #move2
                        if self.device_info.get_cur_set_no() > 0 and self.device_info.get_cur_set_no() < 8:
                            self.upper.ctrl_cylinder("DOWN")
                            #rospy.sleep(0.2)
                            if self.upper.cylinder_check("DOWN") == False :
                                self.upper.arm.to_ERROR()
                            self.upper.ctrl_cylinder("EXHAUST")
                        rospy.loginfo( "[Upper Arm]:[Current State]:%s ",self.upper.arm.state )
                        self.upper.p_wait2_front_up_to_wait2_front[self.device_info.get_cur_set_no()].execute(False)
                        self.flg_waiting_state = "APPROACH"
                    elif self.flg_waiting_state == "APPROACH": #move3
                        if self.device_info.get_cur_set_no() > 0 and self.device_info.get_cur_set_no() < 8:
                            if self.upper.cylinder_check("DOWN") == False :
                                self.upper.arm.to_ERROR()
                                rospy.loginfo( "[Upper Arm]:[Current State]:%s ",self.upper.arm.state )
                                self.error_approach = duaro_plan.UpperArmPath("JOINT",self.upper.l_wait2_front_up[self.device_info.get_cur_set_no()])
                                self.error_approach.execute(True)
                            else:
                                rospy.loginfo( "[Upper Arm]:[Current State]:%s ",self.upper.arm.state )
                                self.upper.p_wait2_front_to_wait2[self.device_info.get_cur_set_no()].execute(False)
                                self.upper.arm.to_WAITING_FOR_SET()
                        else:
                            rospy.loginfo( "[Upper Arm]:[Current State]:%s ",self.upper.arm.state )
                            self.upper.p_wait2_front_to_wait2[self.device_info.get_cur_set_no()].execute(False)
                            self.upper.arm.to_WAITING_FOR_SET()
                        self.flg_waiting_state = "NEUTRAL"

            elif self.upper.arm.state == "WAITING_FOR_SET":
                if self.upper.is_finish_current_move() == True and self.is_finish_packing == True:
                    self.is_finish_packing = False
                    self.upper.arm.to_DEPARTING()
                    rospy.loginfo( "[Upper Arm]:[Current State]:%s ",self.upper.arm.state )
                    
            elif self.upper.arm.state == "DEPARTING":
                if self.upper.tray_exist_check() == True :
                    if self.device_info.get_cur_set_no() >= 10:
                        self.upper.arm.to_WAITING_FOR_TRAY()
                        self.flg_pulling_state = "NEUTRAL"
                    else:
                        self.upper.arm.to_WAITING()
                else:
                    self.upper.arm.to_ERROR()
                self.upper.p_wait2_to_wait2_up[self.device_info.get_cur_set_no()-1].execute(False)
                if self.device_info.get_cur_set_no() < 9:
                    self.upper.ctrl_cylinder("UP")
                rospy.loginfo( "[Upper Arm]:[Current State]:%s ",self.upper.arm.state )
                    

            elif self.upper.arm.state == "WAITING_FOR_TRAY":
                if self.upper.is_finish_current_move() == True :
                    if self.device_info.get_cur_set_no() >= 10:
                        self.upper.p_wait2_up_to_pull2_up.execute(False)
                    self.upper.arm.to_TRAY_PULL()
                    if self.upper.cylinder_check("UP") == False :
                        self.upper.arm.to_ERROR()
                    self.upper.complete_to_plc()
                    rospy.loginfo( "[Upper Arm]:[Current State]:%s ",self.upper.arm.state )
                    
            elif self.upper.arm.state == "TRAY_PULL":
                if self.upper.is_finish_current_move() == True :
                    if self.flg_pulling_state == "NEUTRAL":
                        if self.upper.nexttray_exist_check() == True :
                            self.upper.p_pull2_up_to_pull2.execute(False)
                            rospy.loginfo( "[Upper Arm]:[Current State]:%s ",self.upper.arm.state )
                            self.flg_pulling_state = "APPROACH"
                    elif self.flg_pulling_state == "APPROACH":
                        if self.upper.nexttray_pull_check() == True :
                            self.upper.p_pull2_to_pull2_dest.execute(False)
                            self.flg_pulling_state = "COMP_CHECK"
                    elif self.flg_pulling_state == "COMP_CHECK":
                        if self.upper.tray_pull_comp() == True:
                            if self.device_info.get_sum_remain_no() == 0:
                                self.upper.arm.to_END()
                            else:
                                self.cur_set_no = self.device_info.get_cur_set_no()
                                self.upper.arm.to_WAITING_FOR_SET()  
                            rospy.loginfo( "[Upper Arm]:[Current State]:%s ",self.upper.arm.state )
                            self.flg_pulling_state = "NEUTRAL"
            
            return True

    def error_handling(self):

        self.upper.hold()
        self.lower.hold()
        
        rospy.loginfo( "[duAro]:[Current State]:%s ",self.control.state )
        rospy.loginfo("When recovery is complete, push a recovery button")

        if self.lower.arm.state != "WAIT_UPPER_CNTUP" :
            #change tray model for collision avoidance
            set_work.change_tray_model(self.scene,onigiri_offset = True)
            rospy.sleep(1)
        
        #chang system state
        self.rviz_util.show_text('Error Handling',-2.5, 0.8, 0.4)

        #waiting for recovery button
        io_ret_1001 = self.IO_service.get_io_signal("1001")
        io_ret_1002 = self.IO_service.get_io_signal("-1002")
        io_ret_1003 = self.IO_service.get_io_signal("-1003")
        io_ret_1004 = self.IO_service.get_io_signal("1004")
        
        count = 0
        while count < 29:
            for i in range(30):
                rospy.sleep(0.002)
                io_ret_1001 = self.IO_service.get_io_signal("1001")
                io_ret_1002 = self.IO_service.get_io_signal("-1002")
                io_ret_1003 = self.IO_service.get_io_signal("-1003")
                io_ret_1004 = self.IO_service.get_io_signal("1004")
                if io_ret_1001 != True or io_ret_1002 != True or io_ret_1003 != True or io_ret_1004 != True:
                    count = 0
                    i = 0
                    break
                else:
                    count = count + 1
                    rospy.logdebug ("Pushing recovery button count:%d",count)
                        
        #restart
        rospy.sleep(3)

        #change driver state to ACTIVE
        self.check_current_driver_state(self.controller,restart=True)
        
        #return waiting position
        self.lower.ctrl_cylinder("UP")
        self.lower.ctrl_hand("CLOSE")
        
        #down speed and acceleration 
        old_lower_speed = duaro_plan.LowerArmPath.max_speed
        old_lower_acceleration = duaro_plan.LowerArmPath.max_speed
        old_upper_speed = duaro_plan.UpperArmPath.max_speed
        old_upper_acceleration = duaro_plan.UpperArmPath.max_speed

        duaro_plan.LowerArmPath.set_max_speed_acceleration(0.05,0.05)
        duaro_plan.UpperArmPath.set_max_speed_acceleration(0.1,0.1)
        
        #lower return home
        if self.lower.hand_check("OPEN",5) == False:
            self.p_l_out = duaro_plan.LowerArmPath("JOINT", [20.902, 21.856, 149.000, -42.758])#dispose_up

            ret = self.p_l_out.check_plan()
            if  ret == False:
                current_joint = duaro_plan.LowerArmPath.get_current_joint_value()
                current_joint[2] = 0.149
                current_joint =  self.ros_joint_to_as_joint(current_joint)
                self.p_l_zup = duaro_plan.LowerArmPath("JOINT", current_joint)
                if self.check_current_driver_state(self.controller) == False:
                    return True
                self.p_l_zup.execute(True)
                ret = self.p_l_out.plan_from_joint()
                
            while  ret == False:
                ret = self.p_l_out.plan_from_joint()
            
            if self.check_current_driver_state(self.controller) == False:
                return True
            self.p_l_out.execute(True)
            self.lower.ctrl_hand("OPEN")

            rospy.sleep(0.1)

        self.p_l_home = duaro_plan.LowerArmPath("JOINT", [13.660, 78.331, 98.000, -92.191])#home1
        ret = self.p_l_home.check_plan()
        if  ret == False:
            current_joint = duaro_plan.LowerArmPath.get_current_joint_value()
            current_joint[2] = 0.149
            current_joint =  self.ros_joint_to_as_joint(current_joint)
            self.p_l_zup = duaro_plan.LowerArmPath("JOINT", current_joint)
            if self.check_current_driver_state(self.controller) == False:
                return True
            self.p_l_zup.execute(True)
            ret = self.p_l_home.plan_from_joint()
            
        while  ret == False:
            ret = self.p_l_home.plan_from_joint()
        
        if self.check_current_driver_state(self.controller) == False:
            return True
        self.p_l_home.execute(True)
        self.lower.ctrl_hand("OPEN")

        #upper return home
        self.upper.ctrl_cylinder("UP")
        current_joint = duaro_plan.UpperArmPath.get_current_joint_value()

        if(current_joint[2] < 0.07):
            current_joint[2] =  0.07
            current_joint =  self.ros_joint_to_as_joint(current_joint)
            self.p_u_home = duaro_plan.UpperArmPath("JOINT", current_joint)#up z
            if self.check_current_driver_state(self.controller) == False:
                return True
            self.p_u_home.execute(True)
        
        self.p_u_home = duaro_plan.UpperArmPath("JOINT", [190.608, -127.648, 149.000, 27.54])#home2
        if self.check_current_driver_state(self.controller) == False:
            return True
        self.p_u_home.execute(True)

        if self.upper.tray_exist_check() == True :
            if self.device_info.get_cur_set_no() > 9:
                self.upper.arm.to_WAITING_FOR_TRAY()
                self.flg_pulling_state = "NEUTRAL"
                self.device_info.reset_parameters(50)
                self.upper.p_home2_to_pull2_up.execute(True)
            else:
                self.upper.arm.to_WAITING()   
                self.upper.p_home2_to_wait2_front_up[self.device_info.get_cur_set_no()].execute(True)
                self.flg_waiting_state = "FORWARD"
        else:
            self.upper.arm.to_WAITING_FOR_TRAY()
            self.upper.p_home2_to_pull2_up.execute(True)
            self.device_info.reset_parameters(50)

        #Undo speed and acceleration
        duaro_plan.LowerArmPath.set_max_speed_acceleration(old_lower_speed, old_lower_acceleration)
        duaro_plan.UpperArmPath.set_max_speed_acceleration(old_upper_speed, old_upper_acceleration)
        set_work.change_tray_model(self.scene,onigiri_offset = False)
                
        #chnage state
        self.rviz_util.show_text('Mode: ROS',-2.5, 1.0, 1.0)
        self.control.to_ROS_CONTROL()
        self.lower.arm.to_START()
        rospy.loginfo( "[Lower Arm]:[Current State]:%s ",self.lower.arm.state )
        rospy.loginfo( "[Upper Arm]:[Current State]:%s ",self.upper.arm.state )

    def check_current_driver_state(self,controller,restart = False):
        if(controller != "REAL"):
            return True
        ret = self.khi_command_service.execute_service('driver','get_status')
        if restart == True:
            if ret.cmd_ret != "ACTIVE":
                self.khi_command_service.set_status('restart','ACTIVE')
                
                rospy.sleep(2)
                ret = self.khi_command_service.execute_service('driver','get_status')
            
        if ret.cmd_ret != "ACTIVE":
            return False
        else:
            return True
    
    def ros_joint_to_as_joint(self, joints):
    
        joints[0] = joints[0]*180.0/math.pi
        joints[1] = joints[1]*180.0/math.pi
        joints[2] = joints[2]*1000
        joints[3] = joints[3]*180.0/math.pi

        return joints