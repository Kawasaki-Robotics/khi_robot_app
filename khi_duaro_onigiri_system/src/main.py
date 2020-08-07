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
import yaml
import moveit_commander
import duaro
import device
import IO
import set_work
import sys

from control_msgs.msg import (FollowJointTrajectoryAction, FollowJointTrajectoryGoal)

def main():

    rospy.init_node('duaro')
    args = sys.argv
    default_param = False
    
    if args[1] == "-s":
        controller = "GAZEBO"#GAZEBO or REAL
        rospy.loginfo("Stub IO mode.")
    elif args[1] == "-r":
        controller = "REAL"#GAZEBO or REAL
        rospy.loginfo("Real robot mode.")
    
    if args[2] == "-d":
        rospy.loginfo("Setting default parameters. App:ROS, Speed:100%")
        default_param = True
    else:
        default_param = False

    if args[3].isdigit():
        rospy.loginfo("Setting onigiri num. Divide by 5 and cut off the remainder")
        sum_onigiri_num = (int(args[3])//5 )* 5
        rospy.loginfo("Setting default onigiri num : %d", sum_onigiri_num)
    else:
        rospy.loginfo("Setting default onigiri num : 50")
        sum_onigiri_num = 50

    if args[4].isdigit():
        rospy.loginfo("Setting cycle loop num: %d",int(args[4]))
        cycle_num = int(args[4])
    else:
        rospy.loginfo("Setting cycle loop num : 10000")
        cycle_num = 1000

    app = "ROS"
    as_lower_program = 'oma_main'
    as_upper_program = 'osl_main'
    as_pc_program1 = 'autostart.pc'
    as_pc_program7 = "autostart7.pc"
    as_pc_program8 = 'autostart8.pc'
    as_signal = 2258
    as_emergency_signal = 2259

    cmd_service = IO.CmdService(controller)
    
    if (controller=="REAL"):
        cmd_service.set_status('restart','ACTIVE')

    rviz_util = IO.RvizUtil()
    device_info = device.InfoControl(sum_onigiri_num)
    rb_ctl = duaro.Control(device_info,controller,cycle_num)

    app_continue = True
    
    if default_param == False:

        app = "ROS"
        rb_ctl.control.to_ROS_CONTROL()
        is_switch = False

        val = raw_input('Set ROS control speed [%]>> ')
        speed = int(val)
        if speed > 100:
            speed = 100
        elif speed < 1:
            speed = 1
    else:
        app = "ROS"
        rb_ctl.control.to_ROS_CONTROL()
        is_switch = False
        speed = 100
        
    #plan all path 
    rb_ctl.plan_all(speed)
    raw_input('Enter to Start')
    
    while app_continue == True:
        rospy.loginfo('App: ' + app)
        if app == "ROS":
            if (controller=="REAL"):
                cmd_service.set_status('restart','ACTIVE')
                
            rviz_util.show_text('Mode: ROS',-2.5, 1.0, 1.0)

            ret = rb_ctl.cycle_move(is_switch)
            try:
                while (not rospy.is_shutdown() ) and ret == True :
                    ret = rb_ctl.cycle_move(is_switch)
            except KeyboardInterrupt:
                pass

            if is_switch == True:
                app = 'AS'
                rb_ctl.control.to_AS_CONTROL()
        
        elif app == "AS":
            # Hold ROS
            cmd_service.execute_service('driver','set_signal -' + str(as_signal))
            ret = cmd_service.set_status('hold','HOLDED')
            if ret == False:
                rospy.logerr('Need error handling')

            rviz_util.show_text('Mode: AS',-2.5, 1.0, 1.0)

            # Execute prgrams
            ret = cmd_service.execute_as_pc_program(1,as_pc_program1)
            if ret == False:
                rospy.logerr('Need error handling')
            ret = cmd_service.execute_as_pc_program(7,as_pc_program7,False)
            if ret == False:
                rospy.logerr('Need error handling')
            ret = cmd_service.execute_as_pc_program(8,as_pc_program8,False)
            if ret == False:
                rospy.logerr('Need error handling')
            ret = cmd_service.execute_as_robot_program(as_lower_program,as_upper_program)
            if ret == False:
                rospy.logerr('Need error handling')

            # Monitoring
            do_continue = True
            while do_continue:
                 ret = cmd_service.execute_service('driver','get_signal ' + str(as_signal),info=False)
                 if ret.cmd_ret == '-1':
                    break

            # Hold programs
            ret = cmd_service.hold_as_robot_program()
            if ret == False:
                rospy.logerr('Need error handling')

            if is_switch == True:
                app = 'ROS'
                rb_ctl.control.to_ROS_CONTROL_RESET()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
