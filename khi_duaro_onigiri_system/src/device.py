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

from geometry_msgs.msg import Pose



class InfoControl:

    req_all_no = 0 

    def __init__(self, all_no):
        self.reset_parameters(all_no)

    def get_sum_remain_no(self):
        return (self.req_all_no - self.Tray.cur_no)
    
    def get_cur_set_no(self):
        return self.Tray.cur_set_no

    #if putting onigiri set in table, do this method 
    def putting_in_Tray(self):
        self.Tray.countup()
        
    #reset Tray parameters             
    def reset_parameters(self, all_no):
        InfoControl.req_all_no = all_no
        self.Tray = Tray()

#
# ***set_no = set number
# ***_no  = onigiri number
class Tray:
    def __init__( self):
        #calculate a set of onigiri number order
        self.cur_no = 0#current onigiri number
        self.cur_set_no = 0#current set number
        

    def countup(self):
        self.cur_no += 5
        self.cur_set_no += 1 

    #get current total onigiri number
    def get_cur_no(self):
        return self.cur_no
        
    #init Tray prameters
    def reset_tray_parameters(self):
        self.cur_no = 0
        self.cur_set_no = 0    
        