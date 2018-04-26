#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
import rospy
import math
import tf
from Battle import BattleEnv 
from teleop_control import Controller

import time
import numpy as np

controller = Controller()
env = BattleEnv()

if __name__ == '__main__':
    rospy.init_node('sendgoal')
    if env.isActionAvaliable(1, 2, 50):
        controller.send_goal(env.navgoal)
        print 'go to position x =%s y=%s'
    else:
        pass
    while True:
        hhh = 1
