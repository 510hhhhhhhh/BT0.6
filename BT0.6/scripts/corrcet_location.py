# !/usr/bin/env python
# -*- coding: utf-8 -*-
#无用



import rospy
from Battle import BattleEnv
from Patrol import Plan
from Following import Follow
from teleop_control import Controller
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from teleop_controller.msg import ShootCmd, ModeSW
import time
import math
import numpy as np
import tf


#启动控制器和环境类
controller = Controller()
env = BattleEnv()
plan =Plan()
follow =Follow()
tflistener = tf.TransformListener()
#定义控制命令
vel = Twist()
nav_goal = env.navgoal
shoot_cmd = ShootCmd()
mode = ModeSW()

# def correct():
#
#      pub = rospy.Publisher('correct_location',Odometry, queue_size=10)
#      rospy.init_node('correct', anonymous=True)
#      rate = rospy.Rate(10) # 10hz
#      while not rospy.is_shutdown():
#          print'well'




if __name__ == '__main__':
     rospy.init_node('correct', anonymous=True)
     pub = rospy.Publisher('correct_location',Odometry, queue_size=10)
     rate = rospy.Rate(10)  # 10hz

     while not rospy.is_shutdown():
        try:
            (trans, quat) = tflistener.lookupTransform('/map', 'robot_0/base_link', rospy.Time(0))
            env.getSelfPoseCallback(trans, quat)
            print(trans)
            print (quat)
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        correct_odom = Odometry()
        correct_odom.pose.pose.position.x
        correct_odom.pose.pose.position.y
        correct_odom.pose.pose.orientation.x
        correct_odom.pose.pose.orientation.y
        correct_odom.pose.pose.orientation.z
        correct_odom.pose.pose.orientation.w

