#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import tf
import rospy
import math
import numpy as np
from teleop_controller.msg import EnemyPos
from PIL import Image
import time

if __name__ == '__main__':
    rospy.init_node('gettf')
    rate = rospy.Rate(50)
    map = np.array(Image.open("icra.pgm"))
    enemy_pub = rospy.Publisher('enemy_pos', EnemyPos, queue_size=1)
    enemy_pos = EnemyPos()
    tflistener = tf.TransformListener()
    trans01=[6.0, 3.0, 0]
    quat01 =[0.0, 0.0, 0.0, 0.0]
    T01 = np.mat(tflistener.fromTranslationRotation(trans01,quat01))
    print T01
    while not rospy.is_shutdown():
        try:
            trans0s, quat0s = tflistener.lookupTransform("/robot_0/odom", "/robot_0/base_footprint",rospy.Time(0))
            trans1e, quat1e = tflistener.lookupTransform("/robot_1/odom", "/robot_1/base_footprint", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        T0s = np.mat(tflistener.fromTranslationRotation(trans0s, quat0s))
        T1e = np.mat(tflistener.fromTranslationRotation(trans1e, quat1e))
        Ts0 = T0s.I
        Tse=Ts0*T01*T1e

        T0e = T01 * T1e
        x0s = int(T0s[0, 3] / 0.05) + 20
        y0s = int(T0s[1, 3] / 0.05) + 20
        x0e = int(T0e[0, 3] / 0.05) + 20
        y0e = int(T0e[1, 3] / 0.05) + 20
        A = np.mat([[x0s, 1], [x0e, 1]])
        B = np.mat([[y0s], [y0e]])
        A_com=np.mat([[x0s], [x0e]])
        print A.I*B
        print B.T*A_com

        if (np.linalg.det(A)!=0):
            ab = np.linalg.solve(A, B)  # 检测两点直线y=ax+b
            print ('ab=%s\t\n')%(ab)
        elif(abs(B.T * A_com) <= 100):
            ab=[[0.001],[0.1]]
        elif(abs(B.T * A_com)>10000):
            ab=[[1],[100]]

        a = ab[0, 0]
        b = ab[1, 0]
        # points = round(abs(x0s -x0e)/0.05)
        obstacle = False
        if x0s > x0e:
            t = x0s
            x0s = x0e
            x0e = t

        for x in range(x0s, x0e):
            y = int(a * x + b)
            if map[101 - y, x] < 255:
                obstacle = True
                break
        if obstacle == False:
            enemy_yaw = math.atan2(Tse[1, 3], Tse[0, 3]) * 180 / 3.1416
            enemy_dist = math.sqrt(Tse[0, 3] * Tse[0, 3] + Tse[1, 3] * Tse[1, 3])
            enemy_pos.enemy_yaw = - enemy_yaw
            enemy_pos.enemy_dist = enemy_dist * 1000
            enemy_pos.enemy_pitch = 0
            enemy_pub.publish(enemy_pos)
        else:
            enemy_yaw = 0
            enemy_dist = 0
            enemy_pos.enemy_yaw = - enemy_yaw
            enemy_pos.enemy_dist = enemy_dist
            enemy_pos.enemy_pitch = 0
            enemy_pub.publish(enemy_pos)

        print 'enemy_yaw=%s\tenemy_dist=%s\t\n'%(enemy_yaw,enemy_dist)
        rate.sleep()
