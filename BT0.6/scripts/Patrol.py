# -*- coding: utf-8 -*-
import rospy
import random
import numpy as np
from Battle import BattleEnv
from geometry_msgs.msg import PoseStamped, Quaternion,Twist, PoseStamped
from nav_msgs.msg import Odometry
from teleop_controller.msg import ShootCmd, EnemyPos
import time
import math
from teleop_control import Controller
from Battle import BattleEnv 


'''controller = Controller()
env = BattleEnv()
nav_goal = env.navgoal'''

class Plan():
    def __init__(self):
        self.count = 0
        pass

    #巡逻路径——发布目标点
    def patrol_path(self,env,controller):
        #中心点
        goal_x = 4.0
        goal_y = 2.5
        goal_yaw = 0
        if env.isActionAvaliable(goal_x, goal_y, goal_yaw):#判断目标点是否可行
            controller.send_goal(env.navgoal)#发送目标
            print 'sending goal.1 successful'     
        else:
            pass 
        time.sleep(5)
        #点2
        goal_x = 7.0
        goal_y = 0.5
        goal_yaw = 0 
        if env.isActionAvaliable(goal_x, goal_y, goal_yaw):#判断目标点是否可行
            controller.send_goal(env.navgoal)#发送目标
            print 'sending goal.2 successful'
        else:
            pass 
        time.sleep(5)
        #点3
        goal_x = 7.4
        goal_y = 1.8
        goal_yaw = 0 
        if env.isActionAvaliable(goal_x, goal_y, goal_yaw):#判断目标点是否可行
            controller.send_goal(env.navgoal)#发送目标
            print 'sending goal.3 successful'
        else:
            pass 
        time.sleep(2)
        #点4
        goal_x = 6.2
        goal_y = 4.4
        goal_yaw = 0  
        if env.isActionAvaliable(goal_x, goal_y, goal_yaw):#判断目标点是否可行
            controller.send_goal(env.navgoal)#发送目标
            print 'sending goal.4 successful' 
        else:
            pass 
        time.sleep(5)
        #点5
        goal_x = 1.0
        goal_y = 4.45
        goal_yaw = 0  
        if env.isActionAvaliable(goal_x, goal_y, goal_yaw):#判断目标点是否可行
            controller.send_goal(env.navgoal)#发送目标
            print 'sending goal.5 successful'
        else:
            pass 
        time.sleep(10)
        #点6
        goal_x = 0.5
        goal_y = 0.5
        goal_yaw = 0
        if env.isActionAvaliable(goal_x, goal_y, goal_yaw):#判断目标点是否可行
            controller.send_goal(env.navgoal)#发送目标
            print 'sending goal.6 successful'
        else:
            pass   
        time.sleep(5)

    #随机生成定点    
    def random_path(self, env, controller):
        goal_xs= np.zeros(10)
        goal_ys= np.zeros(10)
        goal_yaws= np.zeros(10)
        for i in range(0,5):
            up_x =env.lastpose[0] + 1.5
            down_x =env.lastpose[0] -1.5
            up_y =  env.lastpose[1] + 1.5
            down_y =env.lastpose[1] -1.5
            goal_xs[i] = random.uniform(down_x, up_x)
            goal_ys[i] = random.uniform(down_y, up_y)
            goal_yaws[i] = 0
            if i == 0:
                for k in range(0,1):
                    if env.gimbalYawSave > 0:
                        count = env.MyPose['theta'] + 180
                    else:
                        count = env.MyPose['theta'] - 180
                        #if count > 180:
                            #count = count - 360
                    if count > 180:
                        count = -360 + count
                    if count < -180:
                        count = 360 + count

                    print 'MyPose %s' % (env.MyPose['theta'])
                    print 'count %s' % (count)
                    if env.isActionAvaliable(env.MyPose['x'], env.MyPose['y'], count):  # 共转1圈
                        controller.send_goal(env.navgoal)
                        print 'twist----twist----twist----twist----twist----twist----'
                        while(env.MyPose['theta'] - count > 40 and env.enemyNew == False):
                            tin = 1
                            print 'MyPose %s' % (env.MyPose['theta'])
                            #time.sleep(0.5)
                            print 'count %s' % (count)

                    else:
                        pass
                    if (env.enemyNew):
                        #env.enemyNew = False
                        break
                #print 'count %s' % (env.MyPose['theta'])
            if (env.enemyNew):
                #env.enemyNew = False
                break

            if env.isActionAvaliable(goal_xs[i],goal_ys[i],goal_yaws[i]):#判断目标点是否可行
                controller.send_goal(env.navgoal)#发送目标
                print '++++++++++++++sending patrol goal-%s successful+++++++++++' % (i + 1)
                print 'goalx%s\tgoaly%s\t' % (goal_xs[i], goal_ys[i])
                while ((((goal_xs[i] - env.MyPose['x']) * (goal_xs[i] - env.MyPose['x']) + (goal_ys[i] - env.MyPose['y']) * (goal_ys[i] - env.MyPose['y'])) > 0.3) and env.enemyNew == False):
                    tin = 1
            else:
                pass




if __name__ == '__main__':
    plan =Plan()
