# !/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist, PoseStamped
#from std_msgs.msg import UInt8MultiArray
from teleop_controller.msg import EnemyPos, ShootCmd, ModeSW

import sys, select, termios, tty

class Controller():
    def __init__(self):
        self.msg = """
        Reading from the keyboard  and Publishing to Twist!
        ---------------------------
        Moving around:
           u    i    o
           j    k    l
           m    ,    .
        For Holonomic mode (strafing), hold down the shift key:
        ---------------------------
           U    I    O
           J    K    L
           M    <    >
        t : up (+z) b : down (-z)
        For RM Robot:
        gimbal control:+/-steps by 1 degree
                8
            4       6
                2
        shoot control:
            G-> GUN ON,         T/B +/- fric wheel speed by 100
            5->single shoot     space->continue shoot           0->stop shoot     
        anything else : stop
        q/z : increase/decrease max speeds by 10%
        w/x : increase/decrease only linear speed by 10%
        e/c : increase/decrease only angular speed by 10%
        CTRL-C to quit
        """

        self.moveBindings = {
            'i': (1, 0, 0, 0),
            'o': (1, 0, 0, -1),
            'J': (0, 0, 0, 1),
            'L': (0, 0, 0, -1),
            'u': (1, 0, 0, 1),
            ',': (-1, 0, 0, 0),
            '.': (-1, 0, 0, 1),
            'm': (-1, 0, 0, -1),
            'k': (0, 0, 0, 0),
            'K': (0, 0, 0, 0),
            'O': (1, -1, 0, 0),
            'W': (1, 0, 0, 0),
            'A': (0, 1, 0, 0),
            'D': (0, -1, 0, 0),
            'U': (1, 1, 0, 0),
            'S': (-1, 0, 0, 0),
            '>': (-1, -1, 0, 0),
            'M': (-1, 1, 0, 0),
    #WASD前后左右  JL左右转弯

            # 't': (0, 0, 1, 0),
            # 'b': (0, 0, -1, 0),
        }

        self.speedBindings = {
            'q': (1.1, 1.1, 0),
            'z': (.9, .9, 0),
            'w': (1.1, 1, 0),
            'x': (.9, 1, 0),
            'e': (1, 1.1, 0),
            'c': (1, .9, 0),
            'r': (0, 0, 1),
            'v': (0, 0, -1),
        }

        self.gimbalBindings = {
            '8': (1, 0),
            '2': (-1, 0),
            '4': (0, 1),
            '6': (0, -1),
        }

        self.shootBindings = {
            '5': (1, 0, 1, 0),
            ' ': (0, 1, 1, 0),
            'G': (0, 0, 1, 0),
            'T': (0, 0, 1, 100),
            'B': (0, 0, 1, -100),
            '0': (0, 0, 0, 0),
        }
        self.vel_pub = rospy.Publisher('robot_1/cmd_vel', Twist, queue_size=1)
        #发布云台指向,模拟视觉输出信息
        self.gimbal_pub = rospy.Publisher('enemy_pos', EnemyPos, queue_size=1)
        #射击指令
        self.shoot_pub = rospy.Publisher('shoot_cmd', ShootCmd, queue_size=1)
        #模式切换
        self.modesw_pub = rospy.Publisher('switch_mode', ModeSW, queue_size=1)
        #发布目标点
        self.nav_goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


    def vels(self,speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed, turn)
    def send_goal(self, goal):
        self.nav_goal_pub.publish(goal)

    def send_vel(self, vel):
        self.vel_pub.publish(vel)
    def shoot(self,shoot):
        self.shoot_pub.publish(shoot)
    def mode_switch(self, mode):
        self.modesw_pub.publish(mode)


if __name__ == "__main__":
    teleop = Controller()
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_control_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    gimbalstep = rospy.get_param("~gimbalstep", 1.0)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0
    yaw = 0
    pitch =0
    sc =0
    csc=0
    fwr = 0
    fws = 1500

    #定义三种消息类
    twist = Twist()
    gimbal = EnemyPos()
    shoot = ShootCmd()

    try:
        #print teleop.msg
        print teleop.vels(speed, turn)
        print "gimbal step is: \t", gimbalstep
        print "yaw angle: %s\t\n pitch angle: %s\t\n" % (yaw, pitch)
        print "single shoot: %s\t continue shoot: %s\t gun on: %s\t gun speed: %s\t" %(sc, csc, fwr, fws)
        while (1):
            key = teleop.getKey()#获取键盘按键
            #key='i'
            if key in teleop.moveBindings.keys():#速度按键信息解析
                x = teleop.moveBindings[key][0]
                y = teleop.moveBindings[key][1]
                z = teleop.moveBindings[key][2]

                twist.linear.x = x * speed;
                twist.linear.y = y * speed;
                twist.linear.z = z * speed;
                twist.angular.x = 0;
                twist.angular.y = 0;
                twist.angular.z = th * turn
                th = teleop.moveBindings[key][3]
                teleop.vel_pub.publish(twist)#发布速度信息
            elif key in teleop.speedBindings.keys():#设置底盘速度及云台步进解析
                speed = speed * teleop.speedBindings[key][0]
                turn = turn * teleop.speedBindings[key][1]
                gimbalstep = gimbalstep + teleop.speedBindings[key][2]
                print "gimbal step is: \t", gimbalstep
                print teleop.vels(speed, turn)
                if (status == 14):
                    print msg
                status = (status + 1) % 15
            elif key in teleop.gimbalBindings.keys():#云台角度指令解析
                yaw = yaw + teleop.gimbalBindings[key][1]*gimbalstep
                pitch = pitch + teleop.gimbalBindings[key][0]*gimbalstep

                gimbal.enemy_yaw = yaw#*3.1416/180
                gimbal.enemy_pitch = pitch#*3.1416/180
                gimbal.enemy_dist = 2.0
                teleop.gimbal_pub.publish(gimbal)
                print "yaw angle: %s\t pitch angle: %s\t" % (yaw, pitch)
            elif key in teleop.shootBindings.keys():#射击指令解析
                sc = teleop.shootBindings[key][0]
                csc = teleop.shootBindings[key][1]
                fwr = teleop.shootBindings[key][2]
                fws = fws + teleop.shootBindings[key][3]

                shoot.shoot_cmd = sc
                shoot.c_shoot_cmd = csc
                shoot.fric_wheel_run = fwr
                shoot.fric_wheel_spd = fws
                teleop.shoot_pub.publish(shoot)
                print "single shoot: %s\t continue shoot: %s\t gun on: %s\t gun speed: %s\t" % (sc, csc, fwr, fws)
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                yaw = 0
                pitch = 0
                sc = 0
                csc = 0
                fwr = 0
                fws = 0
                if (key == '\x03'):
                    break


    finally:
        twist = Twist()
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0
        teleop.vel_pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

