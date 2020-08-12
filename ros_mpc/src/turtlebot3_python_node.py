#! /usr/bin/env python3
import rospy
import std_msgs
import sensor_msgs
import geometry_msgs
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion

import numpy as np
import matplotlib as plt
from casadi import *
from casadi.tools import *
import pdb
import sys, os

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))+'/do-mpc/')
# print(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))+'/do-mpc/')
# print(sys.path)
import do_mpc
import pickle
import time


sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))+'/python3/')
from simulation_0_model import simulation_0_model
from simulation_0_mpc import simulation_0_mpc
from simulation_0_simulator import simulation_0_simulator


def quaternion_to_euler(x, y, z, w):
    
        import math
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        return X, Y, Z

class MPC_controller:
    def __init__(self):
        self.iteration = 0
        self.dt = 1
        self.init = [0, 0, 0]
        self.target = [0, 0, 0]
        self._u_con = [0.2, 2.0]
        
        self.isInit = 0

        self.model = simulation_0_model(self.dt, self.target)
        self.mpc = simulation_0_mpc(self.model, self.dt, self._u_con)
        self.simulator = simulation_0_simulator(self.model, self.dt)
        self.estimator = do_mpc.estimator.StateFeedback(self.model)


        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.pub_control = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.x = 0
        self.y = 0
        self.theta = 0
        print('MPC_controller init')

    def printStatus(self):
        print(self.iteration)

    def runMPC(self):
        if self.isInit == 1:
            self.iteration = self.iteration + 1
            print('run mpc', self.iteration)
            x0 = self.simulator.x0
            x0['p_x'] = self.x
            x0['p_y'] = self.y
            x0['p_theta'] = self.theta
            self.mpc.x0 = x0
            self.mpc.set_initial_guess()
            u0 = self.mpc.make_step(x0)
            pub_msg = Twist()
            pub_msg.linear.x = u0[0]
            pub_msg.angular.z = u0[1]
            self.pub_control.publish(pub_msg)
            print('u0[0] is: ', u0[0])
            print('u0[1] is: ', u0[1])
            print('pos x    : ', x0['p_x'])
            print('pos y    : ', x0['p_y'])
            print('pos theta: ', x0['p_theta'])
            
        else:
            self.iteration = self.iteration

    def odom_callback(self, msg):
        self.isInit = 1
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = quaternion_to_euler(orientation_list[0],orientation_list[1],orientation_list[2],orientation_list[3])
        yaw = yaw*(pi/180)
        self.theta = yaw

if __name__ == '__main__':
    rospy.init_node('turtlebot3_python_node')
    myMPC = MPC_controller()
    print('init complete')
    rospy.sleep(1)
    
    
    while not rospy.is_shutdown():
        myMPC.runMPC()

        rospy.sleep(1)