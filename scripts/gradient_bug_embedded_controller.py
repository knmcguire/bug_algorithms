#!/usr/bin/env python

"""
Com_bug_controller
"""
import rospy
import math, random
import numpy
import time
from argos_bridge.msg import Puck
from argos_bridge.msg import PuckList
from argos_bridge.msg import Proximity
from argos_bridge.msg import ProximityList
from argos_bridge.msg import Rangebearing
from argos_bridge.msg import RangebearingList
from geometry_msgs.msg import PoseStamped
from neat_ros.srv import StartSim
import matplotlib.pyplot as plt


from geometry_msgs.msg import Twist
from scipy.stats._continuous_distns import beta
import wall_following
import receive_rostopics

from copy import deepcopy

import sys
sys.path.append('/home/knmcguire/Software/catkin_ws/src/gradient_bug/scripts/bug_algorithms')
import gradient_bug_v1 




class GradientBugController:

    WF=wall_following.WallFollowing()
    RRT = receive_rostopics.RecieveROSTopic()
    GB =  gradient_bug_v1.GradientBugController()

    distance_to_wall = 0;
    first_rotate = True
    direction = 1
    last_bearing = 0
    hitpoint = PoseStamped()
    stateStartTime=0
    state = "ROTATE_TO_GOAL"
    rssi_goal_angle_adjust = 0



    def __init__(self):

        #Get Desired distance from the wall
        self.distance_to_wall=self.WF.getWantedDistanceToWall();
        self.direction = self.WF.getDirectionTurn();
        self.hitpoint = PoseStamped();
        self.first_rotate = True;
        self.last_bearing = 0;
        self.stateStartTime = 0;
        self.state =  "ROTATE_TO_GOAL"
        self.pose_tower = PoseStamped();
        self.first_run = 1
        self.current_UWB_bearing = 2000
        self.last_range = 0.0
        self.current_range = 0.0
        self.last_range_diff = 0.0

        self.last_heading_rate = 2.5;
        self.first_gradient = 1
        self.diff_diff_range = 0.0
        self.diff_range = 0.0

        self.mem_range = 2000;
        self.mem_heading = 1000
        self.heading_before_turning = 0;

        self.rotated_half_once = False
        self.mem_hit_point_range = 2000
        
        #Init embedded gradient bug
        self.GB.init(self.distance_to_wall,self.WF.getMaximumForwardSpeed(),self.WF.getMaximumRotationSpeed())
        
        self.current_UWB_range = 0
        
        self.rssi_goal_angle_adjust = 0



    def stateMachine(self,RRT,odometry):
        
        self.RRT = RRT

        self.current_range = self.RRT.getUWBRange()

        if self.first_gradient:
            self.last_range = self.current_range
            self.first_gradient =0
        self.diff_range = (self.current_range-self.last_range)/100.0
        self.diff_diff_range = self.last_range_diff-self.diff_range;



        range_front = 1000.0
        range_side = 1000.0
        if self.direction is 1:
            range_front=self.RRT.getRangeFrontLeft()
            range_side=self.RRT.getRangeLeft()
        elif self.direction is -1:
            range_front=self.RRT.getRangeFrontRight()
            range_side=self.RRT.getRangeRight()

        bot_pose = PoseStamped();
        bot_pose.pose.position.x = odometry.pose.position.x;
        bot_pose.pose.position.y = odometry.pose.position.y;

        if self.first_run:
            self.bot_init_position = self.RRT.getPoseBot();
            pose_tower_abs = self.RRT.getPoseTower();
            self.pose_tower.pose.position.x = pose_tower_abs.pose.position.x - self.bot_init_position.pose.position.x;
            self.pose_tower.pose.position.y = pose_tower_abs.pose.position.y - self.bot_init_position.pose.position.y;
            self.stateStartTime = deepcopy(self.RRT.getArgosTime())
            if( abs(pose_tower_abs.pose.position.x)> 0.0):
                self.first_run = False


        else:
            rel_x =  self.pose_tower.pose.position.x - bot_pose.pose.position.x  ;
            rel_y =   self.pose_tower.pose.position.y - bot_pose.pose.position.y ;
            theta = self.wrap_pi(-1*self.RRT.getHeading()-self.rssi_goal_angle_adjust);

            rel_loc_x = rel_x*numpy.math.cos(theta)-rel_y*numpy.math.sin(theta)
            rel_loc_y = rel_x*numpy.math.sin(theta)+rel_y*numpy.math.cos(theta)

            self.current_UWB_bearing =  self.wrap_pi(numpy.arctan2(rel_loc_y,rel_loc_x))
            self.current_UWB_range = math.sqrt(math.pow(rel_loc_y,2)+math.pow(rel_loc_x,2))
            


            #self.current_UWB_bearing = self.RRT.getUWBBearing();
        
        
        
        
        # Handle State transition
        if isinstance(self.current_UWB_bearing,numpy.ndarray):
            self.current_UWB_bearing=float(self.current_UWB_bearing[0])
            
        twist, self.rssi_goal_angle_adjust = self.GB.stateMachine(self.RRT.getRealDistanceToWall(),self.RRT.getRangeRight(),self.RRT.getRangeLeft(),
                        self.RRT.getHeading(),self.current_UWB_bearing,self.current_UWB_range, self.RRT.getRSSITower(),self.RRT.getArgosTime()/10,False, self.WF,self.RRT)
        
        




       #self.cmdVelPub.publish(twist)
        self.lastTwist = twist


        self.last_range_diff = deepcopy(self.diff_range)
        self.last_range = deepcopy(self.current_range)

        return twist


    # Transition state and restart the timer
    def transition(self, newState):
        self.state = newState
        self.stateStartTime = self.RRT.getArgosTime()

    # See if a value is within a margin from the wanted value
    def logicIsCloseTo(self, real_value = 0.0, checked_value =0.0, margin=0.05):

        if real_value> checked_value-margin and real_value< checked_value+margin:
            return True
        else:
            return False



    def headingGradientDescentRange(self):
        v = 1

        command = self.last_heading_rate
        print(self.current_range,self.diff_range,self.diff_diff_range)

        if(self.diff_diff_range>0):
            command = -1*self.last_heading_rate

        print('command',command)

#         if abs(self.diff_diff_range)< 0.0005:
#             print "zero command"
#             command = self.diff_diff_range

        #command = 0
        #w = (command + self.last_heading_rate)/2
#         command = 1000*self.diff_diff_range
#
#         if abs(command)>2.5:
#             command = 2.5 * command/abs(command)
#
        w = command;


        self.last_heading_rate = deepcopy(w)

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w

        return twist

    def headingZigZag(self):
        v = 1
        command = -1*self.last_heading_rate
        w = command;
        self.last_heading_rate = deepcopy(w)

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w

        return twist


    def wrap_pi(self, angles_rad):
        return numpy.mod(angles_rad+numpy.pi, 2.0 * numpy.pi) - numpy.pi
