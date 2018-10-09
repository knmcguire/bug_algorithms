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
from argos_bridge.srv import GetCmds
from argos_bridge.srv import GetCmdsResponse
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import Float32

from std_srvs.srv import Empty

import com_bug_controller
import bug_2_controller
import i_bug_controller
import alg_1_controller
import alg_2_controller
import wall_following_controller
import blind_bug_controller
import gradient_bug_controller
import gradient_bug_embedded_controller

import i_bug_2_controller

from geometry_msgs.msg import Twist
from scipy.stats._continuous_distns import beta
import wall_following
import receive_rostopics

class BugAlgorithms:
    cmdVelPub = None
    bug_type="com_bug";
    bug_controller =  com_bug_controller.ComBugController()
    RRT = receive_rostopics.RecieveROSTopic()
    WF=wall_following.WallFollowing()
    reset_bug = False
    random_environment = False;
    odometry=[0,0];
    twist = Twist()
    noise_level = 0.0;
    odometry = PoseStamped()
    odometry_perfect = PoseStamped()
    odometry_x_array = []
    odometry_y_array = []

    odometry_x_per_array = []
    odometry_y_per_array = []
    
    previous_time = 0

    send_stop = False
    
    def getController(self,argument):
        switcher = {
            "com_bug": com_bug_controller.ComBugController(),
            "bug_2": bug_2_controller.Bug2Controller(),
            "i_bug": i_bug_controller.IBugController(),
            "i_bug_2": i_bug_2_controller.IBug2Controller(),
            "alg_1": alg_1_controller.Alg1Controller(),
            "alg_2": alg_2_controller.Alg2Controller(),
            "wf": wall_following_controller.WallFollowController(),
            "blind_bug": blind_bug_controller.BlindBugController(),
            "gradient_bug": gradient_bug_controller.GradientBugController(),
            "gradient_embedded_bug": gradient_bug_embedded_controller.GradientBugController(),

        }

        return switcher.get(argument, False)

    def __init__(self):
        # Subscribe to topics and init a publisher
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        #rospy.Subscriber('proximity', ProximityList, self.RRT.prox_callback,queue_size=100)
        #rospy.Subscriber('rangebearing', RangebearingList, self.RRT.rab_callback,queue_size=100)
        #rospy.Subscriber('position', PoseStamped, self.RRT.pose_callback,queue_size=100)
        rospy.Subscriber('position', PoseStamped, self.RRT.pose_callback,queue_size=10)
        rospy.Subscriber('/tower/position', PoseStamped, self.RRT.pose_callback_tower,queue_size=10)
        rospy.Subscriber('/switch_bug', String, self.switchBug,queue_size=10)
        rospy.Subscriber('/random_environment', Bool, self.random_environment,queue_size=10)
        rospy.Subscriber('/noise_level', Float64, self.noise_level_cb,queue_size=10)
        rospy.Subscriber('RSSI_to_tower', Float32, self.RRT.rssi_tower_callback,queue_size=10)

        rospy.wait_for_service('/start_sim')

        s1 = rospy.Service('get_vel_cmd',GetCmds,self.runStateMachine)
        #s2 = rospy.Service('switch_bug',SwitchBug,self.switchBug)

        try:
            start_sim = rospy.ServiceProxy('/start_sim', StartSim)
	    # Start sim with indoor environment from file (from indoor environment generator package)
            start_sim(4,1,1)
        except rospy.ServiceException as e:
            print "Service call failed: %s"%e

        #full_param_name = rospy.search_param('bug_type')
        self.bug_type = rospy.get_param('bug_algorithms/bug_type')

        self.bug_controller = self.getController(self.bug_type);
        self.bug_controller.__init__()

        if self.bug_controller == False:
            print "Wrong bug type!"
        
        self.send_stop = False


    # Ros loop were the rate of the controller is handled
    def rosLoop(self):

        rospy.spin()
#         rate = rospy.Rate(1000)
#
#         while not rospy.is_shutdown():
#             rospy.wait_for_message("proximity", ProximityList)
#             twist = self.bug_controller.stateMachine(self.RRT);
#             self.cmdVelPub.publish(twist)
#             rate.sleep()

    def switchBug(self,req):
        self.bug_controller = self.getController(req.data);
        self.bug_type = req.data
        self.reset_bug = True
        print req.data
        try:
            start_sim = rospy.ServiceProxy('/start_sim', StartSim)
            if(self.random_environment):
                #start_sim(1,1,1)
                start_sim(4,1,1)
                self.random_environment = False
                print "python, send regenerate environment"
            else:
                start_sim(2,1,1)
                print "python, reset experiment with same environment"


        except rospy.ServiceException as e:
            print "Service call failed: %s"%e
        if self.bug_controller == False:
            print "Wrong bug type!"

    def runStateMachine(self, req):

        rospy.wait_for_service('/stop_sim')
        try:
            stop_sim = rospy.ServiceProxy('/stop_sim', Empty)
        except rospy.ServiceException as e:
            print "Service call failed: %s"%e
            

        self.RRT.prox_callback(req.proxList);
        self.RRT.rab_callback(req.RabList);
        self.RRT.pose_callback(req.PosQuat);
        
        pose_bot = PoseStamped()
        pose_bot = self.RRT.getPoseBot()
        distance_to_tower = math.sqrt(math.pow(4-pose_bot.pose.position.x,2)+math.pow(4-pose_bot.pose.position.y,2))
       # print("distance to tower ", distance_to_tower)
       # print("name space ",rospy.get_namespace())
       # print(req.botID)

        if req.reset or self.reset_bug:
            self.WF.init()
            self.bug_controller.__init__()
            self.reset_bug = False
            self.odometry = PoseStamped()
            self.odometry_perfect = PoseStamped()

            self.odometry_x_array = []
            self.odometry_y_array = []
            self.odometry_x_per_array = []
            self.odometry_y_per_array = []
            
            self.send_stop = False
        
        #print("close to goal ",self.RRT.getUWBRange())
        if (distance_to_tower>1.5 ):
            if self.bug_type == 'alg_1':
                self.twist = self.bug_controller.stateMachine(self.RRT,self.get_odometry_from_commands(0.0),0.0,self.noise_level)
            elif self.bug_type == 'alg_2' :
                self.twist = self.bug_controller.stateMachine(self.RRT,self.get_odometry_from_commands(0.0),0.0,self.noise_level,0.0)
            elif self.bug_type == 'i_bug':
                self.twist = self.bug_controller.stateMachine(self.RRT,self.get_odometry_from_commands(0.0),self.noise_level)
            else:

                self.twist = self.bug_controller.stateMachine(self.RRT,self.get_odometry_from_commands(self.noise_level))
        
        
            self.odometry_x_array.append(self.odometry.pose.position.x)
            self.odometry_y_array.append(self.odometry.pose.position.y)

          #  numpy.savetxt('rel_x.txt',self.odometry_x_array,delimiter=',')
          #  numpy.savetxt('rel_y.txt',self.odometry_y_array,delimiter=',')
            self.get_odometry_from_commands_perfect()
        
            self.odometry_x_per_array.append(self.odometry_perfect.pose.position.x)
            self.odometry_y_per_array.append(self.odometry_perfect.pose.position.y)
            
          #  numpy.savetxt('rel_x_per.txt',self.odometry_x_per_array,delimiter=',')
           # numpy.savetxt('rel_y_per.txt',self.odometry_y_per_array,delimiter=',')            
            return GetCmdsResponse(self.twist)
        else:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            if( self.send_stop is False):
                print "bug has reached goal"
                stop_sim()
                self.send_stop = True
            return GetCmdsResponse(self.twist)


    def get_odometry_from_commands(self,noise):
        current_time=float(self.RRT.getArgosTime())/10
        diff_time = current_time - self.previous_time
        if noise < 0.01:
            noisy_velocity_estimate = self.twist.linear.x*0.35
            noisy_heading = self.RRT.getHeading()
        else:
            noisy_velocity_estimate = numpy.random.normal(self.twist.linear.x*0.35,noise,1);
            noisy_heading =  numpy.random.normal(self.RRT.getHeading(),noise,1);
        self.odometry.pose.position.x = self.odometry.pose.position.x + diff_time*noisy_velocity_estimate*math.cos(noisy_heading)
        self.odometry.pose.position.y = self.odometry.pose.position.y + diff_time*noisy_velocity_estimate*math.sin(noisy_heading)
        self.previous_time = current_time
        return self.odometry
    
    def get_odometry_from_commands_perfect(self):
        noisy_velocity_estimate = self.twist.linear.x*0.035
        self.odometry_perfect.pose.position.x = self.odometry_perfect.pose.position.x + noisy_velocity_estimate*math.cos(self.RRT.getHeading())
        self.odometry_perfect.pose.position.y = self.odometry_perfect.pose.position.y + noisy_velocity_estimate*math.sin(self.RRT.getHeading())
        return self.odometry

    def random_environment(self,req):
        self.random_environment = req.data;

    def noise_level_cb(self,req):
        self.noise_level = req.data;


if __name__ == '__main__':

    time.sleep(2)
    rospy.init_node("bug_algorithms")
    controller = BugAlgorithms()
    
    #rospy.wait_for_service('get_vel_cmd')
    try:
        controller.rosLoop()
    except rospy.ROSInterruptException:
        pass
