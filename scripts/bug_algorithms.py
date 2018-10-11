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
    noise_level = 0.4;
    odometry = PoseStamped()
    odometry_perfect = PoseStamped()
    previous_time = 0
    send_stop = False
    
    # select the appropiate controller by the appropiate bug
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

        #Wait for services to begin
        rospy.wait_for_service('/start_sim')
        s1 = rospy.Service('get_vel_cmd',GetCmds,self.runStateMachine)

        try:
            start_sim = rospy.ServiceProxy('/start_sim', StartSim)
            
	    # Start sim with indoor environment from file (from indoor environment generator package)
            start_sim(4,1,1)
        except rospy.ServiceException as e:
            print "Service call failed: %s"%e

        self.bug_type = rospy.get_param('bug_algorithms/bug_type')

        self.bug_controller = self.getController(self.bug_type);
        self.bug_controller.__init__()

        if self.bug_controller == False:
            print "Wrong bug type!"
        
        self.send_stop = False


    # Ros loop were the rate of the controller is handled
    def rosLoop(self):

        rospy.spin()


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
            
        #Send values from service to the rostopic retrieval 
        self.RRT.prox_callback(req.proxList);
        self.RRT.rab_callback(req.RabList);
        self.RRT.pose_callback(req.PosQuat);
        
        # Get the current position of the bot
        pose_bot = PoseStamped()
        pose_bot = self.RRT.getPoseBot()
        
        # Get the distance to the tower
        distance_to_tower = math.sqrt(math.pow(4-pose_bot.pose.position.x,2)+math.pow(4-pose_bot.pose.position.y,2))

        # If the bug algorithm is reset or the bug has been changed, initialize everythong
        if req.reset or self.reset_bug:
            self.WF.init()
            self.bug_controller.__init__()
            self.reset_bug = False
            self.odometry = PoseStamped()
            self.odometry_perfect = PoseStamped()
            self.send_stop = False
            
        # If the bug is not near the tower (yet)
        if (distance_to_tower>1.5 ):
            #Select the bug statemachine based on the launch file
            if self.bug_type == 'alg_1':
                self.twist = self.bug_controller.stateMachine(self.RRT,self.get_odometry_from_commands(0.0),0.0,self.noise_level)
            elif self.bug_type == 'alg_2' :
                self.twist = self.bug_controller.stateMachine(self.RRT,self.get_odometry_from_commands(0.0),0.0,self.noise_level,0.0)
            elif self.bug_type == 'i_bug':
                self.twist = self.bug_controller.stateMachine(self.RRT,self.get_odometry_from_commands(0.0),self.noise_level)
            else:
                self.twist = self.bug_controller.stateMachine(self.RRT,self.get_odometry_from_commands(self.noise_level))
        
            #Save values for the debugging (matlab)
                # Odometry with drift
            numpy.savetxt('rel_x.txt',[self.odometry.pose.position.x],delimiter=',')
            numpy.savetxt('rel_y.txt',[self.odometry.pose.position.y],delimiter=',')
                # Odometry perfect
            self.get_odometry_from_commands_perfect()
            numpy.savetxt('rel_x_per.txt',[self.odometry_perfect.pose.position.x],delimiter=',')
            numpy.savetxt('rel_y_per.txt',[self.odometry_perfect.pose.position.y],delimiter=',') 
            
            #Return the commands to the controller           
            return GetCmdsResponse(self.twist)
        else:
            #Stop the bug and sent a stop signal for the simulator
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            if( self.send_stop is False):
                print "bug has reached goal"
                stop_sim()
                self.send_stop = True
            return GetCmdsResponse(self.twist)

    # Make a (noisy) odometry measurment based on the inputs on the system, based on a guassian, returns an odometry position.
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
    
    # Returns a perfect odometry, for comparison purposes
    def get_odometry_from_commands_perfect(self):
        noisy_velocity_estimate = self.twist.linear.x*0.035
        self.odometry_perfect.pose.position.x = self.odometry_perfect.pose.position.x + noisy_velocity_estimate*math.cos(self.RRT.getHeading())
        self.odometry_perfect.pose.position.y = self.odometry_perfect.pose.position.y + noisy_velocity_estimate*math.sin(self.RRT.getHeading())
        return self.odometry
    
    # Retrieve command if need to make another environment
    def random_environment(self,req):
        self.random_environment = req.data;

    # Retrieve noise level from topic
    def noise_level_cb(self,req):
        self.noise_level = req.data;


if __name__ == '__main__':

    time.sleep(2)
    rospy.init_node("bug_algorithms")
    controller = BugAlgorithms()
    
    try:
        controller.rosLoop()
    except rospy.ROSInterruptException:
        pass
