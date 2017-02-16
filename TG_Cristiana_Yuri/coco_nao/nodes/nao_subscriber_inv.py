#!/usr/bin/env python

# Year: 2016
# Author: Yuri, email: 

# Standard imports
from naoqi import ALProxy
import roslib
roslib.load_manifest('coco_nao')
import rospy

# Related third part imports
import math
from naoqi import motion
import time

# # Personal imports
import utils.nao_utils as n_utils

# ROS services
#from tg_nao_teleoperation.srv import *

#ROS messages
#from tg_nao_teleoperation.msg import NaoFrame_inv, NaoChain, NaoSupPol

class ControlNAO(object):
        
    def StiffnessOn(self, chainName):
        # We use the "Body" name to signify the collection of all joints
        pNames = chainName
        pStiffnessLists = 1.0
        pTimeLists = 1.0
        self.motionProxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

    def get_elapsed_time(self):
        """
        """
        if self.first_time == True:
            self.start_time = time.time()
            self.first_time = False
            return 0.0
        else:
            elapsed_time = time.time() - self.start_time   
            return elapsed_time 
        

    def send_to_initial_pose(self, desired_position):
        """
        """
        # Set NAO in Stiffness On
        self.StiffnessOn("LArm")
        self.StiffnessOn("RArm")


        # Send NAO to Pose Init
        # self.postureProxy.goToPosture(desired_position, 0.5)
        
        time.sleep(1.0)
        
        # self.initial_larm_position = self.motionProxy.getPosition("LArm", motion.SPACE_TORSO, True)
        
    def create_proxies(self):
        # Connect to NAOqi
        try:
            self.motionProxy = ALProxy("ALMotion", self.ip, self.port)
        except RuntimeError, e:
            rospy.logerr("Could not create ALMotion ALProxy: %s", e)
            exit(1)
        
        # Connect to module ALRobotPosture
        try:
            self.postureProxy = ALProxy("ALRobotPosture", self.ip, self.port)
        except Exception, e:
            print "Could not create proxy to ALRobotPosture"
            print "Error was: ", e
            
        # Connect to ALMemory
        try:
            self.memoryProxy = ALProxy("ALMemory", self.ip, self.port)
        except Exception, e:
            print "Could not create proxy to ALMemory"
            print "Error was: ", e
            
    def charge_parameters(self):
        # Change values in launch file, these are the default values
        # For naoqi_ip, we have the options: home, lpci, lara
        self.ip = '10.2.0.2' # IP DO SEU ROBÔ!!! 
        self.port = 9559 #int(rospy.get_param('~nao_port', '9559'))
        
        # Check if parameters are ok
        rospy.loginfo('Parameter %s has value %s',
                       rospy.resolve_name('naoqi_ip'),
                       self.ip)

    def __init__(self):
    
        # ROS inicialization
        rospy.init_node('nao_controller', anonymous=True)
        
        rospy.loginfo("control nao arms is running...")
        
        self.wait_change_support = False
        self.change_axis_mask = False
        self.first_time = True
        
        # Get NAO robot parameters
        self.charge_parameters()
                
        # Connect to NAO modules
        self.create_proxies()

        # Start NAO in a specific position
        self.send_to_initial_pose("StandInit")
        #self.postureProxy.goToPosture("StandInit", 1.0)

        # Subscribes to a topic to receive NAO angles
        #self.listen_to_nao_topics() #achei esse nome ruim... =p

if __name__ == '__main__':
    control_nao = ControlNAO()
    rospy.spin()
    
    rospy.loginfo("control nao arms stopped running.")

    exit(0)  
