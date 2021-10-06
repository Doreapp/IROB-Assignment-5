#!/usr/bin/env python

import numpy as np
from numpy import linalg as LA
from geometry_msgs import msg
import math

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from std_srvs.srv import Empty, SetBool, SetBoolResponse
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

from moveit_msgs.msg import MoveItErrorCodes, Grasp, PickupAction, PickupGoal, PickupResult

from robotics_project.msg import PickUpPoseAction, PickUpPoseGoal, PickUpPoseResult, PickUpPoseFeedback

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

class StateMachine(object):
    def __init__(self):
        
        self.node_name = "Student SM"

        # Access rosparams
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')

        self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')

        self.cube_pose = rospy.get_param(rospy.get_name() + '/cube_pose')

        # Subscribe to topics

        # Wait for service providers
        rospy.wait_for_service(self.mv_head_srv_nm, timeout=30)
        rospy.wait_for_service(self.pick_srv_nm, timeout=30)
        rospy.wait_for_service(self.place_srv_nm, timeout=30)

        # Instantiate publishers
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
        self.aruco_pose_pub = rospy.Publisher(self.aruco_pose_top, PoseStamped, queue_size=10, latch = True)

        # Set up action clients
        rospy.loginfo("%s: Waiting for play_motion action server...", self.node_name)
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
        if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /play_motion action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to play_motion action server", self.node_name)

        rospy.loginfo("%s: Publishing cube pose", self.node_name)

        cube_pose = self.cube_pose.split(',')
        aruco_pose_msg = PoseStamped()
        aruco_pose_msg.header.frame_id = "base_footprint"
        aruco_pose_msg.pose.position.x = float(cube_pose[0])
        aruco_pose_msg.pose.position.y = float(cube_pose[1])
        aruco_pose_msg.pose.position.z = float(cube_pose[2])
        aruco_pose_msg.pose.orientation.x = float(cube_pose[3])
        aruco_pose_msg.pose.orientation.y = float(cube_pose[4])
        aruco_pose_msg.pose.orientation.z = float(cube_pose[5])
        aruco_pose_msg.pose.orientation.w = float(cube_pose[6])
        aruco_pose_msg.header.stamp = rospy.Time.now()
        self.aruco_pose_pub.publish(aruco_pose_msg)


        # Init state machine
        self.state = 0
        rospy.sleep(3)
        self.check_states()


    def check_states(self):
        ERROR_STATE = 10
        while not rospy.is_shutdown() and self.state != 5:
            # State 0: Tuck the arm
            if self.state == 0:
                rospy.loginfo("%s: Tucking the arm...", self.node_name)
                goal = PlayMotionGoal()
                goal.motion_name = 'home'
                goal.skip_planning = True
                self.play_motion_ac.send_goal(goal)
                success_tucking = self.play_motion_ac.wait_for_result(rospy.Duration(100.0))

                if success_tucking:
                    rospy.loginfo("%s: Arm tuck: ", self.play_motion_ac.get_result())
                    self.state += 1
                else:
                    self.play_motion_ac.cancel_goal()
                    rospy.logerr("%s: play_motion failed to tuck arm, reset simulation", self.node_name)
                    self.state = ERROR_STATE

                rospy.sleep(1)
            
            # State 1: Pick the cube
            if self.state == 1:
                try:
                    rospy.loginfo("%s: Picking the cube", self.node_name)
                    pick_srv = rospy.ServiceProxy(self.pick_srv_nm, SetBool)
                    pick_req = pick_srv()
                    
                    if pick_req.success == True:
                        self.state += 1
                        rospy.loginfo("%s: Picking succeded!", self.node_name)
                    else:
                        rospy.loginfo("%s: Picking failed!", self.node_name)
                        self.state = ERROR_STATE

                    rospy.sleep(3)
                
                except rospy.ServiceException as e:
                    print ("Service call to pick server failed: %s"%e)
                    self.state = ERROR_STATE

            # State 2: Turn around 
            if self.state == 2:
                rospy.loginfo("%s: Turning", self.node_name)
                duration = 3.0 # Duration in second
                turn_twist_msg = Twist()
                turn_twist_msg.angular.z = math.pi / duration

                now = rospy.Time.now()
                rate = rospy.Rate(10)

                while rospy.Time.now() < now + rospy.Duration.from_sec(duration):
                    self.cmd_vel_pub.publish(turn_twist_msg)
                    rate.sleep() 
                self.state += 1
            
            # State 3: Move forward
            if self.state == 3:
                rospy.loginfo("%s: Moving forward", self.node_name)
                duration = 3.0 # Duration in second
                walk_twist_msg = Twist()
                walk_twist_msg.linear.x = 1.0 / duration

                now = rospy.Time.now()
                rate = rospy.Rate(10)

                while rospy.Time.now() < now + rospy.Duration.from_sec(duration):
                    self.cmd_vel_pub.publish(walk_twist_msg)
                    rate.sleep() 
                self.state += 1
            
            # Stage 4: Place the cube
            if self.state == 4:
                try:
                    rospy.loginfo("%s: Placing the cube", self.node_name)
                    place_srv = rospy.ServiceProxy(self.place_srv_nm, SetBool)
                    place_req = place_srv()
                    
                    if place_req.success == True:
                        self.state += 1
                        rospy.loginfo("%s: Placing succeded!", self.node_name)
                    else:
                        rospy.loginfo("%s: Placing failed!", self.node_name)
                        self.state = ERROR_STATE

                    rospy.sleep(3)
                
                except rospy.ServiceException as e:
                    print ("Service call to pick server failed: %s"%e)
                    self.state = ERROR_STATE

            # Error handling
            if self.state == ERROR_STATE:
                rospy.logerr("%s: State machine failed. Check your code and try again!", self.node_name)
                return

        rospy.loginfo("%s: State machine finished!", self.node_name)
        return

if __name__ == "__main__":

	rospy.init_node('main_state_machine')
	try:
		#StateMachine()
		StateMachine()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
