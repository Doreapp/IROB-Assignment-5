# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviours needed for the example student solution.


import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse

from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math


class counter(pt.behaviour.Behaviour):

    """
    Returns running for n ticks and success thereafter.
    """

    def __init__(self, n, name):

        rospy.loginfo("Initialising counter behaviour.")

        # counter
        self.i = 0
        self.n = n

        # become a behaviour
        super(counter, self).__init__(name)

    def update(self):

        # increment i
        self.i += 1

        # succeed after count is done
        return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS

class go(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, name, linear, angular):

        rospy.loginfo("Initialising go behaviour.")

        # action space
        #self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_top = "/key_vel"
        #rospy.loginfo(self.cmd_vel_top)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = linear
        self.move_msg.angular.z = angular

        # become a behaviour
        super(go, self).__init__(name)

    def update(self):

        # send the message
        rate = rospy.Rate(10)
        self.cmd_vel_pub.publish(self.move_msg)
        rate.sleep()

        # tell the tree that you're running
        return pt.common.Status.RUNNING

class tuckarm(pt.behaviour.Behaviour):

    """
    Sends a goal to the tuck arm action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising tuck arm behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False
        self.finished = False

        # become a behaviour
        super(tuckarm, self).__init__("Tuck arm!")

    def update(self):

        # already tucked the arm
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to tuck arm if haven't already
        elif not self.sent_goal:

            # send the goal
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():

            # than I'm finished!
            self.finished = True

            rospy.loginfo("Arm tucked.")
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.play_motion_ac.get_result():
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING


class movehead(pt.behaviour.Behaviour):

    """
    Lowers or raisesthe head of the robot.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self, direction):

        rospy.loginfo("Initialising move head behaviour.")

        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # head movement direction; "down" or "up"
        self.direction = direction

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(movehead, self).__init__("Lower head!")

    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.move_head_req = self.move_head_srv(self.direction)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.move_head_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_head_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING


class pickup(pt.behaviour.Behaviour):

    """
    Request to pick up the cube using /pick_srv
    It knows the cube placed thanks to the arco detector
    """

    def __init__(self):

        rospy.loginfo("Initialising pickup behaviour.")

        self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        rospy.wait_for_service(self.pick_srv_nm, timeout=30)

        self.pick_srv = rospy.ServiceProxy(self.pick_srv_nm, SetBool)

        # execution checker
        self.requested = False
        self.finished = False

        # become a behaviour
        super(pickup, self).__init__("Pickup!")

    def update(self):
        # already picked up
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to pick up if haven't already
        elif not self.requested:

            # send the goal
            self.pick_req = self.pick_srv()
            self.requested = True
            return pt.common.Status.RUNNING

        elif self.pick_req.success:
            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.pick_req.success:
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING

class moveToTable(pt.behaviour.Behaviour):

    """
    Move to table:
        Rotate by 180 degrees in 3s
        Then move forward of 1m in 3s
    """

    def __init__(self):

        rospy.loginfo("Initialising moveToTable behaviour")

        # action space
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
        
        # Two steps: turn and move
        self.turned = False
        self.moved = False

        # become a behaviour
        super(moveToTable, self).__init__("Move to table")

    def update(self):
        if not self.turned:
            # Turn 
            turn_twist_msg = Twist()
            turn_twist_msg.angular.z = math.pi/5.0
            
            now = rospy.Time.now()
            rate = rospy.Rate(10)
            
            rospy.loginfo("Turning")
            
            while rospy.Time.now() < now + rospy.Duration.from_sec(5):
                self.cmd_vel_pub.publish(turn_twist_msg)
                rate.sleep() 

            # Stop the robot
            self.cmd_vel_pub.publish(Twist())
            self.turned = True
            rospy.sleep(1)

            return pt.common.Status.RUNNING

        elif not self.moved:
            # Move forward 
            walk_twist_msg = Twist()
            walk_twist_msg.linear.x = 0.9/3.0
            
            now = rospy.Time.now()
            rate = rospy.Rate(10)

            rospy.loginfo("Moving")
            
            while rospy.Time.now() < now + rospy.Duration.from_sec(3):
                self.cmd_vel_pub.publish(walk_twist_msg)
                rate.sleep() 


            # Stop the robot
            self.cmd_vel_pub.publish(Twist())
            self.moved = True

            return pt.common.Status.RUNNING
        
        else:
            return pt.common.Status.SUCCESS

class place(pt.behaviour.Behaviour):

    """
    Request to place the cube using /place_srv
    """

    def __init__(self):

        rospy.loginfo("Initialising place behaviour.")

        self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
        rospy.wait_for_service(self.place_srv_nm, timeout=30)

        self.place_srv = rospy.ServiceProxy(self.place_srv_nm, SetBool)

        # execution checker
        self.requested = False
        self.finished = False

        # become a behaviour
        super(place, self).__init__("Place!")

    def update(self):
        # already placed the cube
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to plae
        elif not self.requested:
            # send the goal
            self.place_req = self.place_srv()
            self.requested = True
            return pt.common.Status.RUNNING

        elif self.place_req.success:
            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.place_req.success:
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING


class cubePlaced(pt.behaviour.Behaviour):

    """
    Check if the cube is well placed.
    Subscribe the topic /aruco_pose_topic and wait for a message, up to 10s
        If receive no message: fail
        If receive a message with the cube
            If the cube Z is too low: fail
            Otherwise: Succeed
    """

    def __init__(self):

        rospy.loginfo("Initialising cubePlaced behaviour.")

        # Wait constants
        self.wait_rate = rospy.Rate(1)
        self.wait_count = 20
        self.count = 0

        # Min z to validate
        self.min_cube_z = 0.40

        self.done = False
        self.started = False

        # become a behaviour
        super(cubePlaced, self).__init__("Check cube position")

    def update(self):
        if not self.started:
            # Start: Subscribe to the topic
            def pose_cb(aruco_pose_msg):
                # Receive a position message
                if self.done: 
                    return 
                rospy.loginfo("Callback detected cube :"+ str(aruco_pose_msg))
                self.pose = aruco_pose_msg
                self.done = True
                
            # Subscribe
            pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
            rospy.Subscriber(pose_top, PoseStamped, pose_cb)
            self.started = True
            return pt.common.Status.RUNNING

        if self.done:
            # Check cube pose
            if self.pose.pose.position.z < self.min_cube_z:
                return pt.common.Status.FAILURE
            else:
                return pt.common.Status.SUCCESS

        elif self.count < self.wait_count:
            rospy.loginfo("Cube detection waiting...")
            # Keep waiting
            self.count+=1
            self.wait_rate.sleep()
            return pt.common.Status.RUNNING
        
        else:
            # Do not detect the cube in time
            return pt.common.Status.FAILURE
