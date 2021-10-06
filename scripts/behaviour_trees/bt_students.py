#!/usr/bin/env python

import py_trees as pt
import py_trees_ros as ptr
import rospy
from behaviours_student import *
from reactive_sequence import RSequence

from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		# tuck the arm
		b0 = tuckarm()

		b1 = movehead("down")

		b2 = detectcube()

		# become the tree
		tree = RSequence(name="Main sequence", children=[b0, b1, b2])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)	


class detectcube(pt.behaviour.Behaviour):
	
	def __init__(self):
		rospy.loginfo("Initialising detectcube behaviour.")
		self.done = False

		def aruco_pose_cb(aruco_pose_msg):
			print("pose", aruco_pose_msg)
			self.done = True

		aruco_pose_subs = rospy.Subscriber("/detected_aruco_pose", PoseStamped, aruco_pose_cb)
		self.tried = True

		super(detectcube, self).__init__("Detect cube!")
		
	def update(self):
		if(self.done):
			return pt.common.Status.SUCCESS
		else:
			return pt.common.Status.RUNNING

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
