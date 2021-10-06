#!/usr/bin/env python

import py_trees as pt
import py_trees_ros as ptr
import rospy
from behaviours_student import *
from reactive_sequence import RSequence

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		# tuck the arm
		b0 = tuckarm()

		# Move head down to detect the cube
		b1 = movehead("down")

		# Puckup the cube
		b2 = pickup()

		# Move to next table
		b3 = moveToTable()

		b4 = place()

		# TODO check if well placed
		

		# become the tree
		tree = RSequence(name="Main sequence", children=[b0, b1, b2, b3, b4])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)	



if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
