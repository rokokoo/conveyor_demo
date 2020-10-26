#!/usr/bin/env python

import sys, copy, rospy, moveit_commander
import moveit_msgs.msg, geometry_msgs.msg
import math, tf, numpy as np
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
	all_equal = True

	if type(goal) is list:
		for index in range(len(goal)):
			if abs(actual[index]-goal[index]) > tolerance:
				return False
	elif type(goal) is geometry_msgs.msg.PoseStamped:
		return all_close(goal.pose,actual.pose, tolerance)
	elif type(goal) is geometry_msgs.msg.Pose:
		return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

	return True

class elfin_picker(object):
	def __init__(self):

		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('elfin_picker', anonymous=False)

		robot = moveit_commander.RobotCommander()
		
		scene = moveit_commander.PlanningSceneInterface()

		group_name = "elfin_arm"
		move_group = moveit_commander.MoveGroupCommander(group_name)

		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

		planning_frame = move_group.get_planning_frame()
		# print "========== Planning frame: %s" % planning_frame

		eef_link = move_group.get_end_effector_link()
		#print "========== End effector link: %s" % eef_link

		group_names = robot.get_group_names()
		# print "========== Available planning groups:", group_names

		# print "========== Printing robot state"
		# print robot.get_current_state()
		# print ""

		self.box_name = ''
		self.robot = robot
		self.scene = scene
		self.move_group = move_group
		self.display_trajectory_publisher = display_trajectory_publisher
		self.planning_frame = planning_frame
		self.eef_link = eef_link
		self.group_names = group_names

		self.home_pose = geometry_msgs.msg.Pose()
		self.home_pose.position.x = 0.0
		self.home_pose.position.y = 0.0
		self.home_pose.position.z = 1.0
		q = tf.transformations.quaternion_from_euler(np.deg2rad(0), np.deg2rad(0), np.deg2rad(0))
		self.home_pose.orientation.x = q[0]
		self.home_pose.orientation.y = q[1]
		self.home_pose.orientation.z = q[2]
		self.home_pose.orientation.w = q[3]

		self.start_pose()
		rospy.sleep(rospy.Duration(2,0))
		self.max_movement()

	def start_pose(self):
		self.move_group.set_pose_target(self.home_pose)

		plan = self.move_group.plan()
		
		self.move_group.execute(plan,wait=True)

		self.move_group.clear_pose_targets()

		cur_pose = self.move_group.get_current_pose(self.eef_link).pose
		if all_close(self.home_pose, cur_pose, 0.01):
			print("All in tolerance")
		else:
			print("Not in tolerance!")

	def max_movement(self):
		pose = self.home_pose

		waypoints = []
		
		pose.position.x += 0.6
		pose.position.z -= 0.4

		q = tf.transformations.quaternion_from_euler(np.deg2rad(21), np.deg2rad(90), np.deg2rad(21))
		pose.orientation.x = q[0]
		pose.orientation.y = q[1]
		pose.orientation.z = q[2]
		pose.orientation.w = q[3]

		waypoints.append(copy.deepcopy(pose))
		
		pose.position.y += 0.5

		waypoints.append(copy.deepcopy(pose))

		pose.position.y -= 1

		waypoints.append(copy.deepcopy(pose))

		waypoints.append(waypoints[0])

		(plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

		self.move_group.execute(plan, wait=True)

if __name__ == '__main__':
	elfin_picker()

# <group_state name="home" group="elfin_arm">
#         <joint name="elfin_joint1" value="0" />
#         <joint name="elfin_joint2" value="1.597" />
#         <joint name="elfin_joint3" value="2.61" />
#         <joint name="elfin_joint4" value="0" />
#         <joint name="elfin_joint5" value="-2.56" />
#         <joint name="elfin_joint6" value="0" />
#     </group_state>

 # -0.0074867022751 -0.103442825818 1.03643627725 -0.000172097821113 0.433533611123 5.61834322488e-05
